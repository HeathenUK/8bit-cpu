#!/usr/bin/env python3
"""MK1 C Compiler v2 — purpose-built for MK1 hardware.

Knows every MK1 constraint:
- A is the only ALU operand. All arithmetic goes through A.
- stsp clobbers D and A. ldsp_b clobbers A.
- push_imm sets A = pushed value.
- deref/ideref for indirect memory access.
- setz/setnz/setc/setnc for boolean evaluation.
- Variable-length instructions (1 or 2 bytes).

Usage: python3 mk1cc2.py input.c [-o output.asm] [-O]
"""

import os, sys

# ── Deterministic codegen ────────────────────────────────────────────
# Several passes (cross-section abstraction, overlay partitioning,
# knapsack helper selection) iterate over sets/dicts whose key order
# depends on Python's hash randomisation. Two compiles of the same
# source file can produce slightly different byte output — different
# extractions chosen, different placement decisions — which means
# identical-looking tests flip between pass and fail runs. Re-exec with
# a fixed PYTHONHASHSEED so every run is byte-identical.
if os.environ.get('PYTHONHASHSEED') != '0':
    env = dict(os.environ)
    env['PYTHONHASHSEED'] = '0'
    os.execvpe(sys.executable, [sys.executable, __file__] + sys.argv[1:], env)

import re, argparse
from collections import namedtuple

# ── Kernel-state allocator ───────────────────────────────────────────
# Single source of truth for memory offsets used by runtime helpers
# (calibration constants, one-shot guards, kernel counters). Replaces
# the previous pattern of hardcoding magic numbers like `240`, `241`,
# `242` in multiple emission sites and assuming non-collision by
# inspection.
#
# To add a new entry: pick a free offset on the chosen page, register
# it here with `owner` and `description` set, then reference it via
# the convenience alias below. The matching C header for the ESP32
# firmware lives at `mk1_esp32_uploader/src/kernel_state.h` — keep
# the two in sync (only three entries today; trivial to maintain).

KSlot = namedtuple('KSlot', ['page', 'offset', 'size', 'owner', 'description'])

KERNEL_STATE = {
    'calib_blocks': KSlot(
        page=2, offset=0xB0, size=1, owner='__delay_cal',
        description='F/4608 — DS3231 SQW HIGH-phase block count, '
                    'consumed by __delay_Nms and __tone_setup'),
    'tone_precomp_done': KSlot(
        page=2, offset=0xB1, size=1, owner='__tone_precompute',
        description='one-shot guard: non-zero means the note table '
                    'has been converted from ratios to half_periods '
                    'and a second precompute would corrupt it'),
    'overlay_r2c_count': KSlot(
        page=2, offset=0xB2, size=1, owner='__eeprom_r2c_loop',
        description='remaining-bytes counter for sequential I2C → '
                    'code page reads (EEPROM overlay loader)'),
}

# Phase 1 introduced f-string aliases (`K_CALIB_BLOCKS`, etc.) for
# emission sites; Phase 3 made them obsolete by routing all accesses
# through `emit_kstate_load`/`emit_kstate_store` which are page-aware.
# The registry entry is now the single source of truth for both page
# and offset; helper methods fan out the correct opcode pair.

# ── Page-2 partition ─────────────────────────────────────────────────
# Plan-of-record memory layout for page 2. The corpus stack-depth
# analysis (Phase 2) showed the deepest observed depth is 9 B, so the
# Phase-2 64-byte stack reservation was very generous; the layout was
# refactored on 2026-04-25 to give the freed 48 B back to the page-2
# overlay slot, raising it from 128 B to 176 B (more storage room
# for the partitioner's page-2 overlay tier).
#
# Page-2 byte layout (low to high):
#   0x00 .. 0xAF   overlay slot     (176 B; storage for page-2 overlays)
#   0xB0 .. 0xDF   kernel state     ( 48 B; KERNEL_STATE entries +
#                                          manifest+pages from 0xC0)
#   0xE0 .. 0xEF   guard band       ( 16 B; unused — overflow buffer)
#   0xF0 .. 0xFF   stack reserved   ( 16 B; SP grows down from 0xFF)
#
# Sizing rationale:
#   - 16B stack: corpus peak observed depth = 9 B (overlay_dashboard).
#     Static analysis enforces depth ≤ 16 B; deeper programs hard-
#     fail compile (mirrors the overlay-placement guard).
#   - 16B guard: catches a runaway stack before it corrupts kernel
#     state; small enough that the budget loss is trivial.
#   - 48B kstate: 3 fixed slots at 0xB0..0xB2 + manifest+pages at
#     0xC0..0xDF (~5 overlays at 3 B each before overflow into guard).
#   - 176B overlay: 48 B more than the original 128; gives the
#     partitioner's page-2 storage tier substantially more room.

P2_OVERLAY_LO        = 0x00
P2_OVERLAY_HI        = 0xAF
P2_KSTATE_LO         = 0xB0
P2_KSTATE_HI         = 0xDF
P2_GUARD_LO          = 0xE0
P2_GUARD_HI          = 0xEF
P2_STACK_LO          = 0xF0   # stack must not descend below this
P2_STACK_HI          = 0xFF   # SP starts here

P2_OVERLAY_BYTES     = P2_OVERLAY_HI - P2_OVERLAY_LO + 1   # 128
P2_KSTATE_BYTES      = P2_KSTATE_HI  - P2_KSTATE_LO  + 1   #  48
P2_GUARD_BYTES       = P2_GUARD_HI   - P2_GUARD_LO   + 1   #  16
P2_STACK_BUDGET      = P2_STACK_HI   - P2_STACK_LO   + 1   #  64

# Manifest + pages array base within the kstate region. Phase 5 moved
# them out of `section page3_code` (where they squatted at the tail of
# the kernel image, occupying 15-30 B of page-3 post-init) into the
# structured page-2 kstate area. KERNEL_STATE fixed slots fill from
# P2_KSTATE_LO upward; the manifest+pages variable region fills from
# P2_MANIFEST_BASE upward. The gap between fixed-slot top and manifest
# base is reserved for future fixed-slot growth.
P2_MANIFEST_BASE     = 0xC0

# ── Tokenizer (from mk1cc.py) ────────────────────────────────────────

TOKEN_PATTERNS = [
    ('MCOMMENT', r'/\*[\s\S]*?\*/'),
    ('COMMENT',  r'//[^\n]*'),
    ('CHAR',     r"'(?:[^'\\]|\\[nrt0\\'])'"),  # character literals
    ('NUMBER',   r'0[xX][0-9a-fA-F]+|0[bB][01]+|\d+'),
    ('STRING',   r'"(?:[^"\\]|\\.)*"'),
    ('IDENT',    r'[a-zA-Z_]\w*'),
    # Longer-first: <<= >>= before << >>; /= %= with the rest.
    ('OP2',      r'<<=|>>=|&&|\|\||[+\-*/%&|^]=|==|!=|<=|>=|<<|>>|\+\+|\-\-'),
    ('OP1',      r'[+\-*/%&|^~!<>=(){},;\[\]?:]'),
    ('SKIP',     r'[ \t]+'),
    ('NEWLINE',  r'\n'),
    ('MISMATCH', r'.'),
]
KEYWORDS = {'if', 'else', 'while', 'for', 'do', 'return', 'unsigned', 'char',
            'void', 'int', 'switch', 'case', 'default', 'break', 'continue',
            'u8', 'u16', 'eeprom', 'sizeof', 'typedef', 'static'}

class Token:
    def __init__(self, type, value, line):
        self.type, self.value, self.line = type, value, line

def preprocess(source):
    """Simple `#define NAME value` substitution. Lines starting with `#define`
    are stripped and their body replaces later whole-word occurrences.
    Blank lines are inserted for stripped directives so error messages keep
    the right line numbers.
    Function-like macros (`#define F(x) ...`) are NOT supported."""
    macros = {}
    out = []
    for line in source.split('\n'):
        stripped = line.lstrip()
        if stripped.startswith('#define'):
            parts = stripped.split(None, 2)
            if len(parts) >= 2:
                name = parts[1]
                value = parts[2] if len(parts) > 2 else '1'
                # strip trailing comments
                value = re.sub(r'//.*$|/\*[\s\S]*?\*/', '', value).rstrip()
                macros[name] = value
            out.append('')
            continue
        if stripped.startswith('#'):
            out.append('')          # silently ignore other directives
            continue
        # Iterate until stable so macros referencing other macros fully expand.
        for _ in range(8):   # plenty; real programs rarely chain this deep
            prev = line
            for name, value in macros.items():
                line = re.sub(rf'\b{re.escape(name)}\b', value, line)
            if line == prev:
                break
        out.append(line)
    return '\n'.join(out)


def tokenize(source):
    source = preprocess(source)
    tokens = []
    line = 1
    pat = '|'.join(f'(?P<{n}>{p})' for n, p in TOKEN_PATTERNS)
    for m in re.finditer(pat, source):
        kind, value = m.lastgroup, m.group()
        if kind == 'NEWLINE': line += 1
        elif kind in ('SKIP', 'COMMENT', 'MCOMMENT'): line += value.count('\n')
        elif kind == 'MISMATCH': raise SyntaxError(f'Line {line}: unexpected {value!r}')
        elif kind == 'STRING': tokens.append(Token('STRING', value, line))
        elif kind == 'CHAR':
            # Convert character literal to integer
            ch = value[1:-1]  # strip quotes
            if ch.startswith('\\'):
                esc = {'n': 10, 'r': 13, 't': 9, '0': 0, '\\': 92, "'": 39}
                tokens.append(Token('NUMBER', esc.get(ch[1], ord(ch[1])), line))
            else:
                tokens.append(Token('NUMBER', ord(ch), line))
        elif kind == 'IDENT' and value in KEYWORDS: tokens.append(Token(value, value, line))
        elif kind == 'NUMBER': tokens.append(Token('NUMBER', int(value, 0), line))
        else: tokens.append(Token(kind, value, line))
    tokens.append(Token('EOF', '', line))
    return tokens

# ── Parser ────────────────────────────────────────────────────────────

class Parser:
    def __init__(self, tokens):
        self.tokens, self.pos = tokens, 0
        self.typedefs = {}           # alias → type string ('u8'/'u16')
        self.globals_sizes = {}      # name → size in bytes (arrays: length)

    def peek(self): return self.tokens[self.pos]
    def advance(self):
        t = self.tokens[self.pos]; self.pos += 1; return t
    def expect(self, tv):
        t = self.peek()
        if t.type == tv or t.value == tv: return self.advance()
        raise SyntaxError(f'Line {t.line}: expected {tv!r}, got {t.value!r}')
    def match(self, tv):
        t = self.peek()
        if t.type == tv or t.value == tv: return self.advance()
        return None

    def parse_program(self):
        functions, globals_ = [], []
        while self.peek().type != 'EOF':
            # Top-level `typedef <type> <alias>;` — records alias, no codegen.
            if self.match('typedef'):
                typ = self.parse_type()
                alias = self.expect('IDENT').value
                self.expect(';')
                self.typedefs[alias] = typ
                continue
            # Check for 'eeprom' storage qualifier before type
            storage = 'eeprom' if self.match('eeprom') else 'ram'
            typ = self.parse_type()
            name = self.expect('IDENT').value
            if self.match('('):
                fn = self.parse_function(typ, name)
                if fn: functions.append(fn)
            elif self.match('='):
                val = self.expect('NUMBER').value; self.expect(';')
                globals_.append((name, val, storage))
                self.globals_sizes[name] = 2 if typ == 'u16' else 1
            elif self.match('['):
                if self.peek().type == 'NUMBER':
                    size = self.expect('NUMBER').value
                else:
                    size = None  # infer from initializer
                self.expect(']')
                if self.match('='):
                    self.expect('{')
                    vals = []
                    while not self.match('}'):
                        vals.append(self.expect('NUMBER').value & 0xFF)
                        self.match(',')  # optional trailing comma
                    self.expect(';')
                    if size is not None and len(vals) < size:
                        vals.extend([0] * (size - len(vals)))
                    globals_.append((name, vals, storage))
                    self.globals_sizes[name] = size if size is not None else len(vals)
                else:
                    self.expect(';')
                    if size is None:
                        raise SyntaxError(f'Line {self.peek().line}: array {name} needs size or initializer')
                    globals_.append((name, [0]*size, storage))
                    self.globals_sizes[name] = size
            elif self.match(';'):
                globals_.append((name, 0, storage))
                self.globals_sizes[name] = 2 if typ == 'u16' else 1
            else: raise SyntaxError(f'Line {self.peek().line}: unexpected after {name}')
        return globals_, functions

    def parse_type(self):
        # Typedef aliases resolve before the built-in types.
        t = self.peek()
        if t.type == 'IDENT' and t.value in self.typedefs:
            self.advance()
            return self.typedefs[t.value]
        if self.match('void'): return 'void'
        if self.match('u8'): return 'u8'
        if self.match('u16'): return 'u16'
        if self.match('unsigned'):
            if self.match('char'): return 'u8'
            if self.match('int'): return 'u16'
            return 'u8'  # bare 'unsigned' = unsigned char
        if self.match('char'): return 'u8'
        if self.match('int'): return 'u16'
        raise SyntaxError(f'Line {self.peek().line}: expected type')

    def parse_function(self, ret_type, name):
        params = []
        if not self.match(')'):
            # Handle (void) parameter list
            if self.peek().value == 'void':
                self.advance(); self.expect(')')
            else:
                while True:
                    ptype = self.parse_type(); pname = self.expect('IDENT').value
                    params.append(pname)
                    if not self.match(','): break
                self.expect(')')
        # Handle inline asm declarations: void f(void) = "...";
        if self.match('='):
            self.advance()  # skip string literal
            self.expect(';')
            return None  # signal to skip
        # Forward declaration: type name(params);
        if self.match(';'):
            return None  # prototype, skip
        body = self.parse_block()
        return (name, params, body, ret_type)

    def parse_block(self):
        self.expect('{'); stmts = []
        while not self.match('}'): stmts.append(self.parse_statement())
        return ('block', stmts)

    def parse_statement(self):
        t = self.peek()
        if t.value == 'if': return self.parse_if()
        if t.value == 'while': return self.parse_while()
        if t.value == 'for': return self.parse_for()
        if t.value == 'do': return self.parse_do_while()
        if t.value == 'switch': return self.parse_switch()
        if t.value == 'break': self.advance(); self.expect(';'); return ('break',)
        if t.value == 'continue': self.advance(); self.expect(';'); return ('continue',)
        if t.value == 'return':
            self.advance()
            if self.match(';'): return ('return', None)
            e = self.parse_expr(); self.expect(';'); return ('return', e)
        if t.value in ('unsigned', 'char', 'int', 'u8', 'u16', 'static'): return self.parse_local_decl()
        # Typedef alias in statement position: behave like a local declaration.
        if t.type == 'IDENT' and t.value in self.typedefs: return self.parse_local_decl()
        if t.value == '{': return self.parse_block()
        e = self.parse_expr(); self.expect(';'); return ('expr_stmt', e)

    def parse_do_while(self):
        self.expect('do')
        body = self.parse_statement()
        self.expect('while'); self.expect('('); c = self.parse_expr(); self.expect(')')
        self.expect(';')
        return ('do_while', body, c)

    def parse_switch(self):
        self.expect('switch'); self.expect('('); expr = self.parse_expr(); self.expect(')')
        self.expect('{')
        cases = []
        default = None
        while not self.match('}'):
            if self.match('case'):
                val = self.expect('NUMBER').value
                self.expect(':')
                stmts = []
                while self.peek().value not in ('case', 'default', '}'):
                    stmts.append(self.parse_statement())
                cases.append((val, ('block', stmts)))
            elif self.match('default'):
                self.expect(':')
                stmts = []
                while self.peek().value not in ('case', '}'):
                    stmts.append(self.parse_statement())
                default = ('block', stmts)
        return ('switch', expr, cases, default)

    def parse_if(self):
        self.expect('if'); self.expect('('); c = self.parse_expr(); self.expect(')')
        then = self.parse_statement()
        els = self.parse_statement() if self.match('else') else None
        return ('if', c, then, els)

    def parse_while(self):
        self.expect('while'); self.expect('('); c = self.parse_expr(); self.expect(')')
        return ('while', c, self.parse_statement())

    def parse_for(self):
        self.expect('for'); self.expect('(')
        init = self.parse_statement() if self.peek().value != ';' else None
        if init is None: self.expect(';')
        cond = self.parse_expr() if self.peek().value != ';' else ('num', 1)
        self.expect(';')
        update = self.parse_expr() if self.peek().value != ')' else None
        self.expect(')')
        return ('for', init, cond, update, self.parse_statement())

    def parse_local_decl(self):
        is_static = bool(self.match('static'))
        typ = self.parse_type(); name = self.expect('IDENT').value
        if self.match('['):
            size = self.expect('NUMBER').value; self.expect(']')
            init_vals = None
            if self.match('='):
                self.expect('{')
                init_vals = []
                while self.peek().value != '}':
                    init_vals.append(self.parse_expr())
                    if not self.match(','): break
                self.expect('}')
            self.expect(';')
            if is_static:
                return ('static_local_arr', name, size, init_vals)
            return ('local_arr', name, size, init_vals)
        init = self.parse_expr() if self.match('=') else None
        self.expect(';')
        if is_static:
            return ('static_local', name, init, typ)
        return ('local', name, init, typ)

    def parse_expr(self): return self.parse_assign()
    def parse_assign(self):
        left = self.parse_ternary()
        if self.peek().value in ('=','+=','-=','*=','/=','%=','&=','|=','^=','<<=','>>='):
            op = self.advance().value; right = self.parse_assign()
            return ('assign', op, left, right)
        return left
    def parse_ternary(self):
        cond = self.parse_log_or()
        if self.match('?'):
            then = self.parse_expr()
            self.expect(':')
            els = self.parse_ternary()
            return ('ternary', cond, then, els)
        return cond
    def parse_log_or(self):
        left = self.parse_log_and()
        while self.peek().value == '||':
            self.advance(); left = ('log_or', left, self.parse_log_and())
        return left
    def parse_log_and(self):
        left = self.parse_bit_or()
        while self.peek().value == '&&':
            self.advance(); left = ('log_and', left, self.parse_bit_or())
        return left
    def parse_bit_or(self):
        left = self.parse_xor()
        while self.peek().value == '|':
            self.advance(); left = ('binop', '|', left, self.parse_xor())
        return left
    def parse_xor(self):
        left = self.parse_bit_and()
        while self.peek().value == '^':
            self.advance(); left = ('binop', '^', left, self.parse_bit_and())
        return left
    def parse_bit_and(self):
        left = self.parse_cmp()
        while self.peek().value == '&':
            self.advance(); left = ('binop', '&', left, self.parse_cmp())
        return left
    def parse_cmp(self):
        left = self.parse_shift()
        if self.peek().value in ('==','!=','<','>','<=','>='):
            op = self.advance().value; return ('binop', op, left, self.parse_shift())
        return left
    def parse_shift(self):
        left = self.parse_add()
        while self.peek().value in ('<<', '>>'):
            op = self.advance().value; left = ('binop', op, left, self.parse_add())
        return left
    def parse_add(self):
        left = self.parse_mul()
        while self.peek().value in ('+','-'):
            op = self.advance().value; left = ('binop', op, left, self.parse_mul())
        return left
    def parse_mul(self):
        left = self.parse_unary()
        while self.peek().value in ('*','/',  '%'):
            op = self.advance().value; left = ('binop', op, left, self.parse_unary())
        return left
    def parse_unary(self):
        if self.peek().value == '~': self.advance(); return ('unop', '~', self.parse_unary())
        if self.peek().value == '!': self.advance(); return ('unop', '!', self.parse_unary())
        if self.peek().value == '-': self.advance(); return ('unop', '-', self.parse_unary())
        if self.peek().value == '++':
            self.advance(); e = self.parse_unary(); return ('preinc', e)
        if self.peek().value == '--':
            self.advance(); e = self.parse_unary(); return ('predec', e)
        return self.parse_postfix()
    def parse_postfix(self):
        e = self.parse_primary()
        while True:
            if self.peek().value == '[':
                self.advance(); idx = self.parse_expr(); self.expect(']')
                e = ('index', e, idx)
            elif self.peek().value == '++':
                self.advance(); e = ('postinc', e)
            elif self.peek().value == '--':
                self.advance(); e = ('postdec', e)
            else: break
        return e
    def parse_primary(self):
        t = self.peek()
        # `sizeof(type)` or `sizeof(ident)` → compile-time integer constant.
        # For an array global, returns its length. For u8/char/unsigned: 1.
        # For u16/int: 2. Unknown scalar identifiers default to 1.
        if t.value == 'sizeof':
            self.advance()
            self.expect('(')
            n = self.peek()
            if n.value in ('u8', 'char', 'unsigned'):
                self.advance()
                if n.value == 'unsigned':
                    self.match('char') or self.match('int')
                size = 1
            elif n.value in ('u16', 'int'):
                self.advance()
                size = 2
            elif n.type == 'IDENT':
                name = self.advance().value
                if name in self.typedefs:
                    size = 2 if self.typedefs[name] == 'u16' else 1
                else:
                    size = self.globals_sizes.get(name, 1)
            else:
                raise SyntaxError(f'Line {t.line}: sizeof expects a type or identifier')
            self.expect(')')
            return ('num', size)
        if t.type == 'NUMBER': self.advance(); return ('num', t.value)
        if t.type == 'STRING':
            self.advance()
            # Strip quotes and decode escapes, including `\xNN` for non-ASCII
            # LCD glyphs (e.g. HD44780 0xDF = °).
            raw = t.value[1:-1]
            out_s = []
            i = 0
            while i < len(raw):
                c = raw[i]
                if c != '\\':
                    out_s.append(c); i += 1; continue
                if i + 1 >= len(raw):
                    out_s.append('\\'); i += 1; continue
                esc = raw[i + 1]
                if esc == 'n': out_s.append('\n'); i += 2
                elif esc == 'r': out_s.append('\r'); i += 2
                elif esc == 't': out_s.append('\t'); i += 2
                elif esc == '0': out_s.append('\0'); i += 2
                elif esc == '\\': out_s.append('\\'); i += 2
                elif esc == '"': out_s.append('"'); i += 2
                elif esc == "'": out_s.append("'"); i += 2
                elif esc == 'x' and i + 3 < len(raw):
                    try:
                        out_s.append(chr(int(raw[i+2:i+4], 16)))
                        i += 4
                    except ValueError:
                        out_s.append(c); i += 1
                else:
                    out_s.append(c); i += 1
            result = ''.join(out_s)
            # Adjacent string literals concatenate: `"abc" "def"` → `"abcdef"`.
            # Just recurse into parse_primary while another STRING follows.
            while self.peek().type == 'STRING':
                nxt = self.parse_primary()   # will eat the next STRING & decode
                result += nxt[1]
            return ('string', result)
        if t.type == 'IDENT':
            name = self.advance().value
            if self.match('('):
                args = []
                if not self.match(')'):
                    while True:
                        args.append(self.parse_expr())
                        if not self.match(','): break
                    self.expect(')')
                return ('call', name, args)
            return ('var', name)
        if self.match('('): e = self.parse_expr(); self.expect(')'); return e
        raise SyntaxError(f'Line {t.line}: unexpected {t.value!r}')

# ── Code Generator ───────────────────────────────────────────────────

class MK1CodeGen:
    """Generates MK1 assembly with full knowledge of hardware constraints.

    Optimizations:
    - Dead variable elimination: vars that are written but never read are skipped
    - Comparison swap: > and <= become single-jump via operand swap
    - Local+assign merge: "type x; x = expr;" becomes "type x = expr;"
    - push_imm for constant initializers
    - Peephole: register tracking, dead code, redundant load elimination
    """

    def __init__(self, optimize=False):
        self.code = []
        self.globals = {}
        self.data_alloc = 0
        self.label_id = 0
        self.optimize = optimize
        self.b_expr = None  # AST expr currently in B, for cache reuse
        self._prefer_delay_cal_init = False

        # ── Port-pin shadow ──────────────────────────────────────────
        # Tracks bits each builtin claims on VIA port A/B so that every
        # ddrb_imm/ddra_imm/orb_imm/ora_imm emission preserves other
        # devices' state. Per-port dict maps device-name → bitmask of
        # bits that must be held as OUTPUT for the lifetime of the
        # program (when the device is active). Builtins that toggle
        # direction per-transaction (e.g. I2C SDA) don't claim their
        # toggling bits — they manage those explicitly.
        #
        # ASSUMPTION: claimed bits mean "this bit must stay as OUTPUT
        # (DDR=1) while the device is active." The shadow mask is OR'd
        # into every emission, which preserves output direction but
        # cannot force a bit to input (DDR=0). This is correct for all
        # current and planned devices (buzzer push-pull, keypad rows as
        # outputs). It would NOT work for a hypothetical second
        # DDR-only open-drain device on the same port, whose idle state
        # is DDR=0. We don't have such a device — I2C is the only
        # open-drain user, and it manages PB0/PB1 directly without
        # shadow claims. Revisit this if that changes.
        self._port_claims = {'DDRB': {}, 'DDRA': {}, 'ORB': {}, 'ORA': {}}

    def _claim_port_bits(self, reg, mask, device):
        """Register always-output (or always-driven) bits for a device.
        Future ddrb_imm / orb_imm / etc. emissions will OR in this mask
        so unrelated devices' pins are preserved.

        reg:    'DDRB', 'DDRA', 'ORB', or 'ORA'
        mask:   bitmask of bits this device owns
        device: string name, for diagnostics
        """
        existing = self._port_claims[reg]
        # Detect collisions with previously-claimed bits
        for other_dev, other_mask in existing.items():
            if other_dev == device:
                continue
            overlap = mask & other_mask
            if overlap:
                raise Exception(
                    f"Port-pin conflict on {reg}: device '{device}' claims bits "
                    f"0x{mask:02X}, which overlap 0x{overlap:02X} already claimed "
                    f"by '{other_dev}' (0x{other_mask:02X})"
                )
        existing[device] = mask

    def _port_shadow_mask(self, reg):
        """OR of all bits claimed on `reg` across all devices. 0 if none."""
        total = 0
        for m in self._port_claims[reg].values():
            total |= m
        return total

    def label(self, prefix='L'):
        self.label_id += 1
        return f'.{prefix}{self.label_id}'

    def emit(self, line):
        self.code.append(line)
        # Invalidate B cache on B-modifying instructions
        s = line.strip()
        if (',$b' in s and s.startswith('mov')) or s.startswith('pop_b') or s == 'swap' or s.startswith('jal'):
            self.b_expr = None

    # ── Port-register emitters with shadow augmentation ─────────────
    # Every ddrb_imm/ddra_imm/orb_imm/ora_imm call site in codegen goes
    # through these helpers instead of directly emitting strings. The
    # value is OR'd with the current always-output / always-driven
    # shadow mask so bits claimed by other devices are preserved.
    #
    # Backward compatible: when no device has claimed bits (the current
    # state of things pre-keypad), the mask is 0 and emission is
    # byte-identical to the old `self.emit('\tddrb_imm 0xNN')`.

    def emit_ddrb(self, value, comment=None):
        shadow = self._port_shadow_mask('DDRB')
        augmented = (value | shadow) & 0xFF
        line = f'\tddrb_imm 0x{augmented:02X}'
        if comment:
            line += f'\t; {comment}'
        elif shadow and augmented != value:
            line += f'\t; (0x{value:02X} widened by shadow 0x{shadow:02X})'
        self.emit(line)

    def emit_ddra(self, value, comment=None):
        shadow = self._port_shadow_mask('DDRA')
        augmented = (value | shadow) & 0xFF
        line = f'\tddra_imm 0x{augmented:02X}'
        if comment:
            line += f'\t; {comment}'
        elif shadow and augmented != value:
            line += f'\t; (0x{value:02X} widened by shadow 0x{shadow:02X})'
        self.emit(line)

    def emit_orb(self, value, comment=None):
        shadow = self._port_shadow_mask('ORB')
        augmented = (value | shadow) & 0xFF
        line = f'\torb_imm 0x{augmented:02X}'
        if comment:
            line += f'\t; {comment}'
        elif shadow and augmented != value:
            line += f'\t; (0x{value:02X} widened by shadow 0x{shadow:02X})'
        self.emit(line)

    def emit_ora(self, value, comment=None):
        shadow = self._port_shadow_mask('ORA')
        augmented = (value | shadow) & 0xFF
        line = f'\tora_imm 0x{augmented:02X}'
        if comment:
            line += f'\t; {comment}'
        elif shadow and augmented != value:
            line += f'\t; (0x{value:02X} widened by shadow 0x{shadow:02X})'
        self.emit(line)

    # ── Kernel-state slot access (page-aware) ────────────────────────
    # All reads/writes to KERNEL_STATE slots go through these helpers
    # so a future page migration (Phase 5 may move slots to free up
    # page-3 space) requires only a registry edit, not a code-site
    # rewrite. The opcode pair for each page is the only difference:
    #   page 2 (stack page): deref2 / ideref2
    #   page 3 (extended):   derefp3 / iderefp3
    # No other supported page; helper raises if the registry entry
    # uses an unsupported page.

    def _kstate_load_lines(self, slot_name):
        """Return asm lines that read slot into A. Convention: A is
        clobbered both as the address (input) and the value (output)."""
        slot = KERNEL_STATE[slot_name]
        if slot.page == 3:
            op = 'derefp3'
        elif slot.page == 2:
            op = 'deref2'
        else:
            raise ValueError(f'kstate {slot_name}: unsupported page {slot.page}')
        return [f'\tldi $a,{slot.offset}', f'\t{op}']

    def _kstate_store_lines(self, slot_name):
        """Return asm lines that write A to slot. B is clobbered (set
        to slot offset). Caller must place the value in A before."""
        slot = KERNEL_STATE[slot_name]
        if slot.page == 3:
            op = 'iderefp3'
        elif slot.page == 2:
            op = 'ideref2'
        else:
            raise ValueError(f'kstate {slot_name}: unsupported page {slot.page}')
        return [f'\tldi $b,{slot.offset}', f'\t{op}']

    def emit_kstate_load(self, slot_name):
        for line in self._kstate_load_lines(slot_name):
            self.emit(line)

    def emit_kstate_store(self, slot_name):
        for line in self._kstate_store_lines(slot_name):
            self.emit(line)

    # ── Dead variable analysis ───────────────────────────────────────

    def _collect_reads(self, node, reads):
        """Walk AST node, collect all variable names that are READ."""
        if not isinstance(node, tuple) or len(node) == 0:
            return
        kind = node[0]
        if kind == 'var':
            reads.add(node[1])
        elif kind == 'assign':
            # LHS is written, not read (unless compound assign)
            op, left, right = node[1], node[2], node[3]
            if op != '=':
                self._collect_reads(left, reads)  # compound: also reads
            if left[0] == 'index':
                self._collect_reads(left, reads)  # index target is read
            self._collect_reads(right, reads)
        elif kind == 'postinc' or kind == 'postdec':
            self._collect_reads(node[1], reads)  # reads old value
        elif kind in ('block',):
            for s in node[1]:
                self._collect_reads(s, reads)
        elif kind == 'local':
            if node[2]:
                self._collect_reads(node[2], reads)
        else:
            for child in node[1:]:
                if isinstance(child, tuple):
                    self._collect_reads(child, reads)
                elif isinstance(child, list):
                    for item in child:
                        if isinstance(item, tuple):
                            self._collect_reads(item, reads)

    def _find_dead_locals(self, body, params):
        """Find local variables that are assigned but never read."""
        reads = set()
        self._collect_reads(body, reads)
        return reads  # return the SET of read vars (dead = declared - reads)

    def _count_reads(self, node, name):
        """Count occurrences where `name` is READ in the AST. Mirrors
        the structural rules of `_collect_reads` (LHS is written not
        read, compound assign reads the LHS, etc). Used to decide
        whether a regparam in $a needs to be saved at function entry:
        a single read can stay in $a until first use, but >1 reads
        require a save because every prior expression may have
        clobbered $a (any `inc`, `addi`, deref, builtin etc)."""
        if not isinstance(node, tuple) or len(node) == 0:
            return 0
        kind = node[0]
        if kind == 'var':
            return 1 if node[1] == name else 0
        if kind == 'assign':
            op, left, right = node[1], node[2], node[3]
            n = self._count_reads(right, name)
            if op != '=':
                n += self._count_reads(left, name)
            if left[0] == 'index':
                n += self._count_reads(left, name)
            return n
        if kind == 'postinc' or kind == 'postdec':
            return self._count_reads(node[1], name)
        if kind == 'block':
            return sum(self._count_reads(s, name) for s in node[1])
        if kind == 'local':
            return self._count_reads(node[2], name) if node[2] else 0
        n = 0
        for child in node[1:]:
            if isinstance(child, tuple):
                n += self._count_reads(child, name)
            elif isinstance(child, list):
                for item in child:
                    if isinstance(item, tuple):
                        n += self._count_reads(item, name)
        return n

    BUILTINS = {'out', 'halt', 'nop', 'exw', 'exr', 'exrw', 'clr_irq0', 'clr_irq1',
                'irq0', 'irq1', 'exr_port_a', 'exr_port_b', 'exr_port_c',
                'via_read_porta', 'via_read_portb',
                'i2c_init', 'i2c_bus_reset', 'i2c_start', 'i2c_stop',
                'i2c_ack', 'i2c_nack', 'i2c_wait_ack', 'i2c_stream',
                'i2c_stream_result',
                'ora_imm', 'orb_imm', 'ddra_imm',
                'write_code', 'call_code', 'eeprom_read_to_code',
                'peek3', 'poke3'}
    # NOTE: i2c_send_byte, i2c_read_byte, eeprom_write_byte, eeprom_read_byte,
    # rtc_read_seconds, rtc_read_temp, lcd_cmd, lcd_char, lcd_init are NOT builtins —
    # they use jal to subroutines that clobber B, C, D. The register
    # allocator must treat them as user function calls.

    # ── $d-clobber analysis ─────────────────────────────────────────
    # Names of compiler-emitted builtins whose generated helper clobbers
    # $d without restoring it. A function that (transitively) calls any
    # of these cannot use $d as a cross-call save slot for a param or
    # local. Verified by inspection of each helper's emission in the
    # compiler source (search for `__lcd_print`, `__i2c_rb`, etc.).
    D_CLOBBER_BUILTINS = frozenset({
        # User-facing builtin names (what appears as `('call', name, ...)`):
        'printf',          # lowers to __lcd_print (+ __lcd_temp_u8 etc.)
        'lcd_print',       # __lcd_print uses $d as pointer
        'lcd_str',         # same
        'lcd_init',        # data-driven, iterates via $d
        'rtc_read_temp',   # uses __i2c_rb (bit counter in $d)
        'rtc_read_seconds',
        'i2c_read_byte',   # __i2c_rb directly
        'eeprom_read_byte',
        'eeprom_read',
        'eeprom_write_byte',
        'eeprom_write',
        'i2c_stream',      # __i2c_stream pointer in $d
        'i2c_stream_result',
        'delay',           # __delay_Nms product high in $d
        'tone',
        'silence',
    })

    def _call_targets(self, node, acc):
        """Collect every `call` target name in the AST subtree."""
        if not isinstance(node, tuple) or len(node) == 0:
            return
        if node[0] == 'call':
            acc.add(node[1])
        for child in node[1:]:
            if isinstance(child, tuple):
                self._call_targets(child, acc)
            elif isinstance(child, list):
                for item in child:
                    if isinstance(item, tuple):
                        self._call_targets(item, acc)

    def _body_calls_any(self, body, target_set):
        """True if the AST body contains a `call` to any name in target_set."""
        if not target_set:
            return False
        seen = set()
        self._call_targets(body, seen)
        return not seen.isdisjoint(target_set)

    def _compute_d_clobber_funcs(self):
        """Transitive closure: a user function clobbers $d if it calls
        a $d-clobbering builtin, or a user function that (transitively)
        clobbers $d. Stored on self as `_d_clobber_funcs`."""
        clobbers = set()
        # Seed with user functions that directly call a clobbering builtin.
        for fname, (_p, body) in self.func_bodies.items():
            if self._body_calls_any(body, self.D_CLOBBER_BUILTINS):
                clobbers.add(fname)
        # Fixed point: add user functions that call another clobberer.
        changed = True
        while changed:
            changed = False
            for fname, (_p, body) in self.func_bodies.items():
                if fname in clobbers:
                    continue
                if self._body_calls_any(body, clobbers):
                    clobbers.add(fname)
                    changed = True
        # Build the combined set used at reg-alloc time.
        self._d_clobber_funcs = set(self.D_CLOBBER_BUILTINS) | clobbers

    def _has_calls(self, node):
        """Check if AST node contains any non-builtin function calls (jal).
        Also detects eeprom array subscripts (generate jal __eeprom_rd)."""
        if not isinstance(node, tuple) or len(node) == 0:
            return False
        if node[0] == 'call' and node[1] not in self.BUILTINS:
            return True
        if (node[0] == 'index' and len(node) > 1
                and isinstance(node[1], tuple) and node[1][0] == 'var'
                and node[1][1] in self.eeprom_globals):
            return True
        for child in node[1:]:
            if isinstance(child, tuple) and self._has_calls(child):
                return True
            elif isinstance(child, list):
                for item in child:
                    if isinstance(item, tuple) and self._has_calls(item):
                        return True
        return False

    def _has_stsp(self, node):
        """Check if AST node would generate any stsp (stack store) instructions.
        stsp clobbers D, so D register allocation is unsafe if stsp occurs."""
        if not isinstance(node, tuple) or len(node) == 0:
            return False
        if node[0] == 'assign' and node[2][0] == 'var':
            # Check if target is a stack local (would need stsp)
            name = node[2][1]
            if name in self.locals and self.locals[name][0] == 'local':
                return True
        if node[0] in ('postinc', 'postdec'):
            if node[1][0] == 'var' and node[1][1] in self.locals:
                if self.locals[node[1][1]][0] == 'local':
                    return True
        for child in node[1:]:
            if isinstance(child, tuple) and self._has_stsp(child):
                return True
            elif isinstance(child, list):
                for item in child:
                    if isinstance(item, tuple) and self._has_stsp(item):
                        return True
        return False

    def _has_xor(self, node):
        """Check if AST node contains any XOR (^) operations.
        xor clobbers D (used as scratch in microcode), so D is unsafe."""
        if not isinstance(node, tuple) or len(node) == 0:
            return False
        if node[0] == 'xor':
            return True
        for child in node[1:]:
            if isinstance(child, tuple) and self._has_xor(child):
                return True
            elif isinstance(child, list):
                for item in child:
                    if isinstance(item, tuple) and self._has_xor(item):
                        return True
        return False

    # ── Phase 7: Auto function splitting ──────────────────────────────

    I2C_CALLS = {'i2c_start', 'i2c_stop', 'i2c_send_byte', 'i2c_read_byte',
                 'i2c_nack', 'i2c_ack', 'i2c_bus_reset', 'i2c_stream',
                 'i2c_repeated_start',
                 'rtc_read_temp', 'rtc_read_seconds', 'eeprom_read_byte',
                 'eeprom_read', 'eeprom_write_byte', 'eeprom_write'}
    LCD_CALLS = {'lcd_cmd', 'lcd_char', 'lcd_print', 'delay', 'tone', 'silence'}

    def _stmt_domain(self, stmt):
        """Classify a statement as 'i2c', 'lcd', or None (neutral)."""
        if not isinstance(stmt, tuple) or len(stmt) == 0:
            return None
        kind = stmt[0]
        if kind == 'expr_stmt':
            return self._expr_domain(stmt[1])
        if kind == 'return' and stmt[1]:
            return self._expr_domain(stmt[1])
        if kind in ('if', 'while', 'do_while', 'for'):
            # Check condition + body
            domains = set()
            for sub in stmt[1:]:
                if isinstance(sub, tuple) and sub[0] == 'block':
                    for s in sub[1]:
                        d = self._stmt_domain(s)
                        if d: domains.add(d)
                elif isinstance(sub, tuple):
                    d = self._expr_domain(sub) if sub[0] != 'block' else None
                    if d: domains.add(d)
            if 'lcd' in domains: return 'lcd'
            if 'i2c' in domains: return 'i2c'
        return None

    def _expr_domain(self, expr):
        """Classify an expression by which helper domain it uses."""
        if not isinstance(expr, tuple) or len(expr) == 0:
            return None
        if expr[0] == 'call':
            name = expr[1]
            if name in self.I2C_CALLS: return 'i2c'
            if name in self.LCD_CALLS: return 'lcd'
            # User function calls: check if the function uses I2C
            if name in self.func_bodies:
                body_str = str(self.func_bodies[name][1])
                if any(c in body_str for c in self.I2C_CALLS): return 'i2c'
                if any(c in body_str for c in self.LCD_CALLS): return 'lcd'
        if expr[0] == 'assign':
            return self._expr_domain(expr[3])
        for sub in expr[1:]:
            if isinstance(sub, tuple):
                d = self._expr_domain(sub)
                if d: return d
        return None

    def _find_vars_in_expr(self, expr, result):
        """Collect all variable names referenced in an expression."""
        if not isinstance(expr, tuple): return
        if expr[0] == 'var':
            result.add(expr[1])
        for sub in expr[1:]:
            if isinstance(sub, tuple):
                self._find_vars_in_expr(sub, result)
            elif isinstance(sub, list):
                for item in sub:
                    if isinstance(item, tuple):
                        self._find_vars_in_expr(item, result)

    def _find_vars_in_stmt(self, stmt, assigned, used):
        """Collect assigned and used variable names in a statement."""
        if not isinstance(stmt, tuple): return
        kind = stmt[0]
        if kind == 'local':
            assigned.add(stmt[1])
            if stmt[2]: self._find_vars_in_expr(stmt[2], used)
        elif kind == 'expr_stmt':
            e = stmt[1]
            if e[0] == 'assign' and e[2][0] == 'var':
                assigned.add(e[2][1])
                self._find_vars_in_expr(e[3], used)
            else:
                self._find_vars_in_expr(e, used)
        elif kind == 'return':
            if stmt[1]: self._find_vars_in_expr(stmt[1], used)
        elif kind in ('if', 'while', 'do_while', 'for'):
            for sub in stmt[1:]:
                if isinstance(sub, tuple) and sub[0] == 'block':
                    for s in sub[1]:
                        self._find_vars_in_stmt(s, assigned, used)
                elif isinstance(sub, tuple):
                    self._find_vars_in_expr(sub, used)

    def _auto_split_functions(self, functions):
        """Split functions spanning I2C + LCD domains into separate phases.
        Also splits main when it has the same transition — the tail becomes
        a new overlay-eligible function that main jal's into."""
        new_functions = []
        main_rewrites = {}  # original_name → [(phase1_name, phase2_name)]

        def _is_redundant_lcd_cmd_after_lcd_init(prev_stmt, stmt):
            """True if stmt is lcd_cmd(0x01) or lcd_cmd(0x02) and prev is lcd_init()
            — the init sequence already clears the display so the cmd is a no-op."""
            if prev_stmt is None or stmt is None:
                return False
            if (prev_stmt[0] == 'expr_stmt' and prev_stmt[1][0] == 'call'
                    and prev_stmt[1][1] == 'lcd_init'):
                if (stmt[0] == 'expr_stmt' and stmt[1][0] == 'call'
                        and stmt[1][1] == 'lcd_cmd' and stmt[1][2]):
                    arg = stmt[1][2][0]
                    if arg[0] == 'num' and arg[1] in (0x01, 0x02):
                        return True
            return False

        for name, params, body, ret_type in functions:
            if body[0] != 'block':
                new_functions.append((name, params, body, ret_type))
                continue

            # AST-level peephole: drop redundant lcd_cmd(0x01)/(0x02) right after
            # lcd_init(). Must match the gen_expr-level peephole so auto-split's
            # domain analysis sees the same sequence the codegen will emit.
            stmts_orig = body[1]
            filtered = []
            prev = None
            for s in stmts_orig:
                if _is_redundant_lcd_cmd_after_lcd_init(prev, s):
                    continue  # drop the redundant clear
                filtered.append(s)
                prev = s
            if filtered != stmts_orig:
                body = ('block', filtered)

            stmts = body[1]
            # Classify each statement
            domains = [(i, self._stmt_domain(s)) for i, s in enumerate(stmts)]

            # Find I2C→LCD transition
            last_i2c = -1
            first_lcd = len(stmts)
            for i, d in domains:
                if d == 'i2c': last_i2c = i
            for i, d in domains:
                if d == 'lcd':
                    first_lcd = i
                    break

            if last_i2c < 0 or first_lcd >= len(stmts) or last_i2c >= first_lcd:
                # No clear I2C→LCD transition
                new_functions.append((name, params, body, ret_type))
                continue

            # Split point: after last_i2c, before first_lcd
            split = last_i2c + 1

            # Find live variables at the split point
            before_assigned = set()
            before_used = set()
            for s in stmts[:split]:
                self._find_vars_in_stmt(s, before_assigned, before_used)

            after_used = set()
            after_assigned = set()
            for s in stmts[split:]:
                self._find_vars_in_stmt(s, after_assigned, after_used)

            live_vars = list(before_assigned & after_used - set(params))

            if len(live_vars) == 0:
                # No live vars → phase1 is void, phase2 takes no extra params
                new_functions.append((name, params, body, ret_type))
                continue

            # Only split if the function has enough statements to benefit.
            # Small functions (< 8 statements) are better left unsplit — the
            # overlay call overhead from splitting outweighs the helper savings.
            if len(stmts) < 8:
                new_functions.append((name, params, body, ret_type))
                continue

            import sys
            print(f"  Phase 7: splitting {name} at stmt {split} "
                  f"(live: {live_vars})", file=sys.stderr)

            # Main splits as dispatcher: main becomes a small function that
            # runs init, calls an I2C-heavy phase (as its own overlay), then
            # resumes with the original display-heavy calls.
            # Keeping the user-function calls in main directly (not inside a
            # phase2) avoids the SCC-merge problem where phase2 would end up
            # in the same overlay slot as every user function it calls,
            # ballooning the slot size.
            if name == 'main':
                INIT_CALLS = {'i2c_init', 'lcd_init', 'delay_calibrate',
                              'ddra_imm', 'ddrb_imm'}
                def _is_init_stmt(s):
                    if s[0] == 'expr_stmt' and s[1][0] == 'call':
                        return s[1][1] in INIT_CALLS
                    return False
                prelude_end = 0
                for si, s in enumerate(stmts):
                    if _is_init_stmt(s):
                        prelude_end = si + 1
                    else:
                        break
                prelude_stmts = list(stmts[:prelude_end])
                body_stmts = list(stmts[prelude_end:])
                split_in_body = split - prelude_end
                if split_in_body < 1 or split_in_body >= len(body_stmts):
                    new_functions.append((name, params, body, ret_type))
                    continue
                # Name without leading underscore so the emitted label is
                # `_main_p1` (one '_'), keeping it classified as a user
                # function rather than a library helper.
                p1_name = 'main_p1'
                p1_inner = list(body_stmts[:split_in_body])
                # Allocate transfer globals for live vars so the post-split
                # code in main can still read them after p1 returns.
                xfer_globals = {}  # lv → xfer_name
                for lv in live_vars:
                    xfer_name = f'__xfer_main_{lv}'
                    if xfer_name not in self.globals and xfer_name not in self.page3_globals:
                        if self.data_alloc < 256:
                            self.globals[xfer_name] = self.data_alloc
                            self.data_alloc += 1
                        else:
                            self.page3_globals[xfer_name] = self.page3_alloc
                            self.page3_alloc += 1
                    xfer_globals[lv] = xfer_name

                # Optimization: for each live_var that is only WRITTEN (never
                # read) inside p1, skip allocating a local and redirect the
                # assignment directly to the xfer global. Saves ~40B in
                # dashboard-class main_p1 by eliminating the local-stack
                # shuffle (push_imm 0 / stsp / mov $d,$a / final copy-out).
                def _used_in_stmts(stmts, name):
                    """Return True if `name` is read as a variable in any of stmts.
                    AST shape: expressions are tuples with [0] = kind.
                    assign tuple: ('assign', op, lhs, rhs) — lhs is a write, rhs is a read."""
                    def rd(e):
                        if e is None: return False
                        if not isinstance(e, tuple): return False
                        t = e[0]
                        if t == 'var':
                            return e[1] == name
                        if t == 'num' or t == 'str':
                            return False
                        if t == 'assign':
                            # rhs (e[3]) is a read. lhs (e[2]) is a write, but
                            # if lhs is `var[expr]` the expr is a read.
                            lhs = e[2]
                            lhs_read = False
                            if isinstance(lhs, tuple) and lhs[0] == 'index':
                                # index(target, idx) — both can contain reads
                                lhs_read = rd(lhs[1]) or rd(lhs[2])
                            return lhs_read or rd(e[3])
                        if t == 'call':
                            return any(rd(a) for a in e[2] or [])
                        if t in ('bin', 'binop'):
                            return rd(e[2]) or rd(e[3])
                        if t in ('un', 'unop'):
                            return rd(e[2])
                        if t == 'index':
                            return rd(e[1]) or rd(e[2])
                        if t == 'addr_of':
                            return rd(e[1])
                        return False
                    def st(s):
                        if not isinstance(s, tuple): return False
                        t = s[0]
                        if t == 'local':
                            return rd(s[2])
                        if t == 'expr_stmt':
                            return rd(s[1])
                        if t == 'return':
                            return rd(s[1]) if len(s) > 1 and s[1] else False
                        if t == 'if':
                            then_block = s[2] if len(s) > 2 else []
                            else_block = s[3] if len(s) > 3 and s[3] else []
                            return rd(s[1]) or any(st(x) for x in then_block) \
                                   or any(st(x) for x in else_block)
                        if t == 'while':
                            return rd(s[1]) or any(st(x) for x in s[2])
                        if t == 'block':
                            return any(st(x) for x in s[1])
                        return False
                    return any(st(s) for s in stmts)

                write_only_lvs = set()
                for lv in live_vars:
                    # Count writes (local decls with init, or assign to var lv)
                    # and check no reads of lv within p1_inner.
                    if not _used_in_stmts(p1_inner, lv):
                        write_only_lvs.add(lv)

                if write_only_lvs:
                    # Rewrite p1_inner: drop local decls for write_only_lvs,
                    # rewrite `lv = expr` → `__xfer_main_lv[0] = expr`.
                    new_p1 = []
                    for s in p1_inner:
                        if s[0] == 'local' and s[1] in write_only_lvs:
                            # Drop the declaration. If there's an initializer,
                            # replace with a direct write to the xfer global.
                            if s[2] is not None:
                                new_p1.append(('expr_stmt',
                                    ('assign', '=',
                                     ('index', ('var', xfer_globals[s[1]]), ('num', 0)),
                                     s[2])))
                            continue
                        if (s[0] == 'expr_stmt' and s[1][0] == 'assign'
                                and s[1][1] == '=' and s[1][2][0] == 'var'
                                and s[1][2][1] in write_only_lvs):
                            lvn = s[1][2][1]
                            new_p1.append(('expr_stmt',
                                ('assign', '=',
                                 ('index', ('var', xfer_globals[lvn]), ('num', 0)),
                                 s[1][3])))
                            continue
                        new_p1.append(s)
                    p1_inner = new_p1
                    import sys as _p7dbg
                    print(f"  Phase 7 direct-xfer: {sorted(write_only_lvs)} "
                          f"write directly to globals (no local alloc)",
                          file=_p7dbg.stderr)

                # For vars that DO get read inside p1, fall back to the
                # original "local + copy-out at end" pattern.
                for lv in live_vars:
                    if lv in write_only_lvs:
                        continue
                    p1_inner.append(('expr_stmt',
                        ('assign', '=',
                         ('index', ('var', xfer_globals[lv]), ('num', 0)),
                         ('var', lv))))
                # Main body after phase 1: redeclare the live vars from globals,
                # then continue with the original post-split statements.
                reload_stmts = []
                for lv in live_vars:
                    reload_stmts.append(('local', lv,
                        ('index', ('var', xfer_globals[lv]), ('num', 0)), 'u8'))
                post_split_stmts = list(body_stmts[split_in_body:])

                # If the display tail is made only of standard builtins,
                # keep it out of resident _main as a second phase. This is the
                # normal mixed-device shape: init/read in phase1, render/halt
                # in phase2. Leaving the render tail resident shrinks the
                # overlay region and can force the RTC/read phase to wrap.
                STD_MAIN_TAIL_CALLS = {
                    'lcd_cmd', 'lcd_char', 'lcd_print', 'lcd_temp_u8',
                    'printf', 'out', 'halt', 'delay', 'silence', 'tone',
                    'i2c_bus_reset',
                }
                def _tail_builtin_only(stmts_tail):
                    ok = True
                    def expr_ok(e):
                        nonlocal ok
                        if not isinstance(e, tuple) or not ok:
                            return
                        if e[0] == 'call':
                            if e[1] not in STD_MAIN_TAIL_CALLS:
                                ok = False
                                return
                            for a in e[2] or []:
                                expr_ok(a)
                            return
                        for child in e[1:]:
                            if isinstance(child, tuple):
                                expr_ok(child)
                            elif isinstance(child, list):
                                for item in child:
                                    expr_ok(item)
                    def stmt_ok(s):
                        nonlocal ok
                        if not isinstance(s, tuple) or not ok:
                            return
                        kind = s[0]
                        if kind == 'expr_stmt':
                            expr_ok(s[1])
                        elif kind == 'local':
                            expr_ok(s[2])
                        elif kind == 'return':
                            expr_ok(s[1] if len(s) > 1 else None)
                        elif kind in ('if', 'while', 'do_while', 'for'):
                            for child in s[1:]:
                                if isinstance(child, tuple) and child[0] == 'block':
                                    for sub in child[1]:
                                        stmt_ok(sub)
                                elif isinstance(child, tuple):
                                    expr_ok(child)
                        elif kind == 'block':
                            for sub in s[1]:
                                stmt_ok(sub)
                    for st in stmts_tail:
                        stmt_ok(st)
                    return ok

                if post_split_stmts and _tail_builtin_only(post_split_stmts):
                    p2_name = 'main_p2'
                    p2_body = ('block', reload_stmts + post_split_stmts)
                    new_main_stmts = (list(prelude_stmts)
                                      + [('expr_stmt', ('call', p1_name, [])),
                                         ('expr_stmt', ('call', p2_name, []))])
                    new_main_body = ('block', new_main_stmts)
                    p1_body = ('block', p1_inner)
                    new_functions.append(('main', list(params), new_main_body, ret_type))
                    new_functions.append((p1_name, [], p1_body, 'void'))
                    new_functions.append((p2_name, [], p2_body, ret_type))
                    self.func_params[p1_name] = 0
                    self.func_bodies[p1_name] = ([], p1_body)
                    self.func_params[p2_name] = 0
                    self.func_bodies[p2_name] = ([], p2_body)
                    continue

                new_main_stmts = (list(prelude_stmts)
                                  + [('expr_stmt', ('call', p1_name, []))]
                                  + reload_stmts
                                  + post_split_stmts)
                new_main_body = ('block', new_main_stmts)
                p1_body = ('block', p1_inner)
                new_functions.append(('main', list(params), new_main_body, ret_type))
                new_functions.append((p1_name, [], p1_body, 'void'))
                self.func_params[p1_name] = 0
                self.func_bodies[p1_name] = ([], p1_body)
                continue

            if len(live_vars) <= 2:
                p1_name = f'{name}_p1'
                p2_name = f'{name}_p2'
                p1_stmts = list(stmts[:split])

                if len(live_vars) == 1:
                    # Phase 1 returns the live var
                    p1_stmts.append(('return', ('var', live_vars[0])))
                    p1_body = ('block', p1_stmts)
                    new_functions.append((p1_name, list(params), p1_body, 'u8'))
                    # Phase 2 takes it as parameter
                    p2_stmts = list(stmts[split:])
                    p2_body = ('block', p2_stmts)
                    new_functions.append((p2_name, [live_vars[0]], p2_body, ret_type))

                elif len(live_vars) == 2:
                    # Phase 1: return second var, output first via out_imm
                    # Use a data page global as the transfer slot
                    xfer_name = f'__xfer_{name}'
                    self.globals[xfer_name] = self.data_alloc
                    self.data_alloc += 1
                    # Store first var to global, return second
                    p1_stmts.append(('expr_stmt',
                        ('assign', '=',
                         ('index', ('var', xfer_name), ('num', 0)),
                         ('var', live_vars[0]))))
                    p1_stmts.append(('return', ('var', live_vars[1])))
                    p1_body = ('block', p1_stmts)
                    new_functions.append((p1_name, list(params), p1_body, 'u8'))
                    # Phase 2 takes second var as param, reads first from global
                    p2_stmts = [
                        ('local', live_vars[0],
                         ('index', ('var', xfer_name), ('num', 0)), 'u8')
                    ] + list(stmts[split:])
                    p2_body = ('block', p2_stmts)
                    new_functions.append((p2_name, [live_vars[1]], p2_body, ret_type))

                main_rewrites[name] = (p1_name, p2_name, len(live_vars))

                # Determine phase2 params
                p2_actual_params = [live_vars[0]] if len(live_vars) == 1 else [live_vars[1]]
                # Update func_params for the new functions
                self.func_params[p1_name] = len(params)
                self.func_params[p2_name] = len(p2_actual_params)
                self.func_bodies[p1_name] = (list(params), p1_body)
                self.func_bodies[p2_name] = (p2_actual_params, p2_body)
            else:
                import sys
                print(f"  Phase 7: {name} has {len(live_vars)} live vars "
                      f"at split — too many, keeping unsplit", file=sys.stderr)
                new_functions.append((name, params, body, ret_type))
                continue

        # Rewrite main's calls to split functions
        if main_rewrites:
            for i, (name, params, body, ret_type) in enumerate(new_functions):
                if name == 'main' and body[0] == 'block':
                    new_stmts = []
                    for s in body[1]:
                        rewritten = False
                        if s[0] == 'expr_stmt' and s[1][0] == 'call':
                            call_name = s[1][1]
                            if call_name in main_rewrites:
                                p1, p2, n_live = main_rewrites[call_name]
                                call_args = s[1][2]
                                # Phase 1: result = p1(args)
                                # Phase 2: p2(result) or p2(result, peek(254))
                                if n_live == 1:
                                    # temp = p1(args); p2(temp)
                                    new_stmts.append(('local', '__split_tmp', ('call', p1, call_args), 'u8'))
                                    new_stmts.append(('expr_stmt', ('call', p2, [('var', '__split_tmp')])))
                                elif n_live == 2:
                                    # tmp2 = p1(args); p2(tmp2)
                                    # (p1 stores first var to data[254], returns second)
                                    new_stmts.append(('local', '__split_tmp', ('call', p1, call_args), 'u8'))
                                    new_stmts.append(('expr_stmt', ('call', p2, [('var', '__split_tmp')])))
                                rewritten = True
                        if not rewritten:
                            new_stmts.append(s)
                    new_functions[i] = (name, params, ('block', new_stmts), ret_type)

        return new_functions

    def _is_builtin_i2c_send_byte_alias(self, name, params, body, ret_type):
        """Recognize the common looped I2C byte-send shim.

        Several older examples define their own `i2c_send_byte()` even though
        the compiler has a smaller known-good helper (`__i2c_sb`). Only alias
        the looped implementation that includes the trailing ACK clock; leave
        unrolled/custom senders alone because some probes intentionally handle
        ACK timing outside the function.
        """
        if name != 'i2c_send_byte' or len(params) != 1 or body[0] != 'block':
            return False
        p_name = params[0][0] if isinstance(params[0], tuple) else params[0]
        body_s = str(body)
        required = [
            "('for',",
            "('local', 'i'",
            f"('var', '{p_name}')",
            "('binop', '&'",
            "('num', 128)",
            "('binop', '<<'",
            "('num', 1)",
            "('call', 'exw', [('num', 0), ('num', 2), ('num', 0)])",
            "('call', 'exw', [('num', 64), ('num', 2), ('num', 0)])",
            "('call', 'exw', [('num', 128), ('num', 2), ('num', 0)])",
            "('call', 'exw', [('num', 192), ('num', 2), ('num', 0)])",
        ]
        if not all(token in body_s for token in required):
            return False
        # The looped LCD examples include a final ACK clock: 0,64,0 after the
        # loop. Require at least two low/released writes and two high writes so
        # we don't catch an unrolled no-ACK scan helper.
        return (body_s.count("('call', 'exw', [('num', 0), ('num', 2), ('num', 0)])") >= 2
                and body_s.count("('call', 'exw', [('num', 64), ('num', 2), ('num', 0)])") >= 2)

    def _alias_builtin_compatible_functions(self, functions):
        aliases = set()
        kept = []
        for name, params, body, ret_type in functions:
            if self._is_builtin_i2c_send_byte_alias(name, params, body, ret_type):
                aliases.add(name)
                continue
            kept.append((name, params, body, ret_type))
        if aliases:
            for name in aliases:
                self.func_params.pop(name, None)
                self.func_bodies.pop(name, None)
            self._builtin_function_aliases = aliases
        return kept

    def _simplify_function_bodies(self, functions):
        """Small AST rewrites before codegen.

        Keep this conservative: only remove a stack local when it is assigned
        once and immediately returned. This commonly appears in hardware
        wrapper functions (`t = rtc_read_temp(); return t;`) and otherwise
        costs several bytes in the overlay body.
        """
        def merge_local_init_runs(block):
            if not isinstance(block, tuple) or block[0] != 'block':
                return block
            stmts = list(block[1])
            out_stmts = []
            i = 0
            def is_call_expr(expr, name):
                return (isinstance(expr, tuple) and expr[0] == 'call'
                        and expr[1] == name)
            def is_call_stmt_args(stmt, name, args):
                return (isinstance(stmt, tuple) and stmt[0] == 'expr_stmt'
                        and isinstance(stmt[1], tuple) and stmt[1][0] == 'call'
                        and stmt[1][1] == name and stmt[1][2] == args)
            while i < len(stmts):
                if (i + 8 < len(stmts)
                        and self._is_call_stmt(stmts[i], 'i2c_start')
                        and self._is_call_stmt(stmts[i + 1], 'i2c_send_byte')
                        and is_call_stmt_args(stmts[i + 2], 'exw',
                                             [('num', 2), ('num', 2), ('num', 0)])
                        and is_call_stmt_args(stmts[i + 3], 'nop', [('num', 5)])
                        and is_call_stmt_args(stmts[i + 4], 'exw',
                                             [('num', 0), ('num', 2), ('num', 0)])
                        and is_call_stmt_args(stmts[i + 5], 'nop', [('num', 5)])
                        and isinstance(stmts[i + 6], tuple)
                        and stmts[i + 6][0] == 'local'
                        and is_call_expr(stmts[i + 6][2], 'via_read_portb')
                        and is_call_stmt_args(stmts[i + 7], 'exw',
                                             [('num', 2), ('num', 2), ('num', 0)])
                        and self._is_call_stmt(stmts[i + 8], 'i2c_stop')
                        and 'i2c_start' not in self.func_params
                        and 'i2c_send_byte' not in self.func_params
                        and 'i2c_stop' not in self.func_params
                        and 'via_read_portb' not in self.func_params):
                    # __i2c_sb already returns the ACK sample in A. Replace
                    # the hand-clocked ACK probe with the helper result while
                    # preserving the user-visible local value and STOP.
                    out_stmts.append(rewrite_stmt(stmts[i]))
                    out_stmts.append((
                        'local', stmts[i + 6][1], stmts[i + 1][1],
                        stmts[i + 6][3] if len(stmts[i + 6]) > 3 else 'u8'))
                    out_stmts.append(rewrite_stmt(stmts[i + 8]))
                    i += 9
                    continue
                # NOTE: a previous peephole here elided `i2c_stop(); i2c_start();
                # i2c_send_byte(read_addr);` to just `i2c_start(); i2c_send_byte;`
                # claiming this matched the compiler builtins' repeated-START
                # behaviour. It didn't — the compiler's `i2c_start` emits
                # `ddrb_imm 0x01` (SDA LOW, SCL HIGH) which expects the bus in
                # idle state (both HIGH). Coming out of `__i2c_sb` the bus is
                # `SDA released, SCL LOW`, so the simultaneous SCL-rise and
                # SDA-fall in that ddrb write is not a clean START — slaves
                # often miss it. The proven `readDS3231Temp` ESP32 program
                # does keep the STOP. Removed; the savings (3 ddrb writes)
                # don't justify the silent-miscompile risk on RTC/EEPROM
                # reads. Surfaced by overlay_dashboard.c which explicitly
                # codes `i2c_stop(); i2c_start();` and was producing
                # garbage byte reads (consistent 0xFE) at run time.
                s = stmts[i]
                if s[0] != 'local' or s[2] is not None:
                    out_stmts.append(rewrite_stmt(s))
                    i += 1
                    continue
                j = i
                locals_run = []
                while j < len(stmts) and stmts[j][0] == 'local' and stmts[j][2] is None:
                    locals_run.append(stmts[j])
                    j += 1
                assigns = []
                k = j
                ok = True
                for local_stmt in locals_run:
                    if k >= len(stmts):
                        ok = False
                        break
                    a = stmts[k]
                    if not (a[0] == 'expr_stmt' and a[1][0] == 'assign'
                            and a[1][1] == '=' and a[1][2] == ('var', local_stmt[1])
                            and not self._has_calls(a[1][3])):
                        ok = False
                        break
                    later_names = {ls[1] for ls in locals_run[locals_run.index(local_stmt)+1:]}
                    reads = set()
                    self._find_vars_in_expr(a[1][3], reads)
                    if reads & later_names:
                        ok = False
                        break
                    assigns.append(a[1][3])
                    k += 1
                if ok and assigns:
                    for local_stmt, init in zip(locals_run, assigns):
                        out_stmts.append((local_stmt[0], local_stmt[1], init,
                                          local_stmt[3] if len(local_stmt) > 3 else 'u8'))
                    i = k
                else:
                    out_stmts.append(rewrite_stmt(s))
                    i += 1
            return ('block', out_stmts)

        def is_lcd_char_digit(stmt, name):
            if (isinstance(stmt, tuple) and stmt[0] == 'block'
                    and len(stmt[1]) == 1):
                stmt = stmt[1][0]
            return (isinstance(stmt, tuple) and stmt[0] == 'expr_stmt'
                    and isinstance(stmt[1], tuple) and stmt[1][0] == 'call'
                    and stmt[1][1] == 'lcd_char'
                    and len(stmt[1][2]) == 1
                    and stmt[1][2][0] == ('binop', '+', ('var', name), ('num', 48)))

        def is_lcd_char_const(stmt, val):
            return (isinstance(stmt, tuple) and stmt[0] == 'expr_stmt'
                    and isinstance(stmt[1], tuple) and stmt[1][0] == 'call'
                    and stmt[1][1] == 'lcd_char'
                    and len(stmt[1][2]) == 1
                    and stmt[1][2][0] == ('num', val))

        def lcd_char_expr(stmt):
            if (isinstance(stmt, tuple) and stmt[0] == 'block'
                    and len(stmt[1]) == 1):
                stmt = stmt[1][0]
            if (isinstance(stmt, tuple) and stmt[0] == 'expr_stmt'
                    and isinstance(stmt[1], tuple) and stmt[1][0] == 'call'
                    and stmt[1][1] == 'lcd_char'
                    and len(stmt[1][2]) == 1):
                return stmt[1][2][0]
            return None

        def is_lcd_char_offset(stmt, name, offset):
            return lcd_char_expr(stmt) == ('binop', '+', ('var', name), ('num', offset))

        def is_hex_digit_if(stmt, name):
            return (isinstance(stmt, tuple) and stmt[0] == 'if'
                    and stmt[1] == ('binop', '>', ('var', name), ('num', 9))
                    and is_lcd_char_offset(stmt[2], name, 55)
                    and len(stmt) >= 4 and is_lcd_char_offset(stmt[3], name, 48))

        def is_high_nibble(expr):
            return (isinstance(expr, tuple) and expr[0] == 'binop'
                    and expr[1] == '>>' and expr[3] == ('num', 4))

        def is_low_nibble(expr):
            return (isinstance(expr, tuple) and expr[0] == 'binop'
                    and expr[1] == '&' and expr[3] == ('num', 0x0F))

        def compact_lcd_hex(block):
            """Lower common LCD hex formatting source to compact helpers.

            Recognizes both:
              - lcd_char((x >> 4) + '0'); lcd_char((x & 0x0F) + '0')
                for BCD-like output, and
              - hi/lo locals plus if (>9) +55 else +48 for hex output.
            """
            if not isinstance(block, tuple) or block[0] != 'block':
                return block
            stmts = list(block[1])
            out = []
            i = 0
            while i < len(stmts):
                if i + 1 < len(stmts):
                    e0 = lcd_char_expr(stmts[i])
                    e1 = lcd_char_expr(stmts[i + 1])
                    if (isinstance(e0, tuple) and isinstance(e1, tuple)
                            and e0[0] == 'binop' and e0[1] == '+'
                            and e0[3] == ('num', 48)
                            and e1[0] == 'binop' and e1[1] == '+'
                            and e1[3] == ('num', 48)
                            and is_high_nibble(e0[2]) and is_low_nibble(e1[2])
                            and e0[2][2] == e1[2][2]
                            and not self._has_calls(e0[2][2])):
                        out.append(('expr_stmt',
                                    ('call', 'lcd_bcd_u8', [e0[2][2]])))
                        i += 2
                        continue
                if i + 3 < len(stmts):
                    h, l, hi_if, lo_if = stmts[i:i+4]
                    if (h[0] == 'local' and l[0] == 'local'
                            and h[2] is not None and l[2] is not None
                            and is_high_nibble(h[2]) and is_low_nibble(l[2])
                            and h[2][2] == l[2][2]
                            and is_hex_digit_if(hi_if, h[1])
                            and is_hex_digit_if(lo_if, l[1])
                            and not self._has_calls(h[2][2])):
                        out.append(('expr_stmt',
                                    ('call', 'lcd_hex_u8', [h[2][2]])))
                        i += 4
                        continue
                out.append(stmts[i])
                i += 1
            return ('block', out)

        def compact_i2c_scan(block):
            """Lower a common I2C bus scanner into a compact primitive.

            Source shape:
              count=0; for (addr=start; addr<end; addr++) {
                i2c_start(); val=i2c_send_byte(addr << 1); i2c_stop();
                if ((val & 1) == 0) { results[count] = addr; count++; }
              }
              results[count] = 0;
            The helper keeps count/addr in registers and stores a sentinel.
            """
            if not isinstance(block, tuple) or block[0] != 'block':
                return block
            stmts = list(block[1])
            out = []
            i = 0
            while i < len(stmts):
                if i + 3 < len(stmts):
                    s0, s1, loop, sentinel = stmts[i:i+4]
                    # Accept either local ordering: count first (init=0)
                    # then addr (uninit), or addr first then count. The
                    # natural source `unsigned char addr; unsigned char
                    # count; count = 0; for(addr...)` defeats
                    # merge_local_init_runs and leaves both uninit, so
                    # the original count_decl-must-be-init guard wouldn't
                    # match. Accept both shapes.
                    count_decl = addr_decl = None
                    if (s0[0] == 'local' and s0[2] == ('num', 0)
                            and s1[0] == 'local' and s1[2] is None):
                        count_decl, addr_decl = s0, s1
                    elif (s1[0] == 'local' and s1[2] == ('num', 0)
                            and s0[0] == 'local' and s0[2] is None):
                        count_decl, addr_decl = s1, s0
                    if not (count_decl is not None and loop[0] == 'for'):
                        out.append(stmts[i]); i += 1; continue
                    count_name = count_decl[1]
                    addr_name = addr_decl[1]
                    init, cond, update, body = loop[1], loop[2], loop[3], loop[4]
                    if not (isinstance(init, tuple)
                            and init[0] == 'expr_stmt'
                            and isinstance(init[1], tuple)
                            and init[1][0] == 'assign'
                            and init[1][1] == '='
                            and init[1][2] == ('var', addr_name)
                            and cond[0] == 'binop' and cond[1] == '<'
                            and cond[2] == ('var', addr_name)
                            and update == ('postinc', ('var', addr_name))
                            and isinstance(body, tuple) and body[0] == 'block'):
                        out.append(stmts[i]); i += 1; continue
                    start_expr = init[1][3]
                    end_expr = cond[3]
                    start = self._const_eval(start_expr)
                    end = self._const_eval(end_expr)
                    b = body[1]
                    if not (start is not None and end is not None
                            and len(b) == 4
                            and self._is_call_stmt(b[0], 'i2c_start')
                            and b[1][0] == 'local'
                            and isinstance(b[1][2], tuple)
                            and b[1][2][0] == 'call'
                            and b[1][2][1] == 'i2c_send_byte'
                            and b[1][2][2] == [('binop', '<<', ('var', addr_name), ('num', 1))]
                            and self._is_call_stmt(b[2], 'i2c_stop')
                            and b[3][0] == 'if'):
                        out.append(stmts[i]); i += 1; continue
                    val_name = b[1][1]
                    if_stmt = b[3]
                    if not (if_stmt[1] == ('binop', '==',
                                            ('binop', '&', ('var', val_name), ('num', 1)),
                                            ('num', 0))
                            and isinstance(if_stmt[2], tuple)
                            and if_stmt[2][0] == 'block'
                            and (len(if_stmt) < 4 or if_stmt[3] is None)):
                        out.append(stmts[i]); i += 1; continue
                    ib = if_stmt[2][1]
                    if not (len(ib) == 2
                            and ib[0][0] == 'expr_stmt'
                            and isinstance(ib[0][1], tuple)
                            and ib[0][1][0] == 'assign'
                            and ib[0][1][1] == '='
                            and isinstance(ib[0][1][2], tuple)
                            and ib[0][1][2][0] == 'index'
                            and ib[0][1][2][2] == ('var', count_name)
                            and ib[0][1][3] == ('var', addr_name)
                            and ib[1] == ('expr_stmt', ('postinc', ('var', count_name)))):
                        out.append(stmts[i]); i += 1; continue
                    arr_expr = ib[0][1][2][1]
                    if not (isinstance(arr_expr, tuple) and arr_expr[0] == 'var'):
                        out.append(stmts[i]); i += 1; continue
                    arr_name = arr_expr[1]
                    if not (sentinel[0] == 'expr_stmt'
                            and sentinel[1] == ('assign', '=',
                                                ('index', ('var', arr_name), ('var', count_name)),
                                                ('num', 0))):
                        out.append(stmts[i]); i += 1; continue
                    if arr_name not in self.globals:
                        out.append(stmts[i]); i += 1; continue
                    cap = max(0, self.global_sizes.get(arr_name, 1) - 1)
                    out.append(('expr_stmt',
                                ('call', 'i2c_scan_results',
                                 [('num', self.globals[arr_name]),
                                  ('num', start & 0xFF),
                                  ('num', end & 0xFF),
                                  ('num', cap & 0xFF)])))
                    i += 4
                    continue
                out.append(stmts[i])
                i += 1
            return ('block', out)

        def compact_lcd_hex_results(block):
            """Lower zero-terminated byte-array hex display loops.

            Recognizes loops that print each non-zero result as
            `" " + hex(byte)` and then output the count. This is the natural
            companion to I2C scan result buffers, but it is phrased as a
            general LCD byte-list formatter.
            """
            if not isinstance(block, tuple) or block[0] != 'block':
                return block
            stmts = list(block[1])
            out = []
            i = 0
            while i < len(stmts):
                if i + 3 < len(stmts):
                    s0, s1, loop, out_stmt = stmts[i:i+4]
                    # Accept either local ordering (idx-first or count-first).
                    # See compact_i2c_scan for the same brittleness rationale.
                    idx_decl = count_decl = None
                    if (s0[0] == 'local' and s0[2] is None
                            and s1[0] == 'local' and s1[2] == ('num', 0)):
                        idx_decl, count_decl = s0, s1
                    elif (s1[0] == 'local' and s1[2] is None
                            and s0[0] == 'local' and s0[2] == ('num', 0)):
                        idx_decl, count_decl = s1, s0
                    if not (idx_decl is not None
                            and loop[0] == 'for'
                            and self._is_call_stmt(out_stmt, 'out')
                            and out_stmt[1][2] == [('var', count_decl[1])]):
                        out.append(stmts[i]); i += 1; continue
                    idx_name = idx_decl[1]
                    count_name = count_decl[1]
                    init, cond, update, body = loop[1], loop[2], loop[3], loop[4]
                    if not (isinstance(init, tuple)
                            and init == ('expr_stmt', ('assign', '=', ('var', idx_name), ('num', 0)))
                            and isinstance(cond, tuple)
                            and cond[0] == 'binop' and cond[1] == '<'
                            and cond[2] == ('var', idx_name)
                            and update == ('postinc', ('var', idx_name))
                            and isinstance(body, tuple) and body[0] == 'block'):
                        out.append(stmts[i]); i += 1; continue
                    limit = self._const_eval(cond[3])
                    b = body[1]
                    if limit is None or len(b) != 5:
                        out.append(stmts[i]); i += 1; continue
                    first_if = b[0]
                    if not (first_if[0] == 'if'
                            and first_if[1][0] == 'binop'
                            and first_if[1][1] == '=='
                            and first_if[1][3] == ('num', 0)
                            and first_if[2] == ('break',)
                            and (len(first_if) < 4 or first_if[3] is None)):
                        out.append(stmts[i]); i += 1; continue
                    arr_idx = first_if[1][2]
                    if not (isinstance(arr_idx, tuple) and arr_idx[0] == 'index'
                            and arr_idx[2] == ('var', idx_name)
                            and isinstance(arr_idx[1], tuple)
                            and arr_idx[1][0] == 'var'):
                        out.append(stmts[i]); i += 1; continue
                    arr_name = arr_idx[1][1]
                    if arr_name not in self.globals:
                        out.append(stmts[i]); i += 1; continue
                    if not (self._is_call_stmt(b[1], 'lcd_char')
                            and b[1][1][2] == [('num', 32)]
                            and b[2][0] == 'expr_stmt'
                            and b[2][1][0] == 'assign'
                            and b[2][1][1] == '='
                            and b[2][1][3] == ('index', ('var', arr_name), ('var', idx_name))
                            and b[3][0] == 'expr_stmt'
                            and isinstance(b[3][1], tuple)
                            and b[3][1][0] == 'call'
                            and b[4] == ('expr_stmt', ('postinc', ('var', count_name)))):
                        out.append(stmts[i]); i += 1; continue
                    out.append(('expr_stmt',
                                ('call', 'out',
                                 [('call', 'lcd_hex_results',
                                   [('num', self.globals[arr_name]),
                                    ('num', limit & 0xFF)])])))
                    i += 4
                    continue
                out.append(stmts[i])
                i += 1
            return ('block', out)

        def decimal_loop_vars(stmt):
            if not (isinstance(stmt, tuple) and stmt[0] == 'while'):
                return None
            cond = stmt[1]
            body = stmt[2]
            if not (isinstance(cond, tuple)
                    and cond[0] == 'binop' and cond[1] == '>='
                    and cond[3] == ('num', 10)
                    and isinstance(cond[2], tuple) and cond[2][0] == 'var'):
                return None
            ones = cond[2][1]
            if not (isinstance(body, tuple) and body[0] == 'block'
                    and len(body[1]) == 2):
                return None
            s0, s1 = body[1]
            if not (s0[0] == 'expr_stmt' and s0[1][0] == 'assign'
                    and s0[1][1] == '=' and s0[1][2] == ('var', ones)
                    and s0[1][3] == ('binop', '-', ('var', ones), ('num', 10))):
                return None
            if not (s1[0] == 'expr_stmt' and s1[1][0] == 'assign'
                    and s1[1][1] == '=' and isinstance(s1[1][2], tuple)
                    and s1[1][2][0] == 'var'
                    and s1[1][3] == ('binop', '+', s1[1][2], ('num', 1))):
                return None
            return s1[1][2][1], ones

        def is_optional_tens_print(stmt, tens):
            return (isinstance(stmt, tuple) and stmt[0] == 'if'
                    and stmt[1] == ('binop', '>', ('var', tens), ('num', 0))
                    and is_lcd_char_digit(stmt[2], tens)
                    and (len(stmt) < 4 or stmt[3] is None))

        def compact_lcd_temp(block):
            if not isinstance(block, tuple) or block[0] != 'block':
                return block
            stmts = list(block[1])
            out = []
            i = 0
            while i < len(stmts):
                # Merged-init form: after merge_local_init_runs collapses
                #   local tens; local ones; tens=0; ones=expr;
                # into
                #   local tens=0; local ones=expr;
                # the prefix-scanning branch below cannot match (its boundary
                # check requires uninit locals + bare assigns). Recognise the
                # merged shape directly: 7 stmts = 2 init-locals + 5 print
                # tail (loop, optional-tens, ones_print, deg, cchar).
                if i + 6 < len(stmts):
                    d0, d1 = stmts[i], stmts[i + 1]
                    if (d0[0] == 'local' and d0[2] == ('num', 0)
                            and d1[0] == 'local' and d1[2] is not None):
                        loop, ifs, ones_print, deg, cchar = stmts[i+2:i+7]
                        vars_ = decimal_loop_vars(loop)
                        if (vars_ == (d0[1], d1[1])
                                and is_optional_tens_print(ifs, d0[1])
                                and is_lcd_char_digit(ones_print, d1[1])
                                and is_lcd_char_const(deg, 0xDF)
                                and is_lcd_char_const(cchar, ord('C'))):
                            out.append(('expr_stmt',
                                        ('call', 'lcd_temp_u8', [d1[2]])))
                            i += 7
                            continue
                if i + 7 < len(stmts):
                    d0, d1 = stmts[i], stmts[i + 1]
                    if d0[0] == 'local' and d0[2] is None \
                            and d1[0] == 'local' and d1[2] is None:
                        j = i + 2
                        prefix = []
                        while j + 6 < len(stmts):
                            a0, a1 = stmts[j], stmts[j + 1]
                            if (a0[0] == 'expr_stmt' and a0[1][0] == 'assign'
                                    and a0[1][1] == '='
                                    and a0[1][2] == ('var', d0[1])
                                    and a0[1][3] == ('num', 0)
                                    and a1[0] == 'expr_stmt'
                                    and a1[1][0] == 'assign'
                                    and a1[1][1] == '='
                                    and a1[1][2] == ('var', d1[1])):
                                break
                            reads = set()
                            self._find_vars_in_stmt(stmts[j], set(), reads)
                            if d0[1] in reads or d1[1] in reads:
                                break
                            prefix.append(rewrite_stmt(stmts[j]))
                            j += 1
                        if j + 6 < len(stmts):
                            a0, a1, loop, ifs, ones_print, deg, cchar = stmts[j:j+7]
                            if (a0[0] == 'expr_stmt' and a0[1][0] == 'assign'
                                    and a0[1][1] == '=' and a0[1][2] == ('var', d0[1])
                                    and a0[1][3] == ('num', 0)
                                    and a1[0] == 'expr_stmt' and a1[1][0] == 'assign'
                                    and a1[1][1] == '=' and a1[1][2] == ('var', d1[1])):
                                vars_ = decimal_loop_vars(loop)
                                if (vars_ == (d0[1], d1[1])
                                        and is_optional_tens_print(ifs, d0[1])
                                        and is_lcd_char_digit(ones_print, d1[1])
                                        and is_lcd_char_const(deg, 0xDF)
                                        and is_lcd_char_const(cchar, ord('C'))):
                                    out.extend(prefix)
                                    out.append(('expr_stmt',
                                                ('call', 'lcd_temp_u8', [a1[1][3]])))
                                    i = j + 7
                                    continue
                out.append(rewrite_stmt(stmts[i]))
                i += 1
            return ('block', out)

        def simplify_block(block):
            merged = merge_local_init_runs(block)
            return compact_lcd_temp(compact_lcd_hex(
                compact_lcd_hex_results(compact_i2c_scan(merged))))

        def rewrite_stmt(stmt):
            if not isinstance(stmt, tuple):
                return stmt
            kind = stmt[0]
            if kind == 'block':
                return simplify_block(stmt)
            if kind == 'if':
                else_part = rewrite_stmt(stmt[3]) if len(stmt) > 3 and stmt[3] else None
                return (kind, stmt[1], rewrite_stmt(stmt[2]),
                        else_part)
            if kind in ('while', 'do_while'):
                return (kind, stmt[1], rewrite_stmt(stmt[2]))
            if kind == 'for':
                return (kind, stmt[1], stmt[2], stmt[3], rewrite_stmt(stmt[4]))
            return stmt

        # Auto-detect compiler-injected calls: any AST-rewrite matcher
        # that lowers a user pattern into a call to a different builtin
        # name introduces a call name that wasn't in the user's source.
        # Capture pre vs post call-name set per function; the delta is
        # the set of "matcher fires" for that function. Future matchers
        # that synthesize new call nodes get tracked automatically — no
        # per-matcher instrumentation. Output via --metrics-out so a
        # corpus regression baseline can detect "matchers that used to
        # fire on program X but stopped firing" without humans having
        # to remember which matchers exist.
        def _collect_call_names(node, acc):
            if isinstance(node, tuple):
                if len(node) >= 2 and node[0] == 'call':
                    acc.add(node[1])
                for c in node:
                    _collect_call_names(c, acc)
            elif isinstance(node, list):
                for c in node:
                    _collect_call_names(c, acc)

        if not hasattr(self, '_matcher_fires'):
            self._matcher_fires = {}  # fn -> set of injected call names
        out = []
        for name, params, body, ret_type in functions:
            pre_calls = set()
            _collect_call_names(body, pre_calls)
            if body[0] == 'block':
                body = simplify_block(body)
                stmts = body[1]
                if (len(stmts) == 3
                        and stmts[0][0] == 'local'
                        and stmts[1][0] == 'expr_stmt'
                        and stmts[1][1][0] == 'assign'
                        and stmts[1][1][1] == '='
                        and stmts[1][1][2] == ('var', stmts[0][1])
                        and stmts[2][0] == 'return'
                        and stmts[2][1] == ('var', stmts[0][1])
                        and stmts[0][2] is None):
                    body = ('block', [('return', stmts[1][1][3])])
                    self._matcher_fires.setdefault(name, set()).add(
                        '<trivial-return-wrapper>')
            post_calls = set()
            _collect_call_names(body, post_calls)
            injected = post_calls - pre_calls
            if injected:
                self._matcher_fires.setdefault(name, set()).update(injected)
            out.append((name, params, body, ret_type))
        return out

    def _is_call_stmt(self, stmt, name):
        return (isinstance(stmt, tuple) and stmt[0] == 'expr_stmt'
                and isinstance(stmt[1], tuple) and stmt[1][0] == 'call'
                and stmt[1][1] == name)

    def _collect_reg_candidates(self, stmts, candidates, depth=0):
        """Recursively collect register allocation candidates.
        Deeper-nested variables get higher priority (negative depth for sorting)."""
        for i, s in enumerate(stmts):
            name = None
            if s[0] == 'local':
                name = s[1]
            elif (i + 1 < len(stmts) and s[0] == 'local' and s[2] is None
                  and stmts[i+1][0] == 'expr_stmt'
                  and stmts[i+1][1][0] == 'assign' and stmts[i+1][1][1] == '='
                  and stmts[i+1][1][2] == ('var', s[1])):
                name = s[1]

            if name and name in self.read_vars:
                has_call = any(self._has_calls(stmts[j]) for j in range(i + 1, len(stmts)))
                if not has_call:
                    candidates.append((-depth, name))  # deeper = more negative = sorted first

            # Recurse into inner blocks that have NO user function calls
            inner_stmts = None
            if s[0] == 'for' and s[4] and s[4][0] == 'block':
                inner_stmts = s[4][1]
            elif s[0] == 'while' and s[2] and s[2][0] == 'block':
                inner_stmts = s[2][1]
            elif s[0] == 'do_while' and s[1] and s[1][0] == 'block':
                inner_stmts = s[1][1]
            elif s[0] == 'block':
                inner_stmts = s[1]
            elif s[0] == 'if':
                # Check then/else branches
                if s[2] and s[2][0] == 'block':
                    self._collect_reg_candidates(s[2][1], candidates)
                if s[3] and s[3][0] == 'block':
                    self._collect_reg_candidates(s[3][1], candidates)

            if inner_stmts is not None:
                has_inner_calls = any(self._has_calls(st) for st in inner_stmts)
                if not has_inner_calls:
                    self._collect_reg_candidates(inner_stmts, candidates, depth + 1)

    def _find_reg_candidates(self, stmts):
        """Find locals eligible for register allocation.
        Returns (c_var, d_var) — names for C and D registers, or None.
        C: safe if no function calls after declaration.
        D: safe if no function calls AND no stsp after declaration.
        Also searches inner blocks (for/while/do-while bodies) when
        the inner scope has no user function calls."""
        c_var = None
        d_var = None
        candidates = []
        self._collect_reg_candidates(stmts, candidates)
        # Sort by depth (deepest first), remove duplicates
        candidates.sort()  # (-depth, name): most negative depth first
        seen = set()
        unique = []
        for _, name in candidates:
            if name not in seen:
                seen.add(name)
                unique.append(name)
        # D is unsafe if the function body has stsp (clobbers D) or xor (uses D as scratch)
        d_unsafe = self._has_stsp(('block', stmts)) or self._has_xor(('block', stmts))

        for name in unique:
            if not c_var:
                c_var = name
            elif not d_var and not d_unsafe:
                d_var = name
                break

        return c_var, d_var

    # ── Compilation ──────────────────────────────────────────────────

    def compile(self, source):
        tokens = tokenize(source)
        parser = Parser(tokens)
        globals_, functions = parser.parse_program()

        self.page3_globals = {}  # name → page 3 address (for overflow arrays)
        self.page3_alloc = 0
        self.eeprom_globals = {}  # name → EEPROM address
        self.eeprom_alloc = 0x0010  # skip 16-byte header (checksum + reserved)
        self.eeprom_data = []  # list of (addr, bytes) for upload
        self.global_sizes = {name: (len(init) if isinstance(init, list) else 1)
                             for name, init, _storage in globals_}

        for name, init, storage in globals_:
            size = len(init) if isinstance(init, list) else 1
            if storage == 'eeprom':
                # Store in EEPROM — must be initialized array
                if not isinstance(init, list):
                    raise Exception(f"eeprom variable '{name}' must be an initialized array")
                self.eeprom_globals[name] = self.eeprom_alloc
                self.eeprom_data.append((self.eeprom_alloc, init))
                self.eeprom_alloc += size
            elif self.data_alloc + size <= 256:
                self.globals[name] = self.data_alloc
                self.data_alloc += size
            else:
                # Overflow to page 3
                self.page3_globals[name] = self.page3_alloc
                self.page3_alloc += size

        # Pre-pass: record param counts and bodies for optimization
        self.func_params = {}
        self.func_bodies = {}  # for compile-time evaluation of pure functions
        for name, params, body, ret_type in functions:
            self.func_params[name] = len(params)
            self.func_bodies[name] = (params, body)

        functions = self._alias_builtin_compatible_functions(functions)
        functions = self._simplify_function_bodies(functions)

        # ── Phase 7: Auto function splitting ──
        # Split functions that span both I2C and LCD domains into separate
        # phases, each needing fewer helpers → smaller overlay slots.
        functions = self._auto_split_functions(functions)

        # Refresh func_bodies to reflect the final post-transform function
        # list, then compute the $d-clobber transitive closure. The
        # reg-alloc pass in compile_function() consults
        # `self._d_clobber_funcs` to decide whether param 0 can be saved to
        # $d or must be spilled to the stack. This must run after
        # `_auto_split_functions` because that pass can introduce new
        # phase-split functions.
        self.func_bodies = {name: (params, body)
                            for (name, params, body, _r) in functions}
        self._compute_d_clobber_funcs()

        # Compile main first (starts at address 0, no j _main jump needed)
        # Then compile other functions after main
        main_fn = None
        other_fns = []
        for fn in functions:
            if fn[0] == 'main':
                main_fn = fn
            else:
                other_fns.append(fn)

        # Pre-scan: check if any function calls lcd_cmd (needs calibrated delays)
        # Must know this BEFORE compilation so lcd_init can auto-insert delay_cal.
        # Skip this detection if the user has defined their own `lcd_cmd` —
        # the builtin's calibrated delay isn't needed for a user implementation.
        all_fn_bodies = [main_fn] + other_fns if main_fn else other_fns
        user_fn_names = {fn[0] for fn in all_fn_bodies}
        if 'lcd_cmd' not in user_fn_names:
            for fn in all_fn_bodies:
                # fn = (name, params, body, ret_type)
                src_str = str(fn[2])  # crude but effective
                if 'lcd_cmd' in src_str:
                    self._needs_delay_calibrate = True
                    break

        if main_fn:
            self.compile_function(*main_fn)

        for fn in other_fns:
            self.compile_function(*fn)

        # In --eeprom mode, the preload uses only __i2c_sb (inline sequential
        # read — no __eeprom_rd / __i2c_rb function dependencies). Ensure
        # __i2c_sb is emitted even if user code doesn't otherwise reference it.
        if getattr(self, 'eeprom_mode', False):
            if not hasattr(self, '_lcd_helpers'):
                self._lcd_helpers = set()
            self._lcd_helpers.add('__i2c_sb')

        # Auto-insert delay_calibrate when lcd_cmd uses calibrated delays.
        # The call is inserted into _main (after VIA/LCD init, before user code).
        # In overlay mode, __delay_cal becomes an overlay-eligible function.
        if (getattr(self, '_needs_delay_calibrate', False)
                and not getattr(self, '_explicit_delay_calibrate', False)):
            if not hasattr(self, '_lcd_helpers'):
                self._lcd_helpers = set()
            self._lcd_helpers.add('__delay_cal')
            # delay_cal inlines STOP — no __i2c_sp dependency
            # Insert jal __delay_cal into _main after the last init-compatible call
            for mi in range(len(self.code) - 1, -1, -1):
                s = self.code[mi].strip()
                if s == 'jal __lcd_init' or s == 'jal __delay_cal':
                    self.code.insert(mi + 1, '\tjal __delay_cal')
                    break
            else:
                # No lcd_init found — insert at _main start
                for mi in range(len(self.code)):
                    if self.code[mi].strip() == '_main:':
                        self.code.insert(mi + 1, '\tjal __delay_cal')
                        break

        # Emit I2C/LCD helpers AFTER all functions
        self._emit_i2c_helpers()

        # Emit tone init code (ddra + precomp loop) as prefix to _main.
        # Must be AFTER all tone()/silence() calls are compiled (so note table is complete)
        # and BEFORE dead function elimination.
        #
        # Flat-mode precompute is inserted inline because there is no overlay
        # loader. EEPROM mode emits __tone_precompute below and calls it after
        # delay_calibrate(), so it sees the calibrated page3[240] value without
        # growing the permanent runtime kernel.
        if (getattr(self, '_needs_tone_init', False) and
            hasattr(self, '_note_table') and self._note_table and
            not getattr(self, 'eeprom_mode', False)):
            note_end = self.data_alloc
            for mi, ml in enumerate(self.code):
                if ml.strip() == '_main:':
                    # Find insertion point: after i2c_init + delay_calibrate inline code
                    # (look for the last jal __delay_cal or the last ;!keep line)
                    insert_at = mi + 1
                    for j in range(mi + 1, min(mi + 30, len(self.code))):
                        s = self.code[j].strip()
                        if s == 'jal __delay_cal' or s.endswith(';!keep'):
                            insert_at = j + 1
                    precomp = [
                        '\tldi $a,0',
                        '\tmov $a,$d',               # D = note ptr
                        '.__precomp:',
                        '\tmov $d,$a',
                        '\tderef',                   # A = data[ptr] = ratio
                        '\ttst 0xFF',
                        '\tjz .__preskip',            # silence (ratio=0)
                        # Save D (note ptr) via push_b/pop_b (push $d + jal conflicts)
                        '\tmov $d,$b',               # B = note ptr
                        '\tpush_b',                  # save note ptr, SP--
                        '\tmov $a,$b',               # B = ratio
                        '\tjal __tone_setup',         # C = half_period
                        '\tpop_b',                   # B = note ptr, SP++
                        '\tmov $c,$a',               # A = half_period
                        '\tideref',                   # data[B] = half_period
                        '\tmov $b,$d',               # D = note ptr (restore)
                        '.__preskip:',
                        '\tmov $d,$a',
                        '\taddi 3,$a',
                        '\tmov $a,$d',
                        f'\tcmpi {note_end}',
                        '\tjnz .__precomp',
                    ]
                    self.code[insert_at:insert_at] = precomp
                    break

        if (getattr(self, '_needs_tone_init', False) and
            hasattr(self, '_note_table') and self._note_table and
            getattr(self, 'eeprom_mode', False)):
            note_base = self._note_table[0][0]
            note_end = self._note_table[-1][0] + 3
            precomp_call_inserted = False
            for mi, ml in enumerate(self.code):
                if ml.strip() == '_main:':
                    insert_at = mi + 1
                    for j in range(mi + 1, min(mi + 40, len(self.code))):
                        if self.code[j].strip() == 'jal __delay_cal':
                            insert_at = j + 1
                    self.code.insert(insert_at, '\tjal __tone_precompute')
                    precomp_call_inserted = True
                    break
            if precomp_call_inserted:
                self.code.extend([
                    '__tone_precompute:',
                    # Idempotency guard: the precompute converts each note
                    # table entry's first byte from `ratio` to `half_period`
                    # IN PLACE. Without this guard, a second `_main` run
                    # after RESET (without re-upload) would call __tone_setup
                    # on already-precomputed half_periods, cascading into
                    # garbage values — empirically observed as twinkle
                    # playing low/slow on the 2nd run and completely wrong
                    # on the 3rd. The kernel-state slot is zero on upload
                    # and persists across MK1 resets, making it a safe
                    # one-shot latch.
                    *self._kstate_load_lines('tone_precomp_done'),
                    '\ttst 0xFF',
                    '\tjnz .__tpc_already',
                    f'\tldi $a,{note_base}',
                    '\tmov $a,$d',
                    '.__tpc_loop:',
                    '\tmov $d,$a',
                    '\tderef',
                    '\ttst 0xFF',
                    '\tjz .__tpc_skip',
                    '\tmov $d,$b',
                    '\tpush_b',
                    '\tmov $a,$b',
                    '\tjal __tone_setup',
                    '\tpop_b',
                    '\tmov $c,$a',
                    '\tideref',
                    '\tmov $b,$d',
                    '.__tpc_skip:',
                    '\tincd',
                    '\tincd',
                    '\tincd',
                    f'\tcmpi {note_end}',
                    '\tjnz .__tpc_loop',
                    '\tldi $a,1',
                    *self._kstate_store_lines('tone_precomp_done'),
                    '.__tpc_already:',
                    '\tret',
                ])

        # Dead function elimination
        self._eliminate_dead_functions()

        # Classify helpers (detect runtime I2C, build dynamic _NO_OVERLAY)
        self._classify_helpers()

        # Cache page 1 globals for _overlay_partition: it emits overlay data
        # in the same data buffer and needs to interleave globals up front so
        # they land at the offsets the allocator assigned.
        self._page1_globals_cache = [(n, i) for n, i, s in globals_ if n in self.globals]

        # ── Helper-registration validator ──
        # Every `jal __X` must resolve to a `__X:` definition somewhere in
        # `self.code`. If a builtin emitter registers helper A but A's body
        # internally jumps to helper B which wasn't separately registered,
        # B never gets emitted and we end up with a dangling jal target.
        # Two recent bugs of this exact shape:
        #   - lcd_init() registered __lcd_init but not __i2c_st_only / __i2c_sp
        #     (which __lcd_init's body jal's to). Latent for typical LCD
        #     programs; surfaced as a 3:46 init-helper-resolver hang on
        #     test_i2c_ack_diag.c.
        #   - silence() registered __play_note but not __tone (which
        #     __play_note's body always jal's). Silence-only programs
        #     emitted a dangling jal __tone; caught only at assembly time.
        #
        # Catch this class at compile time, before partitioning, with a
        # clear error naming the unresolved target(s).
        _defined_labels = set()
        _jal_targets = set()
        for _l in self.code:
            _s = _l.strip()
            if _s.endswith(':') and not _s.startswith('.') and not _s.startswith(';'):
                _defined_labels.add(_s[:-1])
            for _prefix in ('jal ', 'j ', 'ocall '):
                if _s.startswith(_prefix):
                    _tgt = _s[len(_prefix):].split()[0].split(';')[0].strip()
                    # Only check helper-style targets (start with __); user
                    # function targets and numeric targets are resolved by
                    # the partitioner / assembler.
                    if _tgt.startswith('__') and not _tgt[0].isdigit():
                        _jal_targets.add(_tgt)
        _unresolved = _jal_targets - _defined_labels
        if _unresolved:
            raise Exception(
                f"Helper registration miss: jal target(s) {sorted(_unresolved)} "
                f"are referenced but never emitted. Likely a builtin emitter "
                f"registered a helper without registering its transitive "
                f"dependencies. Check `_lcd_helpers.add(...)` in the relevant "
                f"`if name == '...':' block; the helper body emitted in "
                f"`_emit_i2c_helpers` is the source of truth for required deps."
            )

        # Two-stage boot overlay partitioning
        self._overlay_partition()

        # Flat mode SP init: in overlay/init-extract modes, _main gets a
        # preamble (`ldi $b,0xFF; mov $b,$sp`) that initializes SP. Flat
        # mode skips that. Without SP init, the first push runs at SP=0
        # (reset), wraps SP to 0xFF, and the next stsp at SP=0xFF triggers
        # the unfixed stsp microcode carry race (writes the wrong stack
        # slot when SP+N wraps). Inject the same 3B preamble here so flat
        # programs have the same invariant.
        flat_mode = (getattr(self, '_overlay_kernel_size', None) is None
                     and not getattr(self, '_init_extraction_done', False))
        if flat_mode:
            for mi, line in enumerate(self.code):
                if line.strip() == '_main:':
                    self.code.insert(mi + 1, '\tldi $b,0xFF')
                    self.code.insert(mi + 2, '\tmov $b,$sp')
                    break

        # Emit page 1 globals ONLY if overlay partitioning didn't inline them.
        # Overlay mode emits them inside the data section before overlay bodies;
        # skip the separate emission here to avoid double-defining the labels.
        page1_vars = self._page1_globals_cache
        overlays_handled_globals = getattr(self, '_overlay_kernel_size', None) is not None
        if (page1_vars and not getattr(self, '_init_extraction_done', False)
                and not overlays_handled_globals):
            self.emit('\tsection data')
            for name, init in page1_vars:
                self.emit(f'_{name}:')
                if isinstance(init, list):
                    for v in init:
                        self.emit(f'\tbyte {v}')
                else:
                    self.emit(f'\tbyte {init}')
            self.emit('\tsection code')

        # Emit page 3 globals
        page3_vars = [(n, i) for n, i, s in globals_ if n in self.page3_globals]
        if page3_vars and not getattr(self, '_init_extraction_done', False):
            self.emit('\tsection page3')
            for name, init in page3_vars:
                self.emit(f'_{name}:')
                if isinstance(init, list):
                    for v in init:
                        self.emit(f'\tbyte {v}')
                else:
                    self.emit(f'\tbyte {init}')
            self.emit('\tsection code')

        # LCD init data pre-populated in data page (compile-time).
        # Skip if overlay partition already emitted it at the right offset
        # (see _lcd_init_already_emitted flag set in overlay path).
        if hasattr(self, '_lcd_init_data') and not getattr(self, '_lcd_init_already_emitted', False):
            data_base, lcd_bytes = self._lcd_init_data
            self.emit('\tsection data')
            self.emit(f'; LCD init data at data[{data_base}..{data_base+len(lcd_bytes)-1}]')
            for b in lcd_bytes:
                self.emit(f'\tbyte {b}')
            self.emit('\tsection code')

        # Emit lcd_print string data in page 1 (data section). Each
        # string carries its own symbolic label `__str_N`; the
        # assembler resolves it to the data-page offset where the
        # bytes physically land. Storing in page 1 avoids the page-3
        # kernel-image collision that the previous implementation
        # silently produced.
        if hasattr(self, '_lcd_print_strings') and not getattr(self, '_init_extraction_done', False):
            self.emit('\tsection data')
            for label, s, d_off in self._lcd_print_strings:
                self.emit(f'; string at data offset {d_off}')
                self.emit(f'{label}:')
                for ch in s:
                    self.emit(f'\tbyte {ord(ch)}')
                self.emit('\tbyte 0')  # null terminator
            self.emit('\tsection code')

        # Emit note table data in page 1 (data page)
        if hasattr(self, '_note_table') and not getattr(self, '_note_table_already_emitted', False):
            self.emit('\tsection data')
            for p1_off, ratio, cyc_lo, cyc_hi in self._note_table:
                self.emit(f'; note at data offset {p1_off}: ratio={ratio} cyc={cyc_hi}:{cyc_lo}')
                self.emit(f'\tbyte {ratio}')
                self.emit(f'\tbyte {cyc_lo}')
                self.emit(f'\tbyte {cyc_hi}')
            self.emit('\tsection code')

        # Emit EEPROM data section (for ESP32 upload shim)
        if self.eeprom_data:
            self.emit('\tsection eeprom')
            # Checksum at addr 0x0000-0x0001
            all_bytes = []
            for addr, data in self.eeprom_data:
                all_bytes.extend(data)
            cksum = sum(all_bytes) & 0xFFFF
            self.emit(f'\tbyte {cksum & 0xFF}')
            self.emit(f'\tbyte {(cksum >> 8) & 0xFF}')
            # Pad to 0x0010 (16-byte header)
            for _ in range(14):
                self.emit('\tbyte 0')
            # Data arrays
            for addr, data in self.eeprom_data:
                self.emit(f'; EEPROM data at 0x{addr:04X} ({len(data)} bytes)')
                for b in data:
                    self.emit(f'\tbyte {b}')
            self.emit('\tsection code')

        # Phase 2: static stack-depth analysis against the page-2 stack
        # reservation. Reports observed depth to stderr; warns (but does
        # not fail) if the budget is exceeded so the gate is informational
        # while we collect data on the corpus. Phase 4 may promote this
        # to a hard error before re-enabling page-2 overlay storage.
        self._analyze_stack_depth()

        # Phase 5: post-self-copy page-3 layout report. Confirms what
        # categories of bytes live in page 3 after the kernel has been
        # copied to page 0, so we can see how much of page 3 is truly
        # "overlay storage" vs static kernel-state residue.
        self._report_page3_post_init_layout()

        return '\n'.join(self.code)

    def _eliminate_dead_functions(self):
        """Remove function bodies that are never referenced."""
        # Find all call/jump targets in emitted code
        refs = set()
        for line in self.code:
            s = line.strip()
            for prefix in ('jal ', 'j ', 'ocall '):
                if s.startswith(prefix):
                    target = s.split()[1]
                    refs.add(target)
        # Keep __tone_setup alive if notes exist (called by __play_note at runtime)
        if hasattr(self, '_note_table') and self._note_table:
            refs.add('__tone_setup')
        # Keep I2C helpers alive when i2c_init() was called — they may be
        # needed for EEPROM preload (generated later in _overlay_partition)
        if getattr(self, '_i2c_init_called', False):
            refs.add('__i2c_sb')
            refs.add('__i2c_rb')
        # In --eeprom mode, the inline preload calls __i2c_sb (the reference
        # is generated later in _overlay_partition and isn't visible here).
        if getattr(self, 'eeprom_mode', False):
            refs.add('__i2c_sb')
        # __lcd_send_raw is reached via fall-through from __lcd_cmd (and
        # via `j __lcd_send_raw` from __lcd_chr) — keep alive when either
        # is referenced. The 2026-04-24 LCD driver rewrite renamed the
        # helper from `__lcd_send` to `__lcd_send_raw` but this dead-
        # function-elimination guard wasn't updated, so the eliminator
        # silently stripped the helper out of the final asm whenever
        # __lcd_cmd / __lcd_chr lacked a direct call site (e.g. when
        # the call goes through a fall-through path the scanner doesn't
        # see). Symptom: lcd_cmd / lcd_char hung indefinitely on
        # hardware because the resident __lcd_cmd body fell through into
        # whatever code happened to be emitted next (typically `_main`,
        # which then reset the stack pointer and recursed). Surfaced by
        # rgb_lcd_smoke when expanded with lcd_cmd(0x80); printf("MK1").
        if '__lcd_cmd' in refs or '__lcd_chr' in refs:
            refs.add('__lcd_send_raw')

        # Identify function body ranges (global label to next global label)
        funcs = []  # (start, end, name)
        i = 0
        while i < len(self.code):
            line = self.code[i]
            s = line.strip()
            if s.endswith(':') and not s.startswith('.') and s.startswith('_'):
                name = s[:-1]  # remove colon
                start = i
                i += 1
                while i < len(self.code):
                    ns = self.code[i].strip()
                    if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                        break
                    i += 1
                funcs.append((start, i, name))
            else:
                i += 1

        if not funcs:
            return

        # Rebuild code keeping only referenced functions and non-function code
        dead_ranges = set()
        for start, end, name in funcs:
            if name not in refs and name != '_main':
                dead_ranges.add((start, end))

        if not dead_ranges:
            return

        new_code = []
        i = 0
        while i < len(self.code):
            skip = False
            for start, end in dead_ranges:
                if start <= i < end:
                    i = end
                    skip = True
                    break
            if not skip:
                new_code.append(self.code[i])
                i += 1

        self.code = new_code
        # After dead-code removal, fold duplicate function bodies.
        self._dedupe_identical_functions()

    def _dedupe_identical_functions(self):
        """Merge functions with identical bodies. Redirects callers of the
        duplicate to the canonical function and removes the duplicate body.
        Label-renaming inside the body is normalized before comparison so
        two functions that only differ in label names are still folded.
        """
        # Locate global functions
        funcs = []  # (name, start, end)
        i = 0
        while i < len(self.code):
            s = self.code[i].strip()
            if s.endswith(':') and not s.startswith('.') and s.startswith('_'):
                name = s[:-1]
                start = i
                i += 1
                while i < len(self.code):
                    ns = self.code[i].strip()
                    if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                        break
                    i += 1
                funcs.append((name, start, i))
            else:
                i += 1

        def normalize_body(lines):
            """Strip comments and rename internal .labels to ".L0", ".L1", ... so
            two functions with identical instructions but different label suffixes
            (common across codegen) compare equal.
            """
            # Collect local labels in order of appearance
            label_map = {}
            counter = [0]
            def map_label(lbl):
                if lbl not in label_map:
                    label_map[lbl] = f'.L{counter[0]}'
                    counter[0] += 1
                return label_map[lbl]
            out = []
            for line in lines:
                s = line.strip()
                if not s or s.startswith(';'):
                    continue
                # Local label definition
                if s.endswith(':') and s.startswith('.'):
                    out.append(map_label(s[:-1]) + ':')
                    continue
                # Global label definition — skip (we compare body only)
                if s.endswith(':'):
                    continue
                # Rewrite any .local label reference in the instruction
                parts = s.split()
                for pi, p in enumerate(parts):
                    if p.startswith('.'):
                        parts[pi] = map_label(p)
                out.append(' '.join(parts))
            return tuple(out)

        # Skip entry points and helpers reached by fall-through (not just jal).
        # __lcd_send_raw is reached via fall-through from __lcd_cmd (and via
        # an explicit `j __lcd_send_raw` from __lcd_chr), so merging its
        # body with another function would break the fall-through linkage.
        # NB: stale name `__lcd_send` (pre-2026-04-24 driver rewrite) was
        # left here too; both have been seen referenced in the codebase
        # while the active emission target is `__lcd_send_raw`.
        SKIP = {'_main', '_overlay_load', '_overlay_load_p1',
                '__lcd_send_raw'}
        body_by_sig = {}  # normalized body → canonical name
        rename_map = {}   # duplicate name → canonical name
        for name, start, end in funcs:
            if name in SKIP:
                continue
            body = self.code[start:end]
            sig = normalize_body(body)
            if not sig:
                continue
            if sig in body_by_sig:
                rename_map[name] = body_by_sig[sig]
            else:
                body_by_sig[sig] = name

        if not rename_map:
            return

        import sys
        n_merged = len(rename_map)
        total_saved = 0
        for dup, canonical in rename_map.items():
            # find dup's body range to sum size
            for name, start, end in funcs:
                if name == dup:
                    total_saved += (end - start)
                    break
        if n_merged:
            print(f"  Dedup: merged {n_merged} function(s), "
                  f"saved ~{total_saved} lines "
                  f"({', '.join(f'{d}→{c}' for d, c in sorted(rename_map.items()))})",
                  file=sys.stderr)

        # Drop bodies of duplicate functions from self.code
        drop_ranges = []
        for name, start, end in funcs:
            if name in rename_map:
                drop_ranges.append((start, end))
        drop_ranges.sort(reverse=True)
        new_code = list(self.code)
        for start, end in drop_ranges:
            del new_code[start:end]
        self.code = new_code

        # Redirect all jal/j references from duplicate → canonical
        for i, line in enumerate(self.code):
            s = line.strip()
            for prefix in ('jal ', 'j ', 'ocall '):
                if s.startswith(prefix):
                    target = s.split()[1]
                    if target in rename_map:
                        self.code[i] = line.replace(target, rename_map[target])
                    break

    @staticmethod
    def _chunk_funcs(funcs, max_size):
        """Split a list of (name, start, end, size) into groups that fit max_size."""
        chunks = []
        current = []
        current_size = 0
        for f in funcs:
            fsize = f[3]
            if current and current_size + fsize > max_size:
                chunks.append((current, current_size))
                current = []
                current_size = 0
            current.append(f)
            current_size += fsize
        if current:
            chunks.append((current, current_size))
        return chunks

    def _analyze_stack_depth(self):
        """Static analysis of maximum stack depth from `_main`.

        Walks the final assembled code, partitions into functions by
        global label, computes per-function local push depth, builds
        the call graph from `jal` references, and recursively combines.
        Counts each `jal` as +1 (return address) plus the called
        function's total depth. Linear-flow analysis: branches that
        skip over pushes can in principle yield underestimates, but
        the corpus's tone-table loops and overlay dispatchers are
        balanced (push/pop pairs within the same basic block) so the
        approximation is tight in practice.

        Reports observed depth to stderr. WARNs if the budget defined
        by P2_STACK_BUDGET (Phase 2 partition) is exceeded. Phase 2
        does not hard-fail on excess; Phase 4 may promote this to an
        error before re-enabling page-2 overlay storage.

        Indirect calls (`jal_r`) are conservative: assume 0 callee
        depth and emit a note. The corpus uses `jal_r` only for
        compile-time-resolved targets in __overlay_load, where the
        target's depth would already be counted via direct refs.
        """
        PUSH_OPS  = {'push', 'push_b', 'push_imm'}
        POP_OPS   = {'pop',  'pop_b'}

        # Partition into per-function bodies by global label.
        functions = {}      # name -> list of stripped lines
        current   = None
        for line in self.code:
            s = line.strip()
            if not s or s.startswith(';'):
                continue
            # Strip inline comments & directives that aren't analysis-relevant.
            if s.startswith('section ') or s.startswith('org '):
                continue
            if s.endswith(':') and not s.startswith('.'):
                current = s[:-1]
                functions[current] = []
                continue
            if current is not None:
                functions[current].append(s)

        # Per-function local analysis: local_max (peak P during body
        # ignoring callee bodies) and call_sites (P at each jal).
        local_max    = {}
        call_sites   = {}   # name -> [(P_at_jal, callee), ...]
        has_jal_r    = set()
        for name, body in functions.items():
            P = 0
            peak = 0
            calls = []
            for ln in body:
                tokens = ln.split(None, 1)
                op = tokens[0]
                arg = tokens[1].split(';', 1)[0].strip() if len(tokens) > 1 else ''
                if op in PUSH_OPS:
                    P += 1
                    peak = max(peak, P)
                elif op in POP_OPS:
                    P -= 1
                elif op == 'jal':
                    target = arg.split(',')[0].split()[0] if arg else ''
                    if target and not target.startswith('.'):
                        calls.append((P, target))
                    P += 1                    # return-addr push
                    peak = max(peak, P)
                    P -= 1                    # callee's ret undoes it
                elif op == 'jal_r':
                    has_jal_r.add(name)
                    P += 1
                    peak = max(peak, P)
                    P -= 1
                # Other ops (j, jnz, jz, ret, …) leave P unchanged for
                # this analysis (ret marks end-of-path but we keep
                # scanning to find max across paths).
            local_max[name]  = peak
            call_sites[name] = calls

        # Recursive total depth memoised on function name.
        memo = {}
        def total(fn, visited):
            if fn in memo:
                return memo[fn]
            if fn in visited:
                # Recursion not supported by the runtime — flag and bail.
                sys.stderr.write(f'  WARNING: recursive call cycle through {fn}\n')
                return 0
            visited = visited | {fn}
            if fn not in functions:
                # External / inline-asm helper — assume 0 (its pushes
                # would have been counted at its definition site if
                # present in self.code).
                return 0
            own  = local_max[fn]
            best = 0
            for P_before, callee in call_sites[fn]:
                # peak when entering callee = P_before + 1 (jal push)
                #                           + total(callee)
                d = P_before + 1 + total(callee, visited)
                best = max(best, d)
            memo[fn] = max(own, best)
            return memo[fn]

        if '_main' not in functions:
            return  # no main (e.g., regression-test snippet); nothing to gate.

        depth = total('_main', set())
        msg = f'stack-depth: max={depth}B from _main (budget={P2_STACK_BUDGET}B)'
        if has_jal_r:
            msg += f' [jal_r in {",".join(sorted(has_jal_r))} — depth may be underestimated]'
        sys.stderr.write(msg + '\n')
        if depth > P2_STACK_BUDGET:
            sys.stderr.write(
                f'  WARNING: stack depth exceeds page-2 reservation '
                f'({depth}B > {P2_STACK_BUDGET}B). Page-2 overlay '
                f'storage cannot be safely activated for this program '
                f'until the stack is bounded below {P2_STACK_LO:#x}.\n')

    def _report_page3_post_init_layout(self):
        """Phase 5: report what occupies page 3 after `__selfcopy`.

        After the bootloader copies page3[0..OVERLAY_REGION-1] into page 0,
        page 3 still holds:
          - Original kernel image at [0..KERNEL_SIZE-1] (transient: overwritten
            by overlay storage as overlays load on top of it).
          - Shared helpers at [KERNEL_SIZE..OVERLAY_REGION-1] (NOT overlaid;
            these survive and are used at runtime via dual-residency).
          - Manifest + pages array at [OVERLAY_REGION..OVERLAY_REGION+meta-1]
            (read every overlay load; persists indefinitely).
          - Overlay storage at [OVERLAY_REGION+meta..] (the actual reason for
            page 3 being overlay-tier in the first place).

        Goal of the larger refactor: page 3 should ideally be 100% overlay
        storage post-init. Phase 5 (deferred) will move the manifest+pages
        out, leaving only kernel-image-residue + shared helpers + overlays.
        Today's report tells us how much each category contributes.
        """
        kernel_size  = getattr(self, '_overlay_kernel_size', None)
        overlay_reg  = getattr(self, '_overlay_region', None)
        if kernel_size is None or overlay_reg is None:
            return  # flat mode (no overlays / no self-copy) — nothing to report.
        meta_size = 0
        if hasattr(self, '_overlay_meta_size'):
            meta_size = self._overlay_meta_size
        shared_size = max(0, overlay_reg - kernel_size)
        sys.stderr.write(
            f'page-3 layout: kernel-image={kernel_size}B (transient), '
            f'shared-helpers={shared_size}B, manifest+pages={meta_size}B, '
            f'overlay-storage at {overlay_reg + meta_size}+\n')

    def _classify_helpers(self):
        """Classify helpers into resident vs overlay-eligible categories.
        Sets self._needs_runtime_i2c and self._dynamic_no_overlay."""
        helpers = getattr(self, '_lcd_helpers', set())

        # Runtime I2C: needed if program uses LCD output, RGB backlight,
        # or EEPROM reads at runtime (not just init-time operations like
        # delay_cal or lcd_init).
        # __lcd_rgb (added when the driver was rewritten for AiP31068L +
        # PCA9633) drives the backlight via runtime I2C transactions and
        # therefore needs the same `__i2c_st_only` / `__i2c_sb` / `__i2c_sp`
        # primitives kept resident as the char/cmd path does.
        runtime_i2c_markers = {'__lcd_chr', '__lcd_cmd', '__lcd_send_raw', '__lcd_print',
                               '__lcd_rgb',
                               '__eeprom_r2c_loop', '__eeprom_dispatch', '__eeprom_load'}
        self._needs_runtime_i2c = bool(helpers & runtime_i2c_markers)

        # Build dynamic _NO_OVERLAY set
        # Always resident: main and overlay loader
        no_ov = {'_main:', '_overlay_load:', '_overlay_load_p1:'}

        # LCD helpers: must be resident because overlays can't call other overlays
        # (the caller gets overwritten when the callee overlay loads).
        if self._needs_runtime_i2c:
            for h in ('__lcd_chr', '__lcd_cmd', '__lcd_print'):
                if h in helpers:
                    no_ov.add(f'{h}:')
                    # __lcd_send_raw (renamed from __lcd_send during the
                    # 2026-04-24 LCD driver rewrite) is reached via fall-
                    # through from __lcd_cmd and an explicit jump from
                    # __lcd_chr; if the partitioner treated it as
                    # overlay-eligible the call site would jump into
                    # whatever overlay happens to be loaded.
                    no_ov.add('__lcd_send_raw:')

        # I2C byte-level helpers: resident if called from runtime code.
        # __i2c_sb and __i2c_rb are called from overlays + resident LCD code.
        # __i2c_st is init-only: runtime LCD sends use __i2c_st_only.
        # __i2c_sp must stay resident when runtime LCD output is present:
        # __lcd_send_raw calls it after each command/character transaction.
        if '__i2c_sb' in helpers:
            no_ov.add('__i2c_sb:')
        if '__i2c_rb' in helpers:
            no_ov.add('__i2c_rb:')
        if '__i2c_rs' in helpers:
            no_ov.add('__i2c_rs:')
        if '__i2c_st_only' in helpers and self._needs_runtime_i2c:
            no_ov.add('__i2c_st_only:')
        if '__i2c_sp' in helpers and self._needs_runtime_i2c:
            no_ov.add('__i2c_sp:')
        # __eeprom_rd: resident only if user code accesses eeprom arrays at
        # runtime. In --eeprom overlay mode it's called only by the init-time
        # preload (before self-copy), so it stays init-only.
        if '__eeprom_rd' in helpers:
            if getattr(self, '_needs_runtime_eeprom_rd', False):
                no_ov.add('__eeprom_rd:')
            # else: init-only, handled by init extraction (stage 1 code)
        if '__i2c_stream' in helpers:
            no_ov.add('__i2c_stream:')

        # Tone runtime helpers must be resident if used — they're called via jal
        # from every overlay that plays notes. Inlining would bloat each overlay.
        # NOTE: __tone_setup is NOT included here — it's init-only (precomputes
        # note table during init). play_note reads precomputed half_period directly.
        tone_runtime = {'__tone', '__play_note', '__delay_Nms'}
        if helpers & tone_runtime:
            for h in tone_runtime:
                if h in helpers:
                    no_ov.add(f'{h}:')

        self._dynamic_no_overlay = no_ov

        # Conditionally resident: helpers that are marked _NO_OVERLAY now but
        # might be bundled into overlays later (step 8b) if they're never
        # called from non-helper resident code (i.e., _main or loader).
        # Helpers in _NO_OVERLAY are conditionally resident — the knapsack
        # + deferred classification (step 8b) decides which to keep resident vs
        # bundle. Exception: helpers needed for I2C infrastructure (EEPROM preload)
        # are unconditionally resident when i2c_init() was called.
        self._conditionally_resident = set()
        unconditional = set()
        if getattr(self, '_i2c_init_called', False) and not self._needs_runtime_i2c:
            # i2c_init() called without LCD → __i2c_sb needed for EEPROM preload.
            unconditional.add('__i2c_sb')
        # In --eeprom mode, the init-time preload runs BEFORE self-copy (so
        # __eeprom_rd can live in stage 1 init code without surviving the
        # self-copy). We still need __i2c_sb kept alive for the preload chain;
        # __i2c_rb and __eeprom_rd are init-only (emitted in stage 1).
        if getattr(self, 'eeprom_mode', False):
            unconditional.add('__i2c_sb')
        # __lcd_send_raw's residency is handled by step 8b propagation
        # (scans resident __lcd_cmd body for 'j __lcd_send_raw' reference)
        for label in no_ov:
            name = label.rstrip(':')
            if name.startswith('__') and name not in unconditional:
                self._conditionally_resident.add(name)

    def _emit_i2c_helpers(self):
        """Emit I2C/LCD helper subroutines if any lcd_cmd/lcd_char builtins were used.

        DEVICE HELPER CONTRACT (mirrors Arduino Wire / MicroPython machine.I2C):
          - ENTRY: bus idle (DDRB=0x00 — both SDA and SCL floating HIGH via pullups)
          - EXIT:  bus idle (DDRB=0x00 via __i2c_sp or merged STOP)
          - Within a multi-phase transaction (e.g. DS3231 register read,
            EEPROM random read: write-address-then-read-data), use
            __i2c_rs (repeated START) between phases. Do NOT emit STOP +
            new START — that releases the bus momentarily, is slower, and
            lets a confused slave (or SQW-coupled SDA) glitch the edge.
          - Every device builtin (rtc_read_*, eeprom_read_byte, lcd_*) is
            self-contained: it assumes bus is idle on entry, returns it
            to idle on exit, and owns nothing persistent between calls.
            This lets the caller interleave calls to different devices
            (e.g. rtc_read_temp() then lcd_char()) without per-device
            cleanup — the protocol IS the cleanup.

        Adding a new I2C device builtin? Follow the contract. Use
        __i2c_rs for any multi-phase transaction the device supports.
        """
        helpers = getattr(self, '_lcd_helpers', set())
        if not helpers:
            return

        # Always emit __i2c_sb (send byte, B=byte, D=counter) — shared by all helpers
        # Optimized: uses ddrb_imm (bus-safe, preserves A), merged tst+shift,
        # zero NOPs, no push/pop in ACK clock. Uses D as counter (was C) to
        # save 1B on entry (no need to route 8 via A).
        # Byte in $d, counter in $b. decb (flashed) lets the loop tail do
        # B-- without round-tripping through A, saving 2B over the old
        # mov/dec/mov pattern. ddrb_imm preserves $a across both branches,
        # so no redundant reload at isbn needed.
        self.emit('__i2c_sb:')
        self.emit('\tmov $a,$d')         # D = byte (survives ldi's A clobber)
        self.emit('\tldi $b,8')          # B = counter
        lbl_s = self.label('isb')
        lbl_h = self.label('isbh')
        lbl_n = self.label('isbn')
        self.emit(f'{lbl_s}:')
        self.emit('\tmov $d,$a')         # A = byte (reload for test)
        self.emit('\ttst 0x80')
        self.emit(f'\tjnz {lbl_h}')
        self.emit_ddrb(0x03)
        self.emit_ddrb(0x01)
        self.emit_ddrb(0x03)
        self.emit(f'\tj {lbl_n}')
        self.emit(f'{lbl_h}:')
        self.emit_ddrb(0x02)
        self.emit_ddrb(0x00)
        self.emit_ddrb(0x02)
        self.emit(f'{lbl_n}:')
        self.emit('\tsll')               # A = byte << 1
        self.emit('\tmov $a,$d')         # D = shifted byte for next iter
        self.emit('\tdecb')              # B--, sets Z
        self.emit(f'\tjnz {lbl_s}')
        # ACK clock: no NOPs needed (ddrb_imm has built-in settling),
        # no push/pop needed (ddrb_imm preserves A)
        self.emit_ddrb(0x02)   # SCL LOW, SDA released
        self.emit_ddrb(0x00)   # SCL HIGH (read ACK)
        self.emit('\texrw 0')          # A = port B (bit 0 = ACK)
        self.emit_ddrb(0x02)   # SCL LOW
        self.emit('\tret')

        # Compact I2C bus scan helpers generated from C scan loops.
        # Contract: C=count, D=addr. Stores ACKed addresses at data[base+n],
        # then writes a zero sentinel. Uses __i2c_sb's ACK sample in A.
        for scan_name, (base, start, end, cap) in sorted(
                getattr(self, '_i2c_scan_helpers', {}).items()):
            if scan_name not in helpers:
                continue
            lbl_loop = self.label('iscan')
            lbl_skip = self.label('iscan_skip')
            lbl_done = self.label('iscan_done')
            self.emit(f'{scan_name}:')
            self.emit('\tldi $c,0')           # result count
            self.emit(f'\tldi $d,{start}')    # current address
            self.emit(f'{lbl_loop}:')
            self.emit('\tmov $d,$a')
            self.emit(f'\tcmpi {end}')
            self.emit(f'\tjc {lbl_done}')     # addr >= end
            self.emit('\texrw 2')
            self.emit_ddrb(0x01)              # START
            self.emit_ddrb(0x03)
            self.emit('\tpush $d')            # __i2c_sb uses D for byte state
            self.emit('\tmov $d,$a')
            self.emit('\tsll')                # 7-bit addr -> write address
            self.emit('\tjal __i2c_sb')       # A = ACK sample
            self.emit('\tmov $a,$b')          # save ACK while restoring addr
            self.emit('\tpop $d')
            self.emit('\tmov $b,$a')          # A = ACK sample, D = addr
            self.emit_ddrb(0x03)              # STOP
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x00)
            self.emit('\ttst 0x01')
            self.emit(f'\tjnz {lbl_skip}')    # NACK -> do not store
            self.emit('\tmov $c,$a')
            self.emit(f'\tcmpi {cap}')
            self.emit(f'\tjc {lbl_skip}')      # keep room for sentinel
            self.emit('\tmov $d,$a')          # A = addr
            self.emit('\tmov $a,$b')          # B = addr value
            self.emit('\tmov $c,$a')          # A = result offset
            if base:
                self.emit(f'\taddi {base},$a')
            self.emit('\tideref')             # data[base+C] = addr
            self.emit('\tincc')
            self.emit(f'{lbl_skip}:')
            self.emit('\tincd')
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
            self.emit('\tclr $a')
            self.emit('\tmov $a,$b')          # B = sentinel 0
            self.emit('\tmov $c,$a')
            if base:
                self.emit(f'\taddi {base},$a')
            self.emit('\tideref')             # data[base+C] = 0
            self.emit('\tret')

        # __i2c_st_only: just START (no address)
        if '__i2c_st_only' in helpers:
            self.emit('__i2c_st_only:')
            self.emit('\texrw 2')
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit('\tret')

        # __i2c_st: START + send address 0x4E
        self.emit('__i2c_st:')
        self.emit('\texrw 2')
        self.emit_ddrb(0x01)
        self.emit_ddrb(0x03)
        self.emit('\tldi $a,0x4E')
        self.emit('\tjal __i2c_sb')
        self.emit('\tret')

        # __i2c_sp: STOP
        self.emit('__i2c_sp:')
        self.emit_ddrb(0x03)
        self.emit_ddrb(0x01)
        self.emit_ddrb(0x00)
        self.emit('\tret')

        # __i2c_rs: REPEATED START. Emits the start edge without first
        # issuing a STOP, so the master keeps ownership of the bus across
        # a write-then-read transaction (DS3231 register read, EEPROM
        # random read). Saves ~6B per call site vs. STOP+START.
        #
        # Entry assumption: previous __i2c_sb left DDRB=0x02 (SCL LOW,
        # SDA released) after ACK clock. Protocol requires:
        #   SCL released HIGH while SDA HIGH → then SDA falls while SCL
        #   HIGH → then SCL falls LOW for data bits. The defensive
        #   ddrb_imm 0x02 at entry guarantees the precondition even if
        #   a future path enters from a different state.
        # Goes through emit_ddrb so port shadow claims (buzzer on PB6,
        # etc.) are preserved across the repeated-START edges.
        if '__i2c_rs' in helpers:
            self.emit('__i2c_rs:')
            self.emit('\texrw 2')           # VIA read before DDRB writes
            self.emit_ddrb(0x02)            # ensure SCL LOW, SDA released
            self.emit_ddrb(0x00)            # SCL rises → momentary idle
            self.emit_ddrb(0x01)            # SDA falls with SCL HIGH → REP START
            self.emit_ddrb(0x03)            # SCL falls → ready for address byte
            self.emit('\tret')

        # __i2c_rb: read one I2C byte into D (8 bits MSB first)
        # Optimized: zero NOPs, no push/pop (uses B for SDA bit),
        # SCL dedup (2 ddrb_imm per bit instead of 3)
        if '__i2c_rb' in helpers:
            lbl_rb = self.label('rb')
            lbl_rz = self.label('rz')
            self.emit('__i2c_rb:')
            # Read one I2C byte into D (8 bits MSB first).
            # exrw 0 clobbers A and C. Only B and D are safe.
            # B = byte accumulator, D = counter. Result → D at end.
            self.emit('\tldi $b,0')        # B = accumulated byte
            self.emit('\tldi $d,8')        # D = bit counter
            self.emit(f'{lbl_rb}:')
            self.emit('\tmov $b,$a')       # A = accumulated
            self.emit('\tsll')             # A <<= 1
            self.emit('\tmov $a,$b')       # B = shifted
            self.emit_ddrb(0x00)   # SCL HIGH
            self.emit('\texrw 0')          # A = port B
            self.emit('\ttst 0x01')        # test SDA
            self.emit(f'\tjz {lbl_rz}')
            self.emit('\tmov $b,$a')       # A = accumulated
            self.emit('\tori 0x01,$a')     # set bit 0
            self.emit('\tmov $a,$b')       # B = result
            self.emit(f'{lbl_rz}:')
            self.emit_ddrb(0x02)   # SCL LOW
            self.emit('\tdecd')            # D--, A clobbered (unused here)
            self.emit(f'\tjnz {lbl_rb}')
            self.emit('\tmov $b,$d')       # D = B (result)
            self.emit('\tret')

        # __eeprom_rd: read one byte from AT24C32 EEPROM
        # Entry: B = addr_hi, A = addr_lo. Returns: A = byte read.
        # AT24C32 at I2C address 0x57 (write=0xAE, read=0xAF).
        if '__eeprom_rd' in helpers:
            self.emit('__eeprom_rd:')
            self.emit('\tmov $a,$d')       # D = addr_lo
            self.emit('\tmov $b,$a')       # A = addr_hi (save for later)
            self.emit('\tpush $a')         # save addr_hi
            # START + device addr write
            self.emit('\texrw 2')
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit('\tldi $a,0xAE')
            self.emit('\tjal __i2c_sb')
            # addr high
            self.emit('\tpop $a')
            self.emit('\tjal __i2c_sb')
            # addr low
            self.emit('\tmov $d,$a')
            self.emit('\tjal __i2c_sb')
            # Repeated START (keeps bus owned, saves ~6B vs STOP+START)
            self.emit('\tjal __i2c_rs')
            self.emit('\tldi $a,0xAF')
            self.emit('\tjal __i2c_sb')
            # Read byte via __i2c_rb (D = byte, then A = byte)
            self.emit('\tjal __i2c_rb')
            self.emit('\tmov $d,$a')       # A = read byte
            # NACK + STOP
            self.emit_ddrb(0x00)
            self.emit_ddrb(0x02)
            self.emit_ddrb(0x03)
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x00)
            self.emit('\tret')

        if '__i2c_stream' in helpers:
            # General-purpose I2C stream interpreter.
            # Entry: A = page3 offset of sentinel-encoded byte sequence.
            # Sentinels: 0xFE=START, 0xFD=STOP, 0xFC N=REPEAT 0x00 N times,
            #            0xFB=READ byte (result in C), 0xFF=END, else=send byte.
            # Uses D as pointer, C for read result. ~60 bytes resident.
            lbl_loop = self.label('is_lp')
            lbl_ns = self.label('is_ns')
            lbl_nr = self.label('is_nr')
            lbl_send = self.label('is_sd')
            lbl_adv = self.label('is_av')
            lbl_done = self.label('is_dn')
            lbl_rpt = self.label('is_rp')

            self.emit('__i2c_stream:')
            self.emit('\tmov $a,$d')          # D = offset (pointer)
            self.emit(f'{lbl_loop}:')
            self.emit('\tmov $d,$a')
            self.emit('\tderefp3')            # A = page3[D]
            self.emit('\tldi $b,0xFF')
            self.emit('\tcmp $b')
            self.emit(f'\tjz {lbl_done}')     # END
            self.emit('\tldi $b,0xFE')
            self.emit('\tcmp $b')
            self.emit(f'\tjnz {lbl_ns}')
            # START
            self.emit('\texrw 2')
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_ns}:')
            self.emit('\tldi $b,0xFD')
            self.emit('\tcmp $b')
            self.emit(f'\tjnz {lbl_nr}')
            # STOP
            self.emit_ddrb(0x03)
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x00)
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_nr}:')
            self.emit('\tldi $b,0xFB')
            self.emit('\tcmp $b')
            lbl_nrd = self.label('is_nrd')
            self.emit(f'\tjnz {lbl_nrd}')
            # READ: inline i2c_rb, result in C
            lbl_rdb = self.label('is_rb')
            lbl_rz = self.label('is_rz')
            self.emit('\tpush $d')          # save pointer
            self.emit('\tldi $b,0')
            self.emit('\tldi $d,8')
            self.emit(f'{lbl_rdb}:')
            self.emit('\tmov $b,$a')
            self.emit('\tsll')
            self.emit('\tmov $a,$b')
            self.emit_ddrb(0x00)    # SCL HIGH
            self.emit('\tnop')
            self.emit('\texrw 0')           # read port
            self.emit('\ttst 0x01')
            self.emit(f'\tjz {lbl_rz}')
            self.emit('\tmov $b,$a')
            self.emit('\tori 0x01,$a')
            self.emit('\tmov $a,$b')
            self.emit(f'{lbl_rz}:')
            self.emit_ddrb(0x02)    # SCL LOW
            self.emit('\tmov $d,$a')
            self.emit('\tdec')
            self.emit('\tmov $a,$d')
            self.emit(f'\tjnz {lbl_rdb}')
            self.emit('\tmov $b,$c')        # C = read result
            self.emit('\tpop $d')           # restore pointer
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_nrd}:')
            self.emit('\tldi $b,0xFC')
            self.emit('\tcmp $b')
            self.emit(f'\tjnz {lbl_send}')
            # REPEAT: next byte = count, send 0x00 count times
            self.emit('\tmov $d,$a')
            self.emit('\tinc')
            self.emit('\tmov $a,$d')          # advance pointer
            self.emit('\tmov $d,$a')
            self.emit('\tderefp3')            # A = count
            self.emit('\tmov $a,$c')          # C = count
            self.emit(f'{lbl_rpt}:')
            self.emit('\tclr $a')
            self.emit('\tjal __i2c_sb')       # send 0x00
            self.emit('\tmov $c,$a')
            self.emit('\tdec')
            self.emit('\tmov $a,$c')
            self.emit(f'\tjnz {lbl_rpt}')
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_send}:')
            self.emit('\tjal __i2c_sb')       # send byte
            self.emit(f'{lbl_adv}:')
            self.emit('\tmov $d,$a')
            self.emit('\tinc')
            self.emit('\tmov $a,$d')
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
            self.emit('\tret')

        if '__lcd_init' in helpers:
            # Data-driven LCD init for DFRobot RGB LCD (AiP31068L + PCA9633/V2.0).
            # LCD at 0x3E (0x7C), RGB at 0x2D (0x5A).
            # END was 0xFF but the RGB backlight brightness bytes emit 0xFF
            # as data, which the walker consumed as the END sentinel and
            # exited init mid-transaction. Moved to 0xFB (no legitimate
            # init byte ever uses 0xFB).
            START, STOP, END, DELAY = 0xFE, 0xFD, 0xFB, 0xFC
            LCD = 0x7C
            RGB = 0x5A
            lcd_init_data = [DELAY, DELAY, DELAY, DELAY, DELAY, DELAY, DELAY, DELAY, DELAY, DELAY] # 50ms
            # AiP31068L: Function Set (8-bit), Display ON, Clear, Entry Mode
            for cmd in [0x38, 0x0C, 0x01, 0x06]:
                lcd_init_data += [START, LCD, 0x80, cmd, STOP]
                if cmd == 0x01:
                    lcd_init_data.append(DELAY)  # clear display needs ~2ms
            # Backlight V2.0 @ 0x2D (register map per DFRobot_RGBLCD1602.cpp
            # _RGBAddr == 0x2D branch: R=0x01, G=0x02, B=0x03). Direct PWM
            # register writes; V2.0 doesn't need MODE1/OUTPUT init (the
            # DFRobot driver only does that for the 0x60 PCA9633 variant).
            # Default full-white (0xFF each); lcd_init(r, g, b) user call
            # substitutes compile-time constants here instead.
            _rgb = getattr(self, '_lcd_init_rgb', None) or (0xFF, 0xFF, 0xFF)
            _r, _g, _b = _rgb
            lcd_init_data += [START, RGB, 0x01, _r, STOP]
            lcd_init_data += [START, RGB, 0x02, _g, STOP]
            lcd_init_data += [START, RGB, 0x03, _b, STOP]
            lcd_init_data.append(END)

            data_base = self.data_alloc
            self.data_alloc += len(lcd_init_data)
            self._lcd_init_data = (data_base, lcd_init_data)

            lbl_loop = self.label('li_lp')
            lbl_ns = self.label('li_ns')
            lbl_send = self.label('li_sd')
            lbl_adv = self.label('li_av')
            lbl_done = self.label('li_dn')

            self.emit('__lcd_init:')
            self.emit(f'\tldi $d,{data_base}')
            self.emit(f'{lbl_loop}:')
            self.emit('\tmov $d,$a')
            self.emit('\tderef')
            self.emit(f'\tcmp {END}')
            self.emit(f'\tjz {lbl_done}')
            self.emit('\tpush $d')
            self.emit(f'\tcmp {START}')
            self.emit(f'\tjnz {lbl_ns}')
            self.emit('\tjal __i2c_st_only')
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_ns}:')
            self.emit(f'\tcmp {STOP}')
            lbl_nd = self.label('li_nd')
            self.emit(f'\tjnz {lbl_nd}')
            self.emit('\tjal __i2c_sp')
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_nd}:')
            self.emit(f'\tcmp {DELAY}')
            self.emit(f'\tjnz {lbl_send}')
            if '__delay_Nms' in helpers:
                self.emit('\tldi $b,5')
                self.emit('\tjal __delay_Nms')
                self.emit(f'\tj {lbl_adv}')
            else:
                lbl_outer = self.label('li_do')
                lbl_inner = self.label('li_di')
                self.emit(f'\tldi $c,5')
                self.emit(f'{lbl_outer}:')
                self.emit('\tldi $a,0')
                self.emit(f'{lbl_inner}:')
                self.emit('\tdec')
                self.emit(f'\tjnz {lbl_inner}')
                self.emit('\tmov $c,$a')
                self.emit('\tdec')
                self.emit('\tmov $a,$c')
                self.emit(f'\tjnz {lbl_outer}')
                self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_send}:')
            self.emit('\tjal __i2c_sb')
            self.emit(f'{lbl_adv}:')
            self.emit('\tpop $d')
            self.emit('\tmov $d,$a')
            self.emit('\tinc')
            self.emit('\tmov $a,$d')
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
            self.emit('\tret')

        # Updated __lcd_chr / __lcd_cmd for AiP31068L
        if '__lcd_chr' in helpers:
            self.emit('__lcd_chr:')
            self.emit('\tpush $a')
            self.emit('\tldi $a,0x40')
            self.emit('\tj __lcd_send_raw')
        if '__lcd_cmd' in helpers:
            self.emit('__lcd_cmd:')
            self.emit('\tpush $a')
            self.emit('\tldi $a,0x80')
            # Fall through to __lcd_send_raw
        if '__lcd_send_raw' in helpers:
            self.emit('__lcd_send_raw:')
            self.emit('\tpush $a')         # save prefix
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x7C')     # LCD Address (0x3E << 1)
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $a')          # restore prefix
            self.emit('\tjal __i2c_sb')
            # Stack at this point: [caller_ret, char, <SP>]. char is the
            # byte pushed by __lcd_chr/__lcd_cmd before the j here, so it
            # sits at SP+1. Was `ldsp 2` which reads caller_ret and sent
            # that byte as the LCD data/command — visible garbage.
            self.emit('\tldsp 1')          # A = char/cmd
            self.emit('\tjal __i2c_sb')
            self.emit('\tjal __i2c_sp')
            self.emit('\tpop $a')          # clean stack
            self.emit('\tret')

        if '__lcd_rgb' in helpers:
            self.emit('__lcd_rgb:')
            # A=R, B=G, [SP+2]=B. Uses 3 transactions for safe non-autoincrement write.
            self.emit('\tpush $b')         # save G
            self.emit('\tpush $a')         # save R
            # Red (0x04)
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x5A')     # RGB Address (0x2D << 1)
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x01')     # Red register (V2.0 @ 0x2D)
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $a')          # restore R
            self.emit('\tjal __i2c_sb')
            self.emit('\tjal __i2c_sp')
            # Green (0x03)
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x5A')
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x02')     # Green register (V2.0 @ 0x2D)
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $a')          # restore G
            self.emit('\tjal __i2c_sb')
            self.emit('\tjal __i2c_sp')
            # Blue (0x02)
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x5A')
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x03')     # Blue register (V2.0 @ 0x2D)
            self.emit('\tjal __i2c_sb')
            self.emit('\tldsp 2')          # load B from stack
            self.emit('\tjal __i2c_sb')
            self.emit('\tjal __i2c_sp')
            self.emit('\tret')


        if '__lcd_print' in helpers:
            # Print null-terminated string from page 1. A = page-1
            # offset (resolved from a `__str_N` label by the assembler;
            # the compiler never bakes a numeric offset).
            lbl_lp = self.label('lp_lp')
            lbl_dn = self.label('lp_dn')
            self.emit('__lcd_print:')
            self.emit('\tmov $a,$d')         # D = pointer
            self.emit(f'{lbl_lp}:')
            self.emit('\tmov $d,$a')         # A = pointer
            self.emit('\tpush $a')           # save pointer
            self.emit('\tderef')             # A = data[pointer]
            self.emit('\tcmp 0')             # null terminator?
            self.emit(f'\tjz {lbl_dn}')
            self.emit('\tmov $a,$d')         # D = char (for __lcd_chr)
            self.emit('\tjal __lcd_chr')     # print char (clobbers all)
            self.emit('\tpop $a')            # A = pointer
            self.emit('\taddi 1,$a')         # pointer++
            self.emit('\tmov $a,$d')         # D = pointer
            self.emit(f'\tj {lbl_lp}')
            self.emit(f'{lbl_dn}:')
            self.emit('\tpop $a')            # balance stack
            self.emit('\tret')

        # __print_u8_dec: A = number 0-255, prints decimal with leading-zero
        # suppression. Structure: if num >= 100, print hundreds + force-print
        # tens + print ones. Else-if num >= 10, print tens + ones. Else print
        # ones only. Fall-throughs avoid needing a "seen nonzero" flag.
        # incc clobbers $a so each subtract-loop saves/restores A around it.
        # MK1 carry convention: `cmp A,imm; jnc L` branches when A < imm.
        if '__print_u8_dec' in helpers:
            lbl_lt100 = self.label('pd_lt100')
            lbl_h = self.label('pd_h')
            lbl_hd = self.label('pd_hd')
            lbl_dotens = self.label('pd_dotens')
            lbl_lt10 = self.label('pd_lt10')
            lbl_t = self.label('pd_t')
            lbl_td = self.label('pd_td')
            self.emit('__print_u8_dec:')
            # --- branch on magnitude ---
            self.emit('\tcmpi 100')
            self.emit(f'\tjnc {lbl_lt100}')         # < 100 → skip hundreds
            # hundreds path (counter in $c, save/restore A around incc)
            self.emit('\tldi $c,0')
            self.emit(f'{lbl_h}:')
            self.emit('\tcmpi 100')
            self.emit(f'\tjnc {lbl_hd}')
            self.emit('\tsubi 100,$a')
            self.emit('\tpush $a'); self.emit('\tincc'); self.emit('\tpop $a')
            self.emit(f'\tj {lbl_h}')
            self.emit(f'{lbl_hd}:')
            self.emit('\tpush $a')                  # save remainder
            self.emit('\tmov $c,$a')
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit('\tpop $a')
            self.emit(f'\tj {lbl_dotens}')          # always print tens after hundreds
            self.emit(f'{lbl_lt100}:')
            self.emit('\tcmpi 10')
            self.emit(f'\tjnc {lbl_lt10}')          # < 10 → just print ones
            self.emit(f'{lbl_dotens}:')
            # tens path
            self.emit('\tldi $c,0')
            self.emit(f'{lbl_t}:')
            self.emit('\tcmpi 10')
            self.emit(f'\tjnc {lbl_td}')
            self.emit('\tsubi 10,$a')
            self.emit('\tpush $a'); self.emit('\tincc'); self.emit('\tpop $a')
            self.emit(f'\tj {lbl_t}')
            self.emit(f'{lbl_td}:')
            self.emit('\tpush $a')
            self.emit('\tmov $c,$a')
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit('\tpop $a')
            self.emit(f'{lbl_lt10}:')
            # --- ones ---
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit('\tret')

        # __lcd_temp_u8: A = 0..99-ish temp, print decimal without leading
        # zero followed by HD44780 degree glyph and 'C'. This is deliberately
        # smaller than generic printf("%d\xDFC") and replaces the repeated
        # tens/ones source idiom used by several LCD temperature examples.
        if '__lcd_temp_u8' in helpers:
            lbl_loop = self.label('ltu_loop')
            lbl_done = self.label('ltu_done')
            lbl_no_tens = self.label('ltu_no_tens')
            self.emit('__lcd_temp_u8:')
            self.emit('\tldi $c,0')          # tens
            self.emit(f'{lbl_loop}:')
            self.emit('\tcmpi 10')
            self.emit(f'\tjnc {lbl_done}')   # A < 10
            self.emit('\tsubi 10,$a')        # ones -= 10
            self.emit('\tincc')              # tens++
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
            self.emit('\tpush $a')           # save ones across optional tens char
            self.emit('\tmov $c,$a')
            self.emit('\ttst 0xFF')
            self.emit(f'\tjz {lbl_no_tens}')
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit(f'{lbl_no_tens}:')
            self.emit('\tpop $a')
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit('\tldi $a,223')
            self.emit('\tjal __lcd_chr')
            self.emit('\tldi $a,67')
            self.emit('\tjal __lcd_chr')
            self.emit('\tret')

        # __lcd_bcd_u8: A = packed BCD byte, print high nibble then low nibble.
        # Common for RTC hour/min/sec display where values are already BCD.
        if '__lcd_bcd_u8' in helpers:
            self.emit('__lcd_bcd_u8:')
            self.emit('\tpush $a')
            self.emit('\tslr'); self.emit('\tslr')
            self.emit('\tslr'); self.emit('\tslr')
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit('\tpop $a')
            self.emit('\tandi 0x0F,$a')
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit('\tret')

        # Compact zero-terminated byte-list printer. A returns the number of
        # bytes printed, so callers can feed it directly to out().
        for hex_results_name, (base, limit) in sorted(
                getattr(self, '_lcd_hex_result_helpers', {}).items()):
            if hex_results_name not in helpers:
                continue
            lbl_loop = self.label('lhr_loop')
            lbl_done = self.label('lhr_done')
            self.emit(f'{hex_results_name}:')
            self.emit('\tldi $c,0')          # count
            self.emit(f'{lbl_loop}:')
            self.emit('\tmov $c,$a')
            self.emit(f'\tcmpi {limit}')
            self.emit(f'\tjc {lbl_done}')
            self.emit('\tmov $c,$a')
            if base:
                self.emit(f'\taddi {base},$a')
            self.emit('\tderef')
            self.emit('\ttst 0xFF')
            self.emit(f'\tjz {lbl_done}')
            self.emit('\tpush $a')
            self.emit('\tldi $a,32')
            self.emit('\tjal __lcd_chr')
            self.emit('\tpop $a')
            self.emit('\tjal __print_u8_hex')
            self.emit('\tincc')
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
            self.emit('\tmov $c,$a')
            self.emit('\tret')

        # __print_u8_hex: A = byte, prints 2 hex digits via __lcd_chr.
        # `jnc` = branch when digit < 10 (skip the +7 for 'A'-'F' offset).
        if '__print_u8_hex' in helpers:
            lbl_hd1 = self.label('px_hd1')
            lbl_hd2 = self.label('px_hd2')
            self.emit('__print_u8_hex:')
            self.emit('\tpush $a')                  # save low nibble for later
            self.emit('\tslr'); self.emit('\tslr')
            self.emit('\tslr'); self.emit('\tslr')  # A = high nibble
            self.emit('\tcmpi 10')
            self.emit(f'\tjnc {lbl_hd1}')           # digit < 10 → plain +48
            self.emit('\taddi 7,$a')                # 'A' - '0' - 10
            self.emit(f'{lbl_hd1}:')
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit('\tpop $a')                   # restore byte
            self.emit('\tandi 0x0F,$a')
            self.emit('\tcmpi 10')
            self.emit(f'\tjnc {lbl_hd2}')
            self.emit('\taddi 7,$a')
            self.emit(f'{lbl_hd2}:')
            self.emit('\taddi 48,$a')
            self.emit('\tjal __lcd_chr')
            self.emit('\tret')

        # __delay_cal: calibrate against DS3231 1 Hz SQW.
        #
        # Stores `blocks` = number of 256-iter (dec+jnz) blocks that fit
        # in the 500 ms HIGH phase, i.e. blocks ≈ F/4608 at clock rate F.
        # This raw value is consumed by TWO different runtime helpers
        # that each apply their own scaling:
        #
        #   * __delay_Nms: shifts right by 1 (blocks/2 ≈ iters_per_ms)
        #     and subtracts one iter to absorb its own per-ms outer
        #     overhead (~14 cycles = ~1.5 bare iters). Yields an
        #     effective delay(N) ≈ N ms within ~2% at 166 kHz.
        #
        #   * __tone_setup: uses the raw blocks value in a multiply-
        #     then-shift pipeline tuned empirically to produce 4096/ratio
        #     Hz tones (ratio = round(4000/freq)). That arithmetic was
        #     designed when blocks was stored directly as "ipm", so
        #     storing anything else here would silently double tone
        #     frequencies and halve their durations — scaling instead
        #     lives in __delay_Nms so __tone_setup stays untouched.
        if '__delay_cal' in helpers:
            lbl_s1 = self.label('ds1')
            lbl_s2 = self.label('ds2')
            lbl_cal = self.label('dcal')
            lbl_chi = self.label('dchi')
            lbl_done = self.label('dcdn')
            self.emit('__delay_cal:')
            # Configure DS3231 SQW = 1Hz
            self.emit('\texrw 2')
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit('\tldi $a,0xD0')
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x0E')
            self.emit('\tjal __i2c_sb')
            self.emit('\tclr $a')
            self.emit('\tjal __i2c_sb')
            # Inline I2C STOP (avoids __i2c_sp dependency for overlay-eligible delay_cal)
            self.emit_ddrb(0x03)   # both LOW
            self.emit_ddrb(0x01)   # SDA LOW, SCL HIGH
            self.emit_ddrb(0x00)   # both HIGH (idle)
            # Clear DDRA so PA0 reads SQW. Routed through emit_ddra so any
            # active port-A claim keeps its bits driven.
            self.emit_ddra(0x00)
            # Sync: wait LOW then HIGH (rising edge)
            self.emit(f'{lbl_s1}:')
            self.emit('\texrw 1')
            self.emit('\ttst 0x01')
            self.emit(f'\tjz {lbl_s2}')
            self.emit(f'\tj {lbl_s1}')
            self.emit(f'{lbl_s2}:')
            self.emit('\texrw 1')
            self.emit('\ttst 0x01')
            self.emit(f'\tjnz {lbl_cal}')
            self.emit(f'\tj {lbl_s2}')
            # Calibrate: count bare (dec+jnz) iterations during SQW HIGH
            # phase (500 ms). A loops 256→0 (bare dec+jnz). B counts
            # A-wraps (256-iter blocks). On exit, A holds B (raw block
            # count) for both __delay_Nms and __tone_setup to consume.
            self.emit(f'{lbl_cal}:')
            self.emit('\tldi $b,0')
            self.emit('\tclr $a')
            self.emit(f'{lbl_chi}:')
            self.emit('\tdec')
            self.emit(f'\tjnz {lbl_chi}')
            self.emit('\tmov $b,$a')
            self.emit('\tinc')
            self.emit('\tmov $a,$b')
            self.emit('\texrw 1')
            self.emit('\ttst 0x01')
            self.emit(f'\tjz {lbl_done}')
            self.emit('\tclr $a')
            self.emit(f'\tj {lbl_chi}')
            self.emit(f'{lbl_done}:')
            self.emit('\tmov $b,$a')       # A = B (blocks during HIGH)
            self.emit_kstate_store('calib_blocks')
            # SQW left enabled — no coupling issue, saves ~20B
            self.emit('\tret')

        # __delay_Nms: B = delay count (ms). Reads ipm from page3[240].
        # Runs B outer iterations × ipm inner (bare dec+jnz) iterations.
        # Same bare inner loop as the calibrator — timing is accurate
        # to within the outer-loop overhead (3 insns per ms, ~0.6% at
        # 500kHz). Matches the calibrator's own outer overhead, so net
        # error stays inside the existing ±2% tolerance.
        #
        # 12B vs old 34B multiply+countdown approach. Same semantics
        # (delay(N) ms for N=1..255, monotonically increasing), better
        # byte cost.
        if '__delay_Nms' in helpers:
            lbl_outer = self.label('dms_o')
            lbl_inner = self.label('dms_i')
            self.emit('__delay_Nms:')
            self.emit_kstate_load('calib_blocks')
            # Scale: blocks is F/4608 (per 500 ms HIGH). We want the
            # inner loop's iters per ms to satisfy 9 × iters + 14 ≈ F/1000
            # where 14 is the outer-loop overhead per ms. That solves to
            # iters ≈ F/9000 - 1.56 ≈ blocks/2 - 1. Implemented as
            # `slr; dec`.
            self.emit('\tslr')             # A = blocks / 2 (≈ ipm)
            self.emit('\tdec')             # A -= 1 (compensate outer overhead)
            self.emit('\tmov $a,$c')       # C = effective ipm
            self.emit(f'{lbl_outer}:')
            self.emit('\tmov $c,$a')       # A = ipm (reload per ms)
            self.emit(f'{lbl_inner}:')
            self.emit('\tdec')
            self.emit(f'\tjnz {lbl_inner}')
            self.emit('\tdecb')            # B--
            self.emit(f'\tjnz {lbl_outer}')
            self.emit('\tret')

        # __tone_setup: B = ratio → C = half_period = (ipm × ratio) >> 4.
        # Reads ipm from page3[240]. Clobbers A, B, D.
        # push $a / pop $a ARE safe with jal — verified from microcode:
        # stor reg,[$sp] has SD (push), load reg,[$sp] has SU (pop).
        # Nested jal/ret/push/pop all manage SP correctly.
        if '__tone_setup' in helpers:
            lbl_mul = self.label('tsmul')
            lbl_noc = self.label('tsnoc')
            self.emit('__tone_setup:')
            self.emit_kstate_load('calib_blocks')
            self.emit('\tmov $a,$c')       # C = ipm
            self.emit('\tldi $d,0')        # D = product high
            self.emit('\tclr $a')          # A = product low
            # Multiply: D:A += C, B times
            self.emit(f'{lbl_mul}:')
            self.emit('\tadd $c,$a')       # A += C
            self.emit(f'\tjnc {lbl_noc}')
            self.emit('\tpush $a')
            self.emit('\tmov $d,$a')
            self.emit('\tinc')
            self.emit('\tmov $a,$d')
            self.emit('\tpop $a')
            self.emit(f'{lbl_noc}:')
            self.emit('\tpush $a')         # save product low
            self.emit('\tmov $b,$a')       # A = B (counter)
            self.emit('\tdec')
            self.emit('\tmov $a,$b')       # B = counter-1
            self.emit('\tpop $a')          # A = product low
            self.emit(f'\tjnz {lbl_mul}')
            # D:A = ipm × ratio. Shift >> 4.
            self.emit('\tslr')
            self.emit('\tslr')
            self.emit('\tslr')
            self.emit('\tslr')             # A = A >> 4
            self.emit('\tmov $a,$c')       # C = A >> 4 (temp)
            self.emit('\tmov $d,$a')       # A = D
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tsll')             # A = D << 4
            self.emit('\tadd $c,$a')       # A = (D<<4) + (A>>4) = (blocks*ratio)/16
            # Subtract 2 to compensate for the per-tone-cycle overhead that
            # the `(blocks × ratio) / 16` formula ignores. With the duty-cycle
            # pad (3 nop + 1 tst, 14 cyc) added below, the per-period
            # overhead is 52 cycles ≈ 2.89 inner-loop iters; rounded with
            # the same +1 empirical bias the original `-1` carried over its
            # analytical -2.11 (overhead 38 pre-pad), the right value is -2.
            self.emit('\tdec')
            self.emit('\tdec')
            self.emit('\tmov $a,$c')       # C = half_period
            self.emit('\tret')

        # __tone: play square wave on PA1.
        # C = half_period (dec+jnz iterations per half-cycle).
        # D:B = 16-bit cycle count (D=high, B=low).
        # DDRA PA1 must already be configured as output by runtime main.
        if '__tone' in helpers:
            lbl_loop = self.label('ttl')
            lbl_hi = self.label('thi')
            lbl_lo = self.label('tlo')
            lbl_done = self.label('tdn')
            self.emit('__tone:')
            self.emit(f'{lbl_loop}:')
            self.emit('\tldi $a,0x02')
            self.emit('\texw 0 1')         # PA1 HIGH
            self.emit('\tmov $c,$a')       # A = half_period
            self.emit(f'{lbl_hi}:')
            self.emit('\tdec')
            self.emit(f'\tjnz {lbl_hi}')
            # Duty-cycle pad: 14 cycles (3 nop + tst 0) to balance the
            # LOW-side outer-tail overhead. Constant — independent of
            # MK1 clock and target frequency. None of these clobber $a.
            # __tone_setup's compensation absorbs the period growth.
            self.emit('\tnop')
            self.emit('\tnop')
            self.emit('\tnop')
            self.emit('\ttst 0')
            self.emit('\tclr $a')
            self.emit('\texw 0 1')         # PA1 LOW
            self.emit('\tmov $c,$a')
            self.emit(f'{lbl_lo}:')
            self.emit('\tdec')
            self.emit(f'\tjnz {lbl_lo}')
            # 16-bit decrement D:B
            self.emit('\tmov $b,$a')       # A = B (low)
            self.emit('\tdec')
            self.emit('\tmov $a,$b')       # B = B-1
            self.emit(f'\tjnz {lbl_loop}') # low byte nonzero, continue
            self.emit('\tmov $d,$a')       # A = D (high)
            self.emit('\ttst 0xFF')
            self.emit(f'\tjz {lbl_done}')  # D==0, all done
            # decd = D-- in 1B (was `dec; mov $a,$d` 2B). A := D-1, but A is
            # immediately reloaded from C at the top of the loop so the
            # clobber doesn't matter.
            self.emit('\tdecd')
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
            self.emit('\tret')

        # __eeprom_r2c_loop: sequential I2C read → code page via istc.
        # Entry: B = code dest start addr. Count on stack (pushed by caller).
        # I2C must already be in sequential read mode.
        # B is saved/restored around __i2c_rb via stack.
        # Count tracked in data[7] (set by caller's push→pop into data[7]).
        if '__eeprom_r2c_loop' in helpers:
            lbl_loop = self.label('r2c')
            lbl_last = self.label('r2cl')
            self.emit('__eeprom_r2c_loop:')
            # Entry: B = code dest, stack = [ret_addr, count, ...]
            # Save B (dest), store count to kernel-state, restore B.
            self.emit('\tpush_b')            # save dest
            self.emit('\tldsp 3')            # A = count (past: dest_pushed, ret_addr)
            self.emit_kstate_store('overlay_r2c_count')
            self.emit('\tpop_b')             # B = dest (restored)
            # Read loop: B = dest (auto-incremented by istc_inc)
            self.emit(f'{lbl_loop}:')
            self.emit('\tpush_b')            # save dest before i2c_rb clobbers B
            self.emit('\tjal __i2c_rb')      # D = byte, A/B/C clobbered
            self.emit('\tpop_b')             # B = dest
            self.emit('\tmov $d,$a')         # A = byte
            self.emit('\tistc_inc')          # code[B] = A; B = dest+1
            # Decrement count: save B (dest+1), do count--, restore B
            self.emit('\tpush_b')            # save dest+1
            self.emit_kstate_load('overlay_r2c_count')   # A = remaining
            self.emit('\tdec')               # A = remaining-1, ZF set
            self.emit_kstate_store('overlay_r2c_count')  # writeback (ZF preserved)
            self.emit('\tpop_b')             # B = dest+1 (ZF preserved: pop_b has no FI)
            self.emit(f'\tjz {lbl_last}')    # ZF from dec survives ldi+ideref
            # ACK
            self.emit_ddrb(0x03)     # SDA LOW, SCL LOW
            self.emit_ddrb(0x01)     # SDA LOW, SCL HIGH
            self.emit_ddrb(0x03)     # SCL LOW
            self.emit_ddrb(0x02)     # SDA released
            self.emit(f'\tj {lbl_loop}')
            # NACK + STOP (DDRB already 0x02 from __i2c_rb)
            self.emit(f'{lbl_last}:')
            self.emit_ddrb(0x00)     # SCL HIGH (NACK)
            self.emit_ddrb(0x02)     # SCL LOW
            self.emit_ddrb(0x03)     # SDA LOW
            self.emit_ddrb(0x01)     # SCL HIGH
            self.emit_ddrb(0x00)     # STOP
            self.emit('\tret')

        # __play_note: A = data page offset into note table.
        # Reads [half_period, cyc_lo, cyc_hi] from data page (page 1) via deref.
        # Note: ratio→half_period precomputation happens during init when
        # available; otherwise current tone entries are stored as compact
        # half-period approximations.
        # If half_period=0 and __delay_Nms available, plays silence (cyc_lo = ms).
        # Otherwise sets C=half_period and calls __tone.
        if '__play_note' in helpers:
            has_silence = '__delay_Nms' in helpers
            lbl_sil = self.label('pnsil') if has_silence else None
            self.emit('__play_note:')
            # Note table in page 1: [half_period, cyc_lo, cyc_hi]
            self.emit('\tmov $a,$d')       # D = base offset
            self.emit('\tderef')           # A = data[offset] = half_period
            if has_silence:
                self.emit('\ttst 0xFF')
                self.emit(f'\tjz {lbl_sil}')   # half_period=0 → silence
            # A = half_period (precomputed/approximated), D = base offset
            self.emit('\tmov $a,$c')       # C = half_period
            self.emit('\tmov $d,$a')       # A = offset
            self.emit('\tinc')
            self.emit('\tmov $a,$d')       # D = offset+1
            self.emit('\tderef')           # A = data[offset+1] = cyc_lo
            self.emit('\tpush $a')         # save cyc_lo
            self.emit('\tmov $d,$a')       # A = offset+1
            self.emit('\tinc')
            self.emit('\tderef')           # A = data[offset+2] = cyc_hi
            self.emit('\tmov $a,$d')       # D = cyc_hi
            self.emit('\tpop $a')          # A = cyc_lo
            self.emit('\tmov $a,$b')       # B = cyc_lo
            self.emit('\tjal __tone')
            self.emit('\tret')
            # Silence path (only if silence() is used)
            if has_silence:
                self.emit(f'{lbl_sil}:')
                self.emit('\tmov $d,$a')       # A = base offset
                self.emit('\tinc')
                self.emit('\tderef')           # A = data[offset+1] = ms
                self.emit('\tmov $a,$b')       # B = ms
                self.emit('\tjal __delay_Nms')
                self.emit('\tret')

    def _overlay_partition(self):
        """Two-stage boot overlay system.

        Stage 1 (init): Uses the full 256B code page for one-time init code
        (VIA init, I2C helpers, delay calibration, LCD init). The last act of
        stage 1 is to copy the stage 2 kernel from page 3 to the code page
        via istc_inc, then jump to main.

        Stage 2 (runtime): Compact kernel (~80-100B) + main at code page
        address 0. Overlay region starts right after kernel+main. The kernel
        was copied from page 3 by stage 1.

        Layout at upload time (code page = init code):
            [init helpers][init calls][self-copy loop][j _main padding]

        Layout after self-copy (code page = runtime):
            [kernel+main from page3_code][overlay region ...]

        Page 3 layout:
            [app globals][kernel image][manifest][overlay data...]
        """
        import sys

        # ── Two-byte instruction set for size estimation ──
        two_byte = {'ldsp','stsp','push_imm','jal','jc','jz','jnc','jnz','j','ldi',
                    'cmp','addi','subi','andi','ori','ld','st','ldsp_b','ldp3','stp3',
                    'ocall','tst','out_imm','cmpi','ddrb_imm','ddra_imm',
                    'ora_imm','orb_imm'}
        # ddrb2_imm is 3B (opcode + 2 imm), ddrb3_imm is 4B (opcode + 3 imm).

        def measure_lines(lines):
            """Measure byte size of assembly lines."""
            size = 0
            for line in lines:
                s = line.strip()
                if not s or s.endswith(':') or s.startswith(';'):
                    continue
                mn = s.split()[0]
                if mn == 'cmp':
                    parts = s.split()
                    size += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
                elif mn == 'ddrb2_imm':
                    size += 3
                elif mn == 'ddrb3_imm':
                    size += 4
                elif mn in two_byte:
                    size += 2
                else:
                    size += 1
            return size

        # ── Step 1: Estimate total code size ──
        size = measure_lines(self.code)

        # Estimate init-extractable code (inline builtins that will move to stage 1).
        # These patterns are extracted from main during init extraction and don't
        # count toward the runtime kernel size.
        init_estimate = 0
        in_main = False
        for line in self.code:
            s = line.strip()
            if s == '_main:':
                in_main = True; continue
            if in_main and s.endswith(':') and not s.startswith('.') and s.startswith('_'):
                in_main = False
            if in_main:
                if (s.startswith('exw 0') or s.startswith('ddrb_imm') or
                    s.startswith('ddra_imm') or s.startswith('exrw 2') or
                    s == 'push $a ;!keep' or s == 'pop $a ;!keep' or
                    s.startswith('clr $a') or s.startswith('ldi $d,') or
                    s.startswith('.via_') or s.startswith('.br_') or
                    s.startswith('.rcv') or s == 'nop' or
                    s.startswith('dec') or s.startswith('mov $c,$a') or
                    (s.startswith('jnz .') and any(s.endswith(x) for x in ['dly', 'rcv', 'br_'])) or
                    s == 'jal __lcd_init'):
                    mn = s.split()[0] if s else ''
                    init_estimate += 2 if mn in two_byte else 1

        runtime_estimate = size - init_estimate

        # Always proceed with partitioning if code > 250B — even if no overlays
        # are needed, init extraction is required to fit in 256B.
        if size <= 250 and not getattr(self, 'eeprom_mode', False):
            return

        # Pre-peephole the code so measure_lines matches the assembler's
        # actual byte counts (peephole optimizes ldi+push → push_imm etc.)
        self.code = peephole(self.code)
        size = measure_lines(self.code)

        # ── Step 2: Find all functions and their sizes ──
        funcs = []  # list of (name, start, end, size)
        i = 0
        _NO_OVERLAY = getattr(self, '_dynamic_no_overlay',
                              {'_main:', '_overlay_load:', '_overlay_load_p1:'})
        while i < len(self.code):
            line = self.code[i]
            s = line.strip()
            if s.endswith(':') and not s.startswith('.') and s.startswith('_') and s not in _NO_OVERLAY:
                name = s[:-1]
                start = i
                func_size = 0
                i += 1
                while i < len(self.code):
                    ns = self.code[i].strip()
                    if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                        break
                    if ns and not ns.endswith(':'):
                        mn = ns.split()[0]
                        if mn == 'cmp':
                            parts = ns.split()
                            func_size += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
                        elif mn in two_byte:
                            func_size += 2
                        else:
                            func_size += 1
                    i += 1
                funcs.append((name, start, i, func_size))
            else:
                i += 1

        if not funcs:
            return

        # ── Step 3: Classify init-only vs runtime functions ──
        # Init-only functions run once during stage 1, then are discarded.
        # Runtime functions are loaded via overlay during stage 2.
        # __tone_setup was previously in this set. But the compiler
        # emits the note-precomputation loop (which calls __tone_setup)
        # AFTER the runtime delay_cal overlay load — past the
        # init/runtime boundary. Result: `jal __tone_setup` lived in
        # runtime code while __tone_setup's body lived in stage-1 only,
        # producing a post-self-copy garbage-execution bug that V3
        # sim_corpus testing caught. Moving __tone_setup out lets the
        # knapsack put it where the call actually happens.
        #
        # __lcd_init stays init-only: that call site IS in init
        # (i2c_init precedes it and init extraction terminates cleanly
        # on `jal __lcd_init` via INIT_MARKERS in step 10b). Moving it
        # out causes kernel growth that overflows i2c_scan_lcd etc.
        INIT_ONLY_NAMES = {
            '__lcd_init',
            # __i2c_st is genuinely init-only — runtime LCD sends use
            # __i2c_st_only (no slave-address byte). __i2c_sp is included
            # because the legacy LCD driver inlined STOP in __lcd_chr;
            # the post-2026-04-24 driver calls __i2c_sp via jal from
            # __lcd_send_raw, but that's reconciled later by the
            # `_needs_runtime_i2c` no-overlay path which keeps __i2c_sp
            # resident anyway. Leaving __i2c_sp here keeps the init
            # path unchanged for programs that DON'T use the LCD —
            # they don't need __i2c_sp resident. (Removing it from
            # this set would force __i2c_sp into the overlay tier for
            # those programs, which is wasteful.)
            '__i2c_st', '__i2c_sp',
        }
        # __delay_cal is normally NOT init-only: it becomes an overlay in the
        # overlay system, freeing ~57B of init space. A compile-time placement
        # retry can opt it into stage 1 when the final stage-1 budget proves
        # there is room. That is preferable for mixed LCD/tone programs because
        # calibration runs once and the runtime only needs page3[240].
        if getattr(self, '_prefer_delay_cal_init', False):
            INIT_ONLY_NAMES.add('__delay_cal')
        # __eeprom_rd is init-only when not used by runtime eeprom array access
        if not getattr(self, '_needs_runtime_eeprom_rd', False):
            INIT_ONLY_NAMES.add('__eeprom_rd')
        # I2C helpers that are init-only when no runtime I2C is needed
        I2C_INIT_ONLY = {'__i2c_sb', '__i2c_sp', '__i2c_st', '__i2c_st_only', '__i2c_rb', '__i2c_rs'}

        needs_runtime_i2c = getattr(self, '_needs_runtime_i2c', False)

        def is_init_only(name):
            """Return True if this function is only needed during init (stage 1)."""
            if name in INIT_ONLY_NAMES:
                return True
            if name in I2C_INIT_ONLY and not needs_runtime_i2c:
                return True
            return False

        init_only_funcs = []    # (name, start, end, size) - go in stage 1
        overlay_eligible = []   # (name, start, end, size) - candidates for overlay

        for name, start, end, fsize in funcs:
            if is_init_only(name):
                init_only_funcs.append((name, start, end, fsize))
            else:
                overlay_eligible.append((name, start, end, fsize))

        # Sort overlay-eligible by size descending
        overlay_eligible.sort(key=lambda f: -f[3])

        # Decide which functions must be overlayed
        # In eeprom_mode, overlay everything that's eligible
        # Otherwise, overlay until resident code fits
        overlay_funcs = []
        remaining_size = size
        # Rough estimate: kernel + main overhead
        kernel_estimate = 80  # kernel loader estimate
        main_overhead = 10    # self-copy + j _main

        if getattr(self, 'eeprom_mode', False):
            overlay_funcs = list(overlay_eligible)
            remaining_size = size - sum(f[3] for f in overlay_eligible)
        else:
            # Overlay functions until remaining code fits in the code page.
            # When total code is very large (>350B), overlay ALL eligible functions
            # to keep the kernel as small as possible.
            target = 240 - kernel_estimate - main_overhead
            if size > 350:
                # Large program: overlay everything, keep kernel minimal
                overlay_funcs = list(overlay_eligible)
                remaining_size = size - sum(f[3] for f in overlay_eligible)
            else:
                for name, start, end, fsize in overlay_eligible:
                    if remaining_size <= target:
                        break
                    overlay_funcs.append((name, start, end, fsize))
                    remaining_size -= fsize

        if not overlay_funcs:
            if size <= 250:
                return  # fits flat

            # No runtime overlays but code > 250B. Init extraction needed.
            # Find boundary: user code ends at first __ helper label.
            helper_start = None
            main_start = None
            for fi in range(len(self.code)):
                s = self.code[fi].strip()
                if s == '_main:':
                    main_start = fi
                elif main_start is not None and helper_start is None:
                    if s.endswith(':') and s.startswith('__'):
                        helper_start = fi
                        break
            if main_start is None:
                return
            if helper_start is None:
                helper_start = len(self.code)

            user_code = self.code[main_start:helper_start]
            helpers = self.code[helper_start:]

            # Classify helpers as init-only or runtime
            init_only_names = {'__lcd_init', '__tone_setup'}
            if not getattr(self, '_needs_runtime_i2c', False):
                init_only_names.update({'__i2c_sb', '__i2c_sp', '__i2c_st', '__i2c_rb', '__i2c_rs'})

            init_helper_lines = []
            runtime_helper_lines = []
            hi = 0
            while hi < len(helpers):
                s = helpers[hi].strip()
                if s.endswith(':') and s.startswith('__') and not s.startswith('.'):
                    fname = s[:-1]
                    start = hi
                    hi += 1
                    while hi < len(helpers):
                        ns = helpers[hi].strip()
                        if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                            break
                        hi += 1
                    block = helpers[start:hi]
                    if fname in init_only_names:
                        init_helper_lines.extend(block)
                    else:
                        runtime_helper_lines.extend(block)
                else:
                    hi += 1

            # Extract init-only inline code from user_code
            INIT_MARKERS_SET = {
                'exw 0 0', 'exw 0 1', 'exw 0 3',
                'push $a ;!keep', 'pop $a ;!keep',
            }
            INIT_PREFIXES_SET = ('ddrb_imm', 'exrw 2', 'exw ', 'ldi $d,', 'ddra_imm', 'push_imm')
            INIT_JAL_TARGETS = {'__lcd_init', '__tone_setup'}
            # __delay_cal excluded: runs as overlay at _main start (not init)
            # Note: __i2c_stream, __i2c_st, __i2c_sp are NOT init-only targets —
            # they can appear in runtime code too. Only delay_cal/lcd_init/tone_setup
            # are definitively init-only calls that extend the init phase.

            init_inline = []
            runtime_main = []
            in_init = True
            for mi, line in enumerate(user_code):
                s = line.strip()
                if s == '_main:':
                    continue
                if not in_init:
                    runtime_main.append(line)
                    continue
                # Check if this line is init
                is_init = (s in INIT_MARKERS_SET or
                           any(s.startswith(p) for p in INIT_PREFIXES_SET) or
                           s.startswith('clr $a') or s.startswith('dec') or
                           s.startswith('.via_') or s.startswith('.br_') or
                           s.startswith('.rcv') or s.startswith('.i2c_init_br') or
                           s.startswith('.__precomp') or
                           s.startswith('.__preskip') or
                           s.startswith('mov $c,$a') or
                           s == 'nop')
                if s.startswith('jnz .') or s.startswith('j .'):
                    is_init = True
                # ldi before init-compatible jal or exw is init.
                # Common pattern: `ldi $a, val; exw E M` for user VIA init.
                if s.startswith('ldi $a,') or s.startswith('ldi $b,'):
                    for nli in range(mi + 1, min(mi + 4, len(user_code))):
                        ns = user_code[nli].strip()
                        if ns.startswith('jal '):
                            if ns.split()[1] in INIT_JAL_TARGETS:
                                is_init = True
                            break
                        if ns.startswith('exw ') or ns.startswith('ddrb_imm') or ns.startswith('ddra_imm'):
                            is_init = True
                            break
                if s.startswith('jal '):
                    target = s.split()[1]
                    if target in INIT_JAL_TARGETS:
                        is_init = True
                        # After the LAST init-only jal, end init phase.
                        # Look ahead: if no more init-only jals follow, terminate.
                        has_more_init_jal = False
                        for fli in range(mi + 1, len(user_code)):
                            fs = user_code[fli].strip()
                            if fs.startswith('jal ') and fs.split()[1] in INIT_JAL_TARGETS:
                                has_more_init_jal = True
                                break
                            if fs.startswith('jal ') and fs.split()[1] not in INIT_JAL_TARGETS:
                                break
                        if not has_more_init_jal:
                            # This is the last init jal — extract it, then end init
                            init_inline.append(line)
                            in_init = False
                            continue
                    else:
                        in_init = False
                if is_init:
                    init_inline.append(line)
                else:
                    in_init = False
                    runtime_main.append(line)

            while runtime_main and runtime_main[-1].strip() == 'ret':
                runtime_main.pop()

            # Include runtime helpers that init code needs (e.g., __i2c_sb
            # called by __lcd_init's EEPROM read, even when I2C is also runtime)
            # Iteratively resolve transitive dependencies: if __eeprom_rd
            # calls __i2c_rb, copying __eeprom_rd reveals __i2c_rb as missing.
            # Loop until no new missing helpers are found.
            any_missing = True
            rename_map = {}
            _prev_missing = None
            while any_missing:
                init_jal_targets = set()
                for line in init_inline + init_helper_lines:
                    s = line.strip()
                    if s.startswith('jal __'):
                        init_jal_targets.add(s.split()[1])
                init_helper_names = set()
                for line in init_helper_lines:
                    s = line.strip()
                    if s.endswith(':') and s.startswith('__'):
                        init_helper_names.add(s[:-1])
                missing = init_jal_targets - init_helper_names
                # Termination guard: if this iteration's `missing` set is
                # identical to the previous iteration's, the inner copy
                # loop made no progress (the missing helpers don't exist
                # in runtime_helper_lines either). Without this guard the
                # outer `while any_missing:` loop spins forever — observed
                # on test_i2c_ack_diag.c where __i2c_st_only was referenced
                # by __lcd_init but never registered (now fixed above; this
                # is the safety net so a similar registration miss in
                # future doesn't hang the compiler).
                if missing == _prev_missing:
                    raise Exception(
                        f"init-helper resolver: cannot satisfy jal targets "
                        f"{sorted(missing)} — these are referenced by init code "
                        f"but no definition exists in either init or runtime "
                        f"helper sections. Likely a missing _lcd_helpers.add() "
                        f"in a builtin emitter."
                    )
                _prev_missing = set(missing)
                any_missing = bool(missing)
                if not missing: break
                # Copy needed helpers from runtime to init, renaming to avoid
                # label conflicts with the same helpers in the kernel section.
                # Rename __foo → __i_foo in both definitions and jal targets.
                hi = 0
                while hi < len(runtime_helper_lines):
                    s = runtime_helper_lines[hi].strip()
                    if s.endswith(':') and s.startswith('__') and not s.startswith('.'):
                        fname = s[:-1]
                        start = hi
                        hi += 1
                        while hi < len(runtime_helper_lines):
                            ns = runtime_helper_lines[hi].strip()
                            if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                                break
                            hi += 1
                        if fname in missing:
                            iname = fname.replace('__', '__i_', 1)
                            rename_map[fname] = iname
                            block = list(runtime_helper_lines[start:hi])
                            # Rename label definition, internal jal targets,
                            # AND local labels (e.g., .rb9 → .i_rb9) to avoid
                            # conflicts with the same labels in the kernel.
                            # Collect local labels first
                            local_labels = set()
                            for line in block:
                                s = line.strip()
                                if s.startswith('.') and s.endswith(':'):
                                    local_labels.add(s[:-1])  # e.g., '.rb9'
                            renamed = []
                            for line in block:
                                for old, new in rename_map.items():
                                    line = line.replace(old + ':', new + ':')
                                    line = line.replace('jal ' + old, 'jal ' + new)
                                # Rename local labels: .foo → .i_foo
                                for ll in local_labels:
                                    line = line.replace(ll, '.i_' + ll[1:])
                                renamed.append(line)
                            init_helper_lines.extend(renamed)
                    else:
                        hi += 1
                # Also rename jal targets in init_inline and existing init_helper_lines
                if rename_map:
                    for idx in range(len(init_inline)):
                        for old, new in rename_map.items():
                            init_inline[idx] = init_inline[idx].replace('jal ' + old, 'jal ' + new)
                    for idx in range(len(init_helper_lines)):
                        for old, new in rename_map.items():
                            if 'jal ' + old in init_helper_lines[idx]:
                                init_helper_lines[idx] = init_helper_lines[idx].replace('jal ' + old, 'jal ' + new)

            # Strip unreferenced helpers from runtime kernel to save bytes.
            # Collect all jal targets from runtime_main + runtime_helper_lines.
            runtime_refs = set()
            for line in runtime_main + runtime_helper_lines:
                s = line.strip()
                if s.startswith('jal ') or (s.startswith('j ') and '__' in s):
                    runtime_refs.add(s.split()[1])
            # Filter runtime_helper_lines: keep only referenced functions
            filtered_helpers = []
            hi = 0
            while hi < len(runtime_helper_lines):
                s = runtime_helper_lines[hi].strip()
                if s.endswith(':') and s.startswith('__') and not s.startswith('.'):
                    fname = s[:-1]
                    start = hi
                    hi += 1
                    while hi < len(runtime_helper_lines):
                        ns = runtime_helper_lines[hi].strip()
                        if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                            break
                        hi += 1
                    if fname in runtime_refs:
                        filtered_helpers.extend(runtime_helper_lines[start:hi])
                else:
                    filtered_helpers.append(runtime_helper_lines[hi])
                    hi += 1
            runtime_helper_lines = filtered_helpers

            # Identify SHARED helpers: functions needed by both init and kernel.
            # These are placed ONCE in the code page at addresses above
            # kernel_size, surviving the self-copy. Eliminates duplication.
            init_refs = set()
            for line in init_inline + init_helper_lines:
                s = line.strip()
                if s.startswith('jal __'):
                    init_refs.add(s.split()[1])
            # Map renamed init refs back to original names
            init_refs_orig = set()
            for ref in init_refs:
                for old, new in rename_map.items():
                    if ref == new:
                        init_refs_orig.add(old)
                        break

            kernel_refs = set()
            for line in runtime_main + runtime_helper_lines:
                s = line.strip()
                if s.startswith('jal ') or (s.startswith('j ') and '__' in s):
                    kernel_refs.add(s.split()[1])

            shared_names = init_refs_orig & kernel_refs  # helpers needed by both
            # Extract shared helpers from runtime_helper_lines
            shared_helper_lines = []
            kernel_only_helpers = []
            hi = 0
            while hi < len(runtime_helper_lines):
                s = runtime_helper_lines[hi].strip()
                if s.endswith(':') and s.startswith('__') and not s.startswith('.'):
                    fname = s[:-1]
                    start = hi
                    hi += 1
                    while hi < len(runtime_helper_lines):
                        ns = runtime_helper_lines[hi].strip()
                        if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                            break
                        hi += 1
                    if fname in shared_names:
                        shared_helper_lines.extend(runtime_helper_lines[start:hi])
                    else:
                        kernel_only_helpers.extend(runtime_helper_lines[start:hi])
                else:
                    kernel_only_helpers.append(runtime_helper_lines[hi])
                    hi += 1

            # Catch helpers classified as init-only that are ALSO called from
            # runtime code. Without this, runtime jal targets resolve to the
            # init-only helper's address, which gets overwritten by self-copy
            # → silent corruption (e.g. jal __i2c_sb hangs in stage 2).
            # Move any such helper from init_helper_lines to shared_helper_lines.
            init_only_runtime_called = set()
            for line in runtime_main + kernel_only_helpers:
                s = line.strip()
                if s.startswith('jal __') or (s.startswith('j __')):
                    init_only_runtime_called.add(s.split()[1])
            extra_shared_lines = []
            cleaned_init_again = []
            hi = 0
            while hi < len(init_helper_lines):
                s = init_helper_lines[hi].strip()
                if s.endswith(':') and s.startswith('__') and not s.startswith('.') and not s.startswith('__i_'):
                    fname = s[:-1]
                    start = hi
                    hi += 1
                    while hi < len(init_helper_lines):
                        ns = init_helper_lines[hi].strip()
                        if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                            break
                        hi += 1
                    if fname in init_only_runtime_called and fname not in shared_names:
                        # Move from init to shared so runtime jal targets are valid.
                        extra_shared_lines.extend(init_helper_lines[start:hi])
                        shared_names = shared_names | {fname}
                    else:
                        cleaned_init_again.extend(init_helper_lines[start:hi])
                else:
                    cleaned_init_again.append(init_helper_lines[hi])
                    hi += 1
            init_helper_lines = cleaned_init_again
            shared_helper_lines = shared_helper_lines + extra_shared_lines

            # Remove renamed copies of shared helpers from init (they'll use the shared versions)
            if shared_names:
                cleaned_init = []
                hi = 0
                while hi < len(init_helper_lines):
                    s = init_helper_lines[hi].strip()
                    if s.endswith(':') and s.startswith('__i_') and not s.startswith('.'):
                        orig = s[:-1].replace('__i_', '__', 1)
                        start = hi
                        hi += 1
                        while hi < len(init_helper_lines):
                            ns = init_helper_lines[hi].strip()
                            if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                                break
                            hi += 1
                        if orig in shared_names:
                            continue  # skip this renamed copy
                        cleaned_init.extend(init_helper_lines[start:hi])
                    else:
                        cleaned_init.append(init_helper_lines[hi])
                        hi += 1
                init_helper_lines = cleaned_init

                # Update jal targets in init: __i_foo → __foo for shared helpers
                for idx in range(len(init_inline)):
                    for name in shared_names:
                        renamed = name.replace('__', '__i_', 1)
                        init_inline[idx] = init_inline[idx].replace('jal ' + renamed, 'jal ' + name)
                for idx in range(len(init_helper_lines)):
                    for name in shared_names:
                        renamed = name.replace('__', '__i_', 1)
                        init_helper_lines[idx] = init_helper_lines[idx].replace('jal ' + renamed, 'jal ' + name)

            # Build runtime kernel WITHOUT shared helpers
            runtime_kernel = ['_main:'] + runtime_main + kernel_only_helpers
            kernel_size = measure_lines(runtime_kernel)

            if kernel_size > 250:
                import sys
                print(f"Warning: runtime kernel {kernel_size}B > 248B, program may not fit",
                      file=sys.stderr)
                return

            # Build output: init code + copy loop + shared helpers (at safe addresses)
            new_code = []

            # Init extraction path doesn't use page2 overlays — no SP init needed.
            # delay_calibrate auto-insertion now handled in lcd_init builtin
            # Stage 1: inline code, skip over init-only helpers, copy loop, shared helpers
            new_code.extend(init_inline)
            new_code.append('\tj __init_done')
            new_code.extend(init_helper_lines)
            new_code.append('__init_done:')

            # Self-copy: copy kernel from page3 to code page.
            # The copy loop must be placed AFTER the kernel's last byte
            # to avoid overwriting itself. Pad init section if needed.
            copy_loop_size = 14  # ldi(2)+ldi(2)+loop(6×1B+jnz 2B)+j(2) = 14 bytes
            copy_addr = kernel_size  # place copy right after kernel area
            init_size = measure_lines(new_code)
            import sys
            # (init extraction size info is in the memory report)
            if init_size < copy_addr:
                # Jump over the gap (2B for jump instruction)
                pad = copy_addr - init_size - 2
                # NOP sled to reach copy_addr. Label prevents peephole stripping.
                new_code.append('\tj __copy_start')
                new_code.append('__copy_start:')
                for _ in range(pad):
                    new_code.append('\tnop')

            # Self-copy: B=code dest, istc_inc does code[B]=A, B++, A=old_B.
            # B auto-tracks both source and dest (they're the same offset).
            # D = countdown. After istc_inc, A=old_B; next iter mov $b,$a gives A=new_B.
            copy_lbl = self.label('sc_lp')
            new_code.append(f'\tldi $b,0')
            new_code.append(f'\tldi $d,{kernel_size}')
            new_code.append(f'{copy_lbl}:')
            new_code.append('\tmov $b,$a')       # A = B (source offset)
            new_code.append('\tderefp3')         # A = page3[B]
            new_code.append('\tistc_inc')        # code[B]=A, B++, A=old_B
            new_code.append('\tmov $d,$a')       # A = D (count)
            new_code.append('\tdec')
            new_code.append('\tmov $a,$d')       # D--
            new_code.append(f'\tjnz {copy_lbl}')
            new_code.append('\tj 0')
            # Shared helpers: placed AFTER copy loop at addresses >= kernel_size.
            # Self-copy only writes code_page[0..kernel_size-1], so these survive.
            # Stage 2 jal targets resolve to these code_page addresses correctly.
            new_code.extend(shared_helper_lines)

            # Kernel in page3_kernel (resets code PC to 0 for self-copy)
            new_code.append('\tsection page3_kernel')
            new_code.extend(runtime_kernel)

            # LCD init data — no emission needed here (stored in EEPROM)
            if hasattr(self, '_lcd_print_strings'):
                # Strings live in page 1 with a per-string symbolic
                # label (`__str_N`). See the codegen path in gen_expr's
                # 'string' case for the rationale on why this isn't
                # page 3 anymore.
                if not any('section data' in l for l in new_code[-5:]):
                    new_code.append('\tsection data')
                for label, s, d_off in self._lcd_print_strings:
                    new_code.append(f'{label}:')
                    for ch in s:
                        new_code.append(f'\tbyte {ord(ch)}')
                    new_code.append('\tbyte 0')  # null terminator

            self.code = new_code
            self._init_extraction_done = True
            return

        # ── Step 4: Build overlay slots via SCC (Tarjan's algorithm) ──
        # Only mutually recursive functions MUST share an overlay slot.
        # A calls B (one-way) → B can be a separate overlay, loaded before A,
        # or bundled into A's overlay. This is decided by the knapsack/bundling.
        ov_names = {name for name, _, _, _ in overlay_funcs}
        ov_by_name = {name: (start, end, fsize) for name, start, end, fsize in overlay_funcs}

        # Build DIRECTED adjacency (A→B means A calls B)
        adj = {name: set() for name in ov_names}
        for name, start, end, _ in overlay_funcs:
            for li in range(start, end):
                s = self.code[li].strip()
                if s.startswith('jal ') or s.startswith('j '):
                    target = s.split()[1]
                    if target in ov_names and target != name:
                        adj[name].add(target)
                        # NOTE: no reverse edge — unidirectional

        # Tarjan's SCC algorithm
        scc_index = [0]
        scc_stack = []
        scc_on_stack = set()
        scc_indices = {}
        scc_lowlinks = {}
        components = []

        def strongconnect(v):
            scc_indices[v] = scc_lowlinks[v] = scc_index[0]
            scc_index[0] += 1
            scc_stack.append(v)
            scc_on_stack.add(v)

            for w in adj.get(v, set()):
                if w not in scc_indices:
                    strongconnect(w)
                    scc_lowlinks[v] = min(scc_lowlinks[v], scc_lowlinks[w])
                elif w in scc_on_stack:
                    scc_lowlinks[v] = min(scc_lowlinks[v], scc_indices[w])

            if scc_lowlinks[v] == scc_indices[v]:
                comp = []
                while True:
                    w = scc_stack.pop()
                    scc_on_stack.discard(w)
                    comp.append(w)
                    if w == v:
                        break
                components.append(comp)

        for v in ov_names:
            if v not in scc_indices:
                strongconnect(v)

        reload_thunk_edges = set()

        # Post-SCC merge: if any function in component X calls a function in
        # component Y, merge X and Y. This handles user-calls-user cases where
        # A→B is one-way but B must be in A's overlay (can't call across overlays).
        # Exception: for expensive one-way user calls, keep the functions in
        # separate overlays and insert a resident reload thunk later. The thunk
        # costs ~11B resident but can save far more overlay-region pressure when
        # a large caller repeatedly uses a large helper (e.g. RTC read wrappers).
        comp_of = {}
        for ci, comp in enumerate(components):
            for name in comp:
                comp_of[name] = ci
        parent = list(range(len(components)))
        def _find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        def _union(x, y):
            px, py = _find(x), _find(y)
            if px != py:
                parent[py] = px
        def _should_split_reload_edge(caller, callee):
            if caller.startswith('__') or callee.startswith('__'):
                return False
            if not caller.startswith('_main_p'):
                return False
            cs = ov_by_name.get(caller, (0, 0, 0))[2]
            ds = ov_by_name.get(callee, (0, 0, 0))[2]
            if ds < 24:
                return False
            call_count = 0
            start, end, _ = ov_by_name.get(caller, (0, 0, 0))
            for line in self.code[start:end]:
                if line.strip() == f'jal {callee}':
                    call_count += 1
            # One thunk per caller/callee edge costs about 11B resident. Split
            # only when the avoided merge is clearly larger than that cost, and
            # bias toward repeated calls where one thunk amortizes well.
            return (cs + ds >= 60 and (call_count >= 2 or ds >= 40))

        for ci, comp in enumerate(components):
            for name in comp:
                for called in adj.get(name, set()):
                    called_ci = comp_of.get(called)
                    if called_ci is not None and called_ci != ci:
                        if _should_split_reload_edge(name, called):
                            reload_thunk_edges.add((name, called))
                            continue
                        _union(ci, called_ci)
        merged = {}
        for ci, comp in enumerate(components):
            root = _find(ci)
            merged.setdefault(root, []).extend(comp)
        components = list(merged.values())

        # ── Step 5: Build overlay slots via library inlining ──
        # Each user function gets its own overlay containing ALL library helpers
        # it depends on. Library helpers are never split from their callers.
        # If a user+library combination exceeds the overlay region → fatal error.
        overlay_slots = []  # list of ([(name, start, end, fsize), ...], total_size)

        # Pre-compute external callers so a slot's most likely entry point is
        # first. Multi-entry dispatch now calls the requested function label
        # after copying the slot, but a stable first function still keeps
        # diagnostics and overlay reports readable.
        def _slot_entry_fn(comp_names):
            """Pick a readable primary function for a slot. Heuristic: a
            function called by NONE of the other functions in the slot is
            likely an external entry point. User functions preferred over
            library helpers. `comp_names` contains bare names (no leading `_`);
            the asm `jal _foo` targets carry the underscore — normalize on
            comparison.
            """
            comp_set = set(comp_names)
            # Build intra-component call graph: who calls whom?
            called_by_intra = {nm: False for nm in comp_names}
            for nm in comp_names:
                # Find this function's body in self.code and scan for jals
                start, end, _ = ov_by_name.get(nm, (0, 0, 0))
                for line in self.code[start:end]:
                    s = line.strip()
                    if s.startswith('jal '):
                        tgt = s.split()[1]
                        # Accept direct match or stripped-underscore match —
                        # depending on how comp_names is built, entries may
                        # or may not carry a leading '_'.
                        matched = None
                        if tgt in comp_set:
                            matched = tgt
                        elif tgt.startswith('_') and not tgt.startswith('__') and tgt[1:] in comp_set:
                            matched = tgt[1:]
                        if matched and matched != nm:
                            called_by_intra[matched] = True
            # Entry candidates: NOT called by any intra function.
            externals = [nm for nm in comp_names if not called_by_intra[nm]]
            candidates = externals if externals else list(comp_names)
            # Prefer user functions over library helpers
            user_first = [n for n in candidates if not n.startswith('__')]
            return (user_first + candidates)[0]

        for comp in components:
            # Reorder so the primary externally-called user function is first.
            entry_name = _slot_entry_fn(comp)
            if entry_name and entry_name in comp:
                comp_ordered = [entry_name] + [n for n in comp if n != entry_name]
            else:
                comp_ordered = list(comp)
            slot = [(n, *ov_by_name[n]) for n in comp_ordered]
            slot_size = sum(f for _, _, _, f in slot)

            # Separate library helpers from user functions
            lib_funcs = [(n, s, e, f) for n, s, e, f in slot if n.startswith('__')]
            user_funcs = [(n, s, e, f) for n, s, e, f in slot if not n.startswith('__')]
            lib_size = sum(f for _, _, _, f in lib_funcs)

            if not user_funcs:
                # All library (e.g., init-only group) — keep as one slot
                overlay_slots.append((slot, slot_size))
            elif not lib_funcs:
                # All user — they're in the same component (call each other),
                # so they must be in ONE overlay slot together. `slot` was
                # reordered above so the entry function is slot[0].
                overlay_slots.append((slot, slot_size))
            else:
                # Mixed: create one overlay per user func + ALL lib helpers
                for uf in user_funcs:
                    combined = [uf] + list(lib_funcs)
                    combined_size = uf[3] + lib_size
                    overlay_slots.append((combined, combined_size))

        overlay_slots.sort(key=lambda x: -x[1])
        overlay_slots.sort(key=lambda x: -x[1])
        max_slot_size = overlay_slots[0][1] if overlay_slots else 0

        # ── Step 6: Extract function bodies for overlays ──
        # Build name→index mapping and combined overlay blocks
        overlay_asm_blocks = []  # (entry_name, combined_lines, slot_size)
        overlay_meta = []        # (idx, entry_name, slot_size)
        func_to_overlay_idx = {}
        func_to_overlay_offset = {}

        for idx, (slot_funcs, slot_size) in enumerate(overlay_slots):
            combined_lines = []
            slot_off = 0
            for name, start, end, fsize in slot_funcs:
                func_to_overlay_offset[name] = slot_off
                combined_lines.extend(self.code[start:end])
                func_to_overlay_idx[name] = idx
                slot_off += fsize
            overlay_asm_blocks.append((slot_funcs[0][0], combined_lines, slot_size))
            overlay_meta.append((idx, slot_funcs[0][0], slot_size))

        # Flat list for code removal
        overlay_funcs_flat = []
        for slot_funcs, _ in overlay_slots:
            overlay_funcs_flat.extend(slot_funcs)

        # NOTE: Overlay self-containment validation moved to AFTER step 9
        # (helper inlining). Validating here would flag cross-overlay jal
        # targets that step 9 will inline, producing false positives.

        # ── Step 7: Extract main body ──
        # Find _main label and its body (everything from _main: to next global label)
        main_start = None
        main_end = None
        for i, line in enumerate(self.code):
            s = line.strip()
            if s == '_main:':
                main_start = i
                j = i + 1
                while j < len(self.code):
                    ns = self.code[j].strip()
                    if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                        break
                    j += 1
                main_end = j
                break

        if main_start is None:
            return  # no main, can't overlay

        main_body = self.code[main_start:main_end]

        # ── Step 8: Remove overlay functions from main body and rewrite calls ──
        # Also remove init-only functions (they go to stage 1 init code)
        remove_ranges = set()
        for name, start, end, fsize in overlay_funcs_flat:
            remove_ranges.add((start, end))
        for name, start, end, fsize in init_only_funcs:
            remove_ranges.add((start, end))

        # Build new main body: resident code minus overlay bodies, with calls rewritten
        # We process the ENTIRE self.code (not just main body) to get all resident code
        new_resident = []
        cache_init_emitted = False
        i = 0
        while i < len(self.code):
            skip = False
            for rs, re in remove_ranges:
                if rs <= i < re:
                    i = re
                    skip = True
                    break
            if not skip:
                line = self.code[i]
                s = line.strip()
                rewritten = False
                if s.startswith('jal '):
                    target = s.split()[1]
                    idx = func_to_overlay_idx.get(target)
                else:
                    idx = None
                if idx is not None:
                    # Clean up ldsp pattern before overlay call
                    if (len(new_resident) >= 3 and
                        new_resident[-1].strip().startswith('ldsp ') and
                        new_resident[-3].strip() == 'push $a'):
                        ldi_b_line = new_resident[-2]
                        new_resident = new_resident[:-3]
                        new_resident.append(ldi_b_line)
                    # Multi-entry overlay dispatch:
                    # Caller sets $c = overlay index, leaves $a/$b as args.
                    # `_overlay_load` copies the slot and returns; the caller
                    # then jumps to the requested function label inside that
                    # loaded slot. This is required because merged slots can
                    # have multiple externally-called functions.
                    new_resident.append(f'\tldi $c,{idx}')
                    new_resident.append('\tjal _overlay_load')
                    new_resident.append(f'\tjal {target}')
                    rewritten = True
                if not rewritten:
                    new_resident.append(line)
                i += 1

        # ── Step 8b: Deferred reclassification ──
        # Helpers marked _conditionally_resident can be bundled into overlays
        # if they're never called from non-helper resident code (_main, loader).
        # This must run AFTER step 8 builds new_resident with overlay calls rewritten.
        cond_res = getattr(self, '_conditionally_resident', set())
        if cond_res:
            # Find all helper body ranges in new_resident
            helper_ranges = {}  # name → (start, end)
            hi2 = 0
            while hi2 < len(new_resident):
                s2 = new_resident[hi2].strip()
                if s2.endswith(':') and s2.startswith('__') and not s2.startswith('.'):
                    hname2 = s2[:-1]
                    hstart2 = hi2
                    hi2 += 1
                    while hi2 < len(new_resident):
                        ns2 = new_resident[hi2].strip()
                        if ns2.endswith(':') and not ns2.startswith('.') and ns2.startswith('_'):
                            break
                        hi2 += 1
                    helper_ranges[hname2] = (hstart2, hi2)
                else:
                    hi2 += 1

            # For each conditionally-resident helper, check if it's called
            # from non-bundleable code (i.e., _main or RESIDENT helpers).
            # Only skip bodies of OTHER conditionally-resident helpers.
            cond_res_ranges = set()
            for hname2 in cond_res:
                if hname2 in helper_ranges:
                    hs, he = helper_ranges[hname2]
                    cond_res_ranges.update(range(hs, he))

            # Tone-runtime helpers must stay resident even when only called
            # from user-function overlays. Bundling them inside an overlay
            # has been observed to hang programs that play tone() from a
            # single user function (e.g. twinkle.c's `p1`); the
            # multi-callsite case works because the knapsack already keeps
            # them resident there. Root cause of the bundled-mode failure
            # is not yet pinned down; until it is, force-resident.
            _force_resident_helpers = {'__tone', '__play_note', '__delay_Nms'}

            for hname in list(cond_res):
                if hname in _force_resident_helpers:
                    continue   # leave in _NO_OVERLAY → stays resident
                called_from_resident = False
                for ci, cline in enumerate(new_resident):
                    # Skip other conditionally-resident helper bodies
                    # but DO check resident helper bodies (they stay in kernel)
                    if ci in cond_res_ranges:
                        # Exception: don't skip our OWN body
                        if hname in helper_ranges:
                            own_s, own_e = helper_ranges[hname]
                            if own_s <= ci < own_e:
                                continue  # skip own body
                        continue  # skip other cond_res bodies
                    cs = cline.strip()
                    if f'jal {hname}' in cs:
                        called_from_resident = True
                        break
                if not called_from_resident:
                    _NO_OVERLAY.discard(f'{hname}:')

            # Residency propagation: if a RESIDENT helper calls/jumps to a
            # bundleable helper, that helper must also stay resident.
            # Also handles fall-through (last instruction not a terminator).
            # Iterate until no more changes (transitive closure).
            changed = True
            while changed:
                changed = False
                helper_order_8b = sorted(helper_ranges.keys(),
                                         key=lambda h: helper_ranges[h][0])
                for idx_h, hname in enumerate(helper_order_8b):
                    if f'{hname}:' not in _NO_OVERLAY:
                        continue  # bundleable, skip
                    # This helper is RESIDENT. Check its body for refs to bundleable helpers.
                    hs, he = helper_ranges[hname]
                    for ri in range(hs, he):
                        cs = new_resident[ri].strip()
                        for target_name in list(cond_res):
                            if f'{target_name}:' in _NO_OVERLAY:
                                continue  # already resident
                            if (f'jal {target_name}' in cs or
                                (f'j {target_name}' in cs and f'jal {target_name}' not in cs)):
                                _NO_OVERLAY.add(f'{target_name}:')
                                cond_res.discard(target_name)
                                changed = True
                    # Fall-through: if body doesn't end with terminator
                    terminators = {'ret', 'hlt'}
                    last_instr = ''
                    for ri in range(he - 1, hs, -1):
                        stripped = new_resident[ri].strip()
                        if stripped and not stripped.endswith(':') and not stripped.startswith(';'):
                            last_instr = stripped.split()[0]
                            break
                    is_term = (last_instr in terminators or
                               last_instr.startswith('j') and not last_instr.startswith('jal'))
                    if not is_term and idx_h + 1 < len(helper_order_8b):
                        next_name = helper_order_8b[idx_h + 1]
                        if f'{next_name}:' not in _NO_OVERLAY:
                            _NO_OVERLAY.add(f'{next_name}:')
                            cond_res.discard(next_name)
                            changed = True

        # Snapshot pre-bundling state so placement retry can rebuild
        # overlays with an updated resident set. Placement retry adds a
        # bundled helper to _NO_OVERLAY, then this loop re-runs step 9
        # so that helper is emitted in the kernel instead of bundled.
        _snap_new_resident = list(new_resident)
        _snap_overlay_asm_blocks = [(n, list(l), s) for n, l, s in overlay_asm_blocks]
        _bundle_placement_pass = 0
        _retry_bundling = True
        while _retry_bundling:
            _retry_bundling = False
            _bundle_placement_pass += 1
            if _bundle_placement_pass > 10:
                raise Exception("bundle/placement retry did not converge after 10 passes")
            if _bundle_placement_pass > 1:
                new_resident = list(_snap_new_resident)
                overlay_asm_blocks = [(n, list(l), s) for n, l, s in _snap_overlay_asm_blocks]

            # ── Step 9: Bundle/inline helpers only used by overlay code ──
            # If a helper is not in _NO_OVERLAY and never called from resident code,
            # BUNDLE it into each overlay that needs it (append body, keep jal intact).
            # For helpers called multiple times within an overlay (e.g., __i2c_sb × 4
            # from __lcd_chr), bundling is far better than inlining (39B vs 156B).
            overlay_names = {name for name, _, _, _ in overlay_funcs_flat}
            helper_bodies = {}
            hi = 0
            while hi < len(new_resident):
                hs = new_resident[hi].strip()
                if hs.endswith(':') and hs.startswith('__') and not hs.startswith('.'):
                    hname = hs[:-1]
                    hstart = hi
                    hi += 1
                    hlines = []
                    while hi < len(new_resident):
                        ns = new_resident[hi].strip()
                        if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                            break
                        hlines.append(new_resident[hi])
                        hi += 1
                    helper_bodies[hname] = (hstart, hi, hlines)
                else:
                    hi += 1

            # Save original ranges before merging (for correct removal later)
            helper_original_ranges = {h: (s, e) for h, (s, e, _) in helper_bodies.items()}

            # Detect fall-through: if a helper body doesn't end with a terminator,
            # it falls through to the next label. Merge it with that label's body
            # so bundling preserves the fall-through semantics.
            terminators = {'ret', 'hlt'}
            helper_order = sorted(helper_bodies.keys(),
                                  key=lambda h: helper_bodies[h][0])
            for idx_h, hname in enumerate(helper_order):
                hstart, hend, hlines = helper_bodies[hname]
                # Find last real instruction
                last_instr = ''
                for hl in reversed(hlines):
                    stripped = hl.strip()
                    if stripped and not stripped.endswith(':') and not stripped.startswith(';'):
                        last_instr = stripped.split()[0]
                        break
                is_terminator = (last_instr in terminators or
                                 last_instr == 'j' or
                                 (last_instr.startswith('j') and not last_instr.startswith('jal')))
                if not is_terminator and idx_h + 1 < len(helper_order):
                    # Falls through to next helper — merge bodies
                    next_name = helper_order[idx_h + 1]
                    ns, ne, nlines = helper_bodies[next_name]
                    # Extend current helper to include next helper's label + body
                    merged_lines = hlines + [new_resident[ns]] + nlines
                    helper_bodies[hname] = (hstart, ne, merged_lines)
                    # Don't remove the next helper from helper_bodies — it can
                    # still be bundled independently by overlays that call it directly

            # Identify which helpers can be bundled (not in _NO_OVERLAY, not called
            # from non-helper resident code).
            bundleable_helpers = []
            for hname, (hstart, hend, hlines) in list(helper_bodies.items()):
                if f'{hname}:' in _NO_OVERLAY:
                    continue
                called_from_resident = False
                for ci, cline in enumerate(new_resident):
                    if ci >= hstart and ci < hend:
                        continue
                    # Skip helper bodies that are already confirmed bundleable
                    # (removed from _NO_OVERLAY). But scan bodies of helpers still
                    # in _NO_OVERLAY — if they call us, we must stay resident.
                    in_removed_helper = False
                    for oh, (ohs, ohe, _) in helper_bodies.items():
                        if oh != hname and ohs <= ci < ohe:
                            if f'{oh}:' not in _NO_OVERLAY:
                                in_removed_helper = True
                                break
                    if in_removed_helper:
                        continue
                    cs = cline.strip()
                    if f'jal {hname}' in cs or f'j {hname}' in cs:
                        called_from_resident = True
                        break
                if not called_from_resident:
                    bundleable_helpers.append(hname)

            # Build dependency graph for transitive closure.
            deps = {h: set() for h in bundleable_helpers}
            for hname in bundleable_helpers:
                _, _, hlines = helper_bodies[hname]
                for hl in hlines:
                    hs2 = hl.strip()
                    if hs2.startswith('jal '):
                        target = hs2.split()[1]
                        if target in deps:
                            deps[hname].add(target)

            # Compute transitive closure: all helpers reachable from h
            def transitive_deps(h, seen=None):
                if seen is None:
                    seen = set()
                for d in deps.get(h, set()):
                    if d not in seen:
                        seen.add(d)
                        transitive_deps(d, seen)
                return seen

            # ── Knapsack: decide which bundleable helpers should stay resident ──
            # Count how many overlay slots need each helper (transitively).
            # For helpers needed by N≥2 overlays, keeping resident saves (N-1)×size
            # in overlay storage at a cost of +size in the kernel.
            helper_usage_count = {h: 0 for h in bundleable_helpers}
            for oi, (oname, olines, ofsize) in enumerate(overlay_asm_blocks):
                needed = set()
                for ol in olines:
                    os2 = ol.strip()
                    if os2.startswith('jal '):
                        target = os2.split()[1]
                        if target in helper_usage_count:
                            needed.add(target)
                all_needed_k = set(needed)
                for h in needed:
                    all_needed_k |= transitive_deps(h)
                for h in all_needed_k:
                    helper_usage_count[h] = helper_usage_count.get(h, 0) + 1

            # Knapsack: for each helper, value = (N-1) × size (bytes saved from
            # not duplicating). Keep resident if value > weight (size).
            # Estimate kernel: resident code MINUS bundleable helpers + ~40B loader overhead
            bundleable_ranges = set()
            for hname_k in bundleable_helpers:
                hs_k, he_k, _ = helper_bodies[hname_k]
                bundleable_ranges.update(range(hs_k, he_k))
            non_bundleable_lines = [l for i, l in enumerate(new_resident) if i not in bundleable_ranges]
            est_kernel_base = measure_lines(non_bundleable_lines) + 40

            import sys
            print(f"  Knapsack: {len(bundleable_helpers)} bundleable, usage={dict(helper_usage_count)}, est_kernel={est_kernel_base}B", file=sys.stderr)
            knapsack_candidates = []
            for hname in bundleable_helpers:
                hsize = measure_lines(helper_bodies[hname][2]) + 1  # +1 for label
                n_users = helper_usage_count.get(hname, 0)
                if n_users >= 2:
                    value = (n_users - 1) * hsize
                    knapsack_candidates.append((value, hsize, hname))

            # Sort by value descending (most savings first)
            knapsack_candidates.sort(reverse=True)
            keep_resident = set()
            est_kernel_growth = 0
            for value, hsize, hname in knapsack_candidates:
                # Only keep resident if the savings outweigh the kernel growth
                # and the kernel doesn't grow too much (leave at least 80B overlay region)
                est_kernel = est_kernel_base + est_kernel_growth + hsize
                est_overlay_region = 250 - est_kernel
                if est_overlay_region >= 40 and value >= hsize:
                    keep_resident.add(hname)
                    est_kernel_growth += hsize
                    # Re-add to _NO_OVERLAY
                    _NO_OVERLAY.add(f'{hname}:')
                    import sys
                    print(f"  Knapsack: keeping {hname} resident ({hsize}B, used by {helper_usage_count[hname]} overlays, saves {value}B)",
                          file=sys.stderr)

            # ── Phase 6: Wrapping-aware helper promotion ──
            # Estimate post-bundling overlay sizes. If any would wrap (exceed the
            # estimated overlay region), force the most-shared bundled helper resident.
            # This reduces overlay sizes at the cost of kernel growth.
            est_overlay_avail = 250 - (est_kernel_base + est_kernel_growth)
            for _wrap_retry in range(3):
                # Estimate post-bundling size for each overlay
                wrapping_overlays = []
                for oi, (oname, olines, ofsize) in enumerate(overlay_asm_blocks):
                    # Estimate bundled size: function size + sum of all bundleable helpers it needs
                    needed_helpers = set()
                    for ol in olines:
                        os2 = ol.strip()
                        if os2.startswith('jal '):
                            target = os2.split()[1]
                            if target in bundleable_helpers:
                                needed_helpers.add(target)
                                needed_helpers |= transitive_deps(target)
                    bundled_size = ofsize + sum(
                        measure_lines(helper_bodies[h][2]) + 1 for h in needed_helpers
                        if h in helper_bodies and h not in keep_resident)
                    if bundled_size > est_overlay_avail:
                        wrapping_overlays.append((oi, oname, bundled_size, needed_helpers - keep_resident))
                if not wrapping_overlays:
                    break
                # Find the bundled helper that, if promoted, would reduce the most wrapping
                helper_impact = {}
                for _, _, bsize, helpers in wrapping_overlays:
                    for h in helpers:
                        if h in helper_bodies:
                            hsize = measure_lines(helper_bodies[h][2]) + 1
                            helper_impact[h] = helper_impact.get(h, 0) + hsize
                if not helper_impact:
                    break
                best = max(helper_impact, key=helper_impact.get)
                best_size = measure_lines(helper_bodies[best][2]) + 1
                # Check that promoting this helper doesn't make kernel too large
                new_kernel = est_kernel_base + est_kernel_growth + best_size
                if new_kernel > 230:  # leave at least 20B overlay region
                    break
                keep_resident.add(best)
                est_kernel_growth += best_size
                est_overlay_avail = 250 - (est_kernel_base + est_kernel_growth)
                _NO_OVERLAY.add(f'{best}:')
                import sys
                print(f"  Phase 6: promoting {best} ({best_size}B) to resident (reduces wrapping by {helper_impact[best]}B)",
                      file=sys.stderr)

            # Remove kept-resident helpers from bundleable list.
            # Identify helpers that must be resident because the kernel
            # (including its helpers) references them. Two patterns:
            #   1. Fall-through merge: resident helper body contains another
            #      helper's label (__foo:) — the sub-helper is merged in.
            #   2. Jump reference: resident helper does `j __bar` — __bar
            #      is not a call (no ret boundary) so __bar MUST be next
            #      in the kernel's code layout OR reachable as a global.
            # Both must be pulled into the kernel.
            resident_helpers_for_merge = set(keep_resident)
            for label in _NO_OVERLAY:
                if label.endswith(':'):
                    hn = label[:-1]
                    if hn.startswith('__') and hn in helper_bodies:
                        resident_helpers_for_merge.add(hn)
            merged_into_resident = set()
            # Iteratively close the "pulled into kernel" set — if A is
            # resident and refs __B (via fall-through or j/jal), __B is too,
            # and then __B's references also propagate.
            frontier = set(resident_helpers_for_merge)
            while frontier:
                next_frontier = set()
                for hname in frontier:
                    if hname not in helper_bodies:
                        continue
                    _, _, hlines = helper_bodies[hname]
                    for hl in hlines:
                        hs = hl.strip()
                        # Fall-through: label inside the helper body
                        if hs.endswith(':') and hs.startswith('__') and not hs.startswith('.'):
                            sub = hs[:-1]
                            if sub != hname and sub in helper_bodies \
                                    and sub not in merged_into_resident \
                                    and sub not in resident_helpers_for_merge:
                                merged_into_resident.add(sub)
                                next_frontier.add(sub)
                        # `j __foo` / `jal __foo` to a helper — pull it along
                        for prefix in ('j __', 'jal __'):
                            if hs.startswith(prefix):
                                tgt = hs.split()[1]
                                if tgt in helper_bodies \
                                        and tgt != hname \
                                        and tgt not in merged_into_resident \
                                        and tgt not in resident_helpers_for_merge:
                                    merged_into_resident.add(tgt)
                                    next_frontier.add(tgt)
                frontier = next_frontier
            # Remove both kept-resident AND merged-sub helpers from bundleable.
            # Merged sub-helpers are part of the resident helper's body.
            remove_from_bundleable = keep_resident | merged_into_resident
            if remove_from_bundleable:
                bundleable_helpers = [h for h in bundleable_helpers if h not in remove_from_bundleable]
                for h in remove_from_bundleable:
                    deps.pop(h, None)
                for h in deps:
                    deps[h] -= remove_from_bundleable

            # For each overlay, find all bundleable helpers it needs (transitively).
            # Then append them all. Order: leaves first (deps before dependents).
            for oi, (oname, olines, ofsize) in enumerate(overlay_asm_blocks):
                # Find directly-called bundleable helpers
                needed = set()
                for ol in olines:
                    os2 = ol.strip()
                    if os2.startswith('jal '):
                        target = os2.split()[1]
                        if target in bundleable_helpers:
                            needed.add(target)
                # Add transitive deps
                all_needed = set(needed)
                for h in needed:
                    all_needed |= transitive_deps(h)

                if not all_needed:
                    continue

                # Topological sort for this overlay's needed helpers (leaves first)
                sorted_for_ov = []
                vis = set()
                def topo(h):
                    if h in vis or h not in all_needed:
                        return
                    vis.add(h)
                    for d in deps.get(h, set()):
                        topo(d)
                    sorted_for_ov.append(h)
                for h in all_needed:
                    topo(h)

                # Append helper bodies in dependency order.
                # Rename all labels with _ovN suffix to avoid duplicate symbols
                # across overlays (each overlay gets its own copy at org OVERLAY_REGION).
                suffix = f'_ov{oi}'

                # Collect ALL __ labels defined in bundled helper bodies (including
                # internal labels from fall-through merging, e.g., __lcd_send_raw
                # inside __lcd_cmd)
                all_bundled_labels = set(all_needed)
                for hname in sorted_for_ov:
                    hstart_b, _, hlines_b = helper_bodies[hname]
                    for hl_b in [new_resident[hstart_b]] + hlines_b:
                        s_b = hl_b.strip()
                        if s_b.endswith(':') and s_b.startswith('__'):
                            all_bundled_labels.add(s_b[:-1])

                new_olines = list(olines)

                # First rename all bundled label references in the overlay's OWN code
                for li in range(len(new_olines)):
                    for lbl in all_bundled_labels:
                        s_check = new_olines[li]
                        # Rename jal references
                        s_check = s_check.replace(f'jal {lbl}', f'jal {lbl}{suffix}')
                        # Rename j references (but not jal, jnz, jz, jc, jnc)
                        if f'j {lbl}' in s_check and f'jal {lbl}' not in s_check:
                            for jtype in ('jnz', 'jz', 'jc', 'jnc'):
                                if f'{jtype} {lbl}' in s_check:
                                    break
                            else:
                                s_check = s_check.replace(f'j {lbl}', f'j {lbl}{suffix}')
                        new_olines[li] = s_check

                # Append renamed helper bodies
                for hname in sorted_for_ov:
                    hstart, _, hlines = helper_bodies[hname]
                    helper_full = [new_resident[hstart]] + hlines
                    for hl in helper_full:
                        l = hl
                        # Rename all bundled labels (definitions and references)
                        for lbl in all_bundled_labels:
                            l = l.replace(f'{lbl}:', f'{lbl}{suffix}:')
                            l = l.replace(f'jal {lbl}', f'jal {lbl}{suffix}')
                            if f'j {lbl}' in l and f'jal {lbl}' not in l:
                                for jtype in ('jnz', 'jz', 'jc', 'jnc'):
                                    if f'{jtype} {lbl}' in l:
                                        break
                                else:
                                    l = l.replace(f'j {lbl}', f'j {lbl}{suffix}')
                        # Rename local labels to avoid conflicts between overlays
                        import re as _re
                        for m in _re.finditer(r'\.([\w]+)', l):
                            ref = '.' + m.group(1)
                            if any(ref + ':' in h2.strip() for h2 in [new_resident[hstart]] + hlines):
                                l = l.replace(ref, ref + suffix)
                        new_olines.append(l)
                new_fsize = measure_lines(new_olines)
                overlay_asm_blocks[oi] = (oname, new_olines, new_fsize)

            # Remove all bundled helpers from resident.
            # Use ORIGINAL ranges (pre-merge) to avoid accidentally removing
            # resident helpers that were merged into a bundled helper's body.
            for hname in bundleable_helpers:
                orig_start, orig_end = helper_original_ranges.get(hname, (0, 0))
                for ri in range(orig_start, orig_end):
                    new_resident[ri] = ''

            new_resident = [l for l in new_resident if l != '']

            # ── Phase 6: Procedure abstraction (within-overlay dedup) ──
            # For each overlay, find repeated contiguous instruction sequences
            # and extract them as local thunks. A pattern of N occurrences of
            # S bytes becomes 1 thunk (S+1 bytes) + N jal calls (2 bytes each).
            # Profitable when (N-1) * S > 2*N + 1, i.e., S ≥ 4 AND N ≥ 2.
            # Thunks live at the tail of the overlay so they're loaded together.
            import mk1ir as _ir_p6
            def _phase6_byte_size(line):
                return _ir_p6.instr_byte_size(line, two_byte)

            def _phase6_is_extractable(instr):
                return _ir_p6.is_locally_extractable(instr)

            def _phase6_seq_stack_balanced(seq_texts):
                return _ir_p6.seq_stack_balanced(seq_texts)

            def _phase6_extract(olines, oi_tag):
                """Find + extract the best repeated sequence. Returns (new_olines,
                num_thunks_added)."""
                # Index instructions
                instr_idx = [i for i, l in enumerate(olines)
                             if l.strip() and not l.strip().startswith(';')
                             and not l.strip().endswith(':')
                             and not l.strip().startswith('section')
                             and not l.strip().startswith('org')]
                texts = [olines[i].strip() for i in instr_idx]
                if len(texts) < 8:
                    return olines, 0
                # Build hash index
                MIN_LEN, MAX_LEN = 4, 10
                seqs = {}
                for i in range(len(texts)):
                    for L in range(MIN_LEN, min(MAX_LEN, len(texts) - i) + 1):
                        ok = True
                        for k in range(L):
                            if not _phase6_is_extractable(texts[i + k]):
                                ok = False; break
                        if not ok:
                            continue
                        key = tuple(texts[i:i + L])
                        if not _phase6_seq_stack_balanced(key):
                            continue
                        seqs.setdefault(key, []).append(i)
                # Score candidates
                best = None  # (savings, key, non_overlap_positions, L, S)
                for key, poses in seqs.items():
                    if len(poses) < 2:
                        continue
                    L = len(key)
                    # Non-overlapping greedy
                    sp = sorted(poses)
                    accepted = []
                    last_end = -1
                    for p in sp:
                        if p >= last_end:
                            accepted.append(p); last_end = p + L
                    N = len(accepted)
                    if N < 2:
                        continue
                    # Byte size of the sequence
                    S = sum(_phase6_byte_size(olines[instr_idx[accepted[0] + k]])
                            for k in range(L))
                    if S < 4:
                        continue
                    savings = (N - 1) * S - 1 - 2 * N
                    if savings <= 0:
                        continue
                    if best is None or savings > best[0]:
                        best = (savings, key, accepted, L, S)
                if best is None:
                    return olines, 0
                savings, key, accepted, L, S = best
                # Extract
                thunk_name = f'__ovthunk_{oi_tag}'
                thunk_body = [f'{thunk_name}:']
                for k in range(L):
                    thunk_body.append(olines[instr_idx[accepted[0] + k]])
                thunk_body.append('\tret')
                # Build the new overlay: instructions we're replacing become
                # `jal __ovthunk_N`; the rest stay as-is. The thunk itself is
                # appended to the overlay tail.
                replace_set = set()
                for start in accepted:
                    for k in range(L):
                        replace_set.add(instr_idx[start + k])
                call_at = set(instr_idx[start] for start in accepted)
                new_lines = []
                for li, line in enumerate(olines):
                    if li in call_at:
                        new_lines.append(f'\tjal {thunk_name}')
                    elif li in replace_set:
                        continue  # drop — part of the extracted sequence
                    else:
                        new_lines.append(line)
                new_lines.extend(thunk_body)
                import sys
                print(f"  Phase 6 abstract: overlay {oi_tag} — extracted "
                      f"{L}-instr {S}B sequence × {len(accepted)} "
                      f"(saves {savings}B) as {thunk_name}",
                      file=sys.stderr)
                return new_lines, 1

            # Iterate: extract the best thunk, re-scan, repeat until no gains
            _thunk_counter = 0
            for oi, (oname, olines, ofsize) in enumerate(overlay_asm_blocks):
                cur_lines = olines
                for _iter in range(4):  # up to 4 thunks per overlay
                    new_lines, added = _phase6_extract(cur_lines,
                                                        f'{oi}_{_thunk_counter}')
                    if added == 0:
                        break
                    _thunk_counter += 1
                    cur_lines = new_lines
                if cur_lines is not olines:
                    overlay_asm_blocks[oi] = (oname, cur_lines, measure_lines(cur_lines))

            # Rebuild overlay_meta with updated sizes
            overlay_meta = [(idx, name, fsize) for idx, (name, _, fsize) in enumerate(overlay_asm_blocks)]
            # Track post-bundling max for diagnostics (may exceed overlay region due to wrapping)
            max_bundled_size = max((fsize for _, _, fsize in overlay_asm_blocks), default=0)

            # ── Step 9a: Selective overlay-to-overlay reload thunks ──
            # Edges recorded before the post-SCC merge are expensive one-way
            # user calls where merging caller+callee into one slot creates
            # more wrap pressure than an 11B resident reload thunk. The thunk:
            #   load callee slot; call callee; reload caller slot; return.
            # `_overlay_load` preserves $a/$b, so callee args and return value
            # survive the reloads without extra stack traffic.
            if reload_thunk_edges:
                thunk_lines = []
                made_thunks = 0
                for caller, callee in sorted(reload_thunk_edges):
                    caller_idx = func_to_overlay_idx.get(caller)
                    callee_idx = func_to_overlay_idx.get(callee)
                    if caller_idx is None or callee_idx is None or caller_idx == callee_idx:
                        continue
                    safe_caller = caller.lstrip('_').replace('.', '_')
                    safe_callee = callee.lstrip('_').replace('.', '_')
                    thunk = f'__ovreload_{safe_caller}_{safe_callee}'
                    replaced = False
                    oname, olines, _ofsize = overlay_asm_blocks[caller_idx]
                    new_olines = []
                    current_func = None
                    for line in olines:
                        s = line.strip()
                        if s.endswith(':') and not s.startswith('.') and s.startswith('_'):
                            current_func = s[:-1]
                        if current_func == caller and s == f'jal {callee}':
                            new_olines.append(f'\tjal {thunk}')
                            replaced = True
                        else:
                            new_olines.append(line)
                    if not replaced:
                        continue
                    overlay_asm_blocks[caller_idx] = (
                        oname, new_olines, measure_lines(new_olines))
                    thunk_lines.extend([
                        f'{thunk}:',
                        f'\tldi $c,{callee_idx}',
                        '\tjal _overlay_load',
                        f'\tjal {callee}',
                        f'\tldi $c,{caller_idx}',
                        '\tjal _overlay_load',
                        '\tret',
                    ])
                    _NO_OVERLAY.add(f'{thunk}:')
                    made_thunks += 1
                if made_thunks:
                    new_resident.extend(thunk_lines)
                    overlay_meta = [(idx, name, fsize)
                                    for idx, (name, _, fsize)
                                    in enumerate(overlay_asm_blocks)]
                    max_bundled_size = max((fsize for _, _, fsize in overlay_asm_blocks),
                                           default=0)
                    import sys
                    print(f"  Overlay split: inserted {made_thunks} reload thunk(s)",
                          file=sys.stderr)

            # ── Step 9b: Validate overlay self-containment (post-inlining) ──
            # Every jal in an overlay must target a function in the SAME overlay
            # or a resident function. Must run AFTER helper inlining (step 9)
            # so we validate the actual post-inlining code, not pre-inlining.
            resident_labels = set()
            for line in new_resident:
                s = line.strip()
                if s.endswith(':') and not s.startswith('.'):
                    resident_labels.add(s[:-1])
            for idx, (oname, olines, ofsize) in enumerate(overlay_asm_blocks):
                for oline in olines:
                    s = oline.strip()
                    if s.startswith('jal ') or s.startswith('j '):
                        target = s.split()[1]
                        if target.startswith('.'):
                            continue  # local label
                        if target in resident_labels:
                            continue  # resident — OK
                        # Check if target is defined within this overlay's own lines
                        overlay_labels = {ol.strip()[:-1] for ol in olines
                                         if ol.strip().endswith(':') and not ol.strip().startswith('.')}
                        if target in overlay_labels:
                            continue  # same overlay — OK
                        # Target is not reachable — broken!
                        raise Exception(
                            f"overlay '{oname}' calls '{target}' which is not resident "
                            f"or in this overlay. This would execute garbage after inlining. "
                            f"Overlay contains: {list(overlay_labels)}"
                        )

            # ── Step 10: Separate main from other resident code ──
            # In the new architecture, _main body goes into page3_code (kernel image).
            # Init-only helpers + init calls stay in section code (stage 1).
            # We need to extract _main from new_resident.
            main_lines = []
            other_resident = []
            in_main = False
            for line in new_resident:
                s = line.strip()
                if s == '_main:':
                    in_main = True
                    main_lines.append(line)
                    continue
                if in_main:
                    if s.endswith(':') and not s.startswith('.') and s.startswith('_'):
                        in_main = False
                        other_resident.append(line)
                    else:
                        main_lines.append(line)
                else:
                    other_resident.append(line)

            # ── Step 10b: Extract init-only code from main ──
            # Init-only code (VIA init, bus recovery, overlay calls for init functions)
            # moves to stage 1. Runtime code stays in main (kernel).
            INIT_MARKERS = {
                'exw 0 0', 'exw 0 1', 'exw 0 3',
                'push $a ;!keep', 'pop $a ;!keep',
                'jal __lcd_init',
                'push_b', 'pop_b', 'iderefp3',
            }
            # 'exw ' prefix covers user-written VIA init like `exw(val, 2, 0)`
            # (DDR setup) — previously these ended the init phase and caused
            # subsequent lcd_init() calls to stay in runtime_main while their
            # body was placed stage-1-only (section-mismatch error).
            INIT_PREFIXES = ('ddrb_imm', 'exrw 2', 'exw ', 'ldi $d,', 'ddra_imm', 'push_imm')
            # Helpers that are safe to call during init (don't end init phase)
            INIT_COMPAT_HELPERS = {'__i2c_stream', '__i2c_st', '__i2c_sp',
                                   '__i2c_rs',
                                   '__lcd_init', '__tone_setup'}

            # Pre-scan: find overlay_load call sequences
            overlay_call_lines = set()
            for mi, ml in enumerate(main_lines):
                if ml.strip() == 'jal _overlay_load':
                    overlay_call_lines.add(mi)
                    for bi in range(max(0, mi - 8), mi):
                        bs = main_lines[bi].strip()
                        if (bs.startswith('ldi ') or bs.startswith('push_b') or
                            bs == 'pop_b' or bs == 'iderefp3' or
                            bs.startswith('mov ')):
                            overlay_call_lines.add(bi)

            init_extracted = []
            runtime_main = []
            in_init_phase = True
            import os as _iidbg_os
            _iidbg = _iidbg_os.environ.get('MK1_DEBUG_INIT')
            for mi, line in enumerate(main_lines):
                s = line.strip()
                if s == '_main:':
                    continue  # handled separately
                if in_init_phase:
                    # Pure init: VIA init builtins, bus recovery — NO function calls
                    # Overlay calls (jal _overlay_load) must stay in runtime because
                    # overlays need resident kernel functions not yet loaded in stage 1.
                    is_init = (s in INIT_MARKERS or
                               any(s.startswith(p) for p in INIT_PREFIXES) or
                               s.startswith('clr $a') or s.startswith('dec') or
                               s.startswith('.via_') or s.startswith('.br_') or
                               s.startswith('.rcv') or s.startswith('.i2c_init_br') or
                               s.startswith('.__precomp') or
                               s.startswith('.__preskip') or
                               s.startswith('mov $c,$a') or
                               s == 'nop')
                    if (s.startswith('jnz .') or s.startswith('j .via') or
                        s.startswith('j .br') or s.startswith('j .rcv') or
                        s.startswith('j .i2c_init_br') or
                        s.startswith('j .__preskip') or
                        s.startswith('j .__precomp')):
                        is_init = True
                    # ldi before init-compatible jal or exw is also init.
                    # Common pattern: `ldi $a, val; exw E M` for user VIA init.
                    if s.startswith('ldi $a,') or s.startswith('ldi $b,'):
                        for nli in range(mi + 1, min(mi + 4, len(main_lines))):
                            ns = main_lines[nli].strip()
                            if ns.startswith('jal '):
                                t = ns.split()[1]
                                if t in INIT_COMPAT_HELPERS or is_init_only(t):
                                    is_init = True
                                break
                            if ns.startswith('exw ') or ns.startswith('ddrb_imm') or ns.startswith('ddra_imm'):
                                is_init = True
                                break
                    if s.startswith('jal '):
                        target = s.split()[1]
                        if is_init_only(target) or target in INIT_COMPAT_HELPERS:
                            is_init = True
                        else:
                            # Any non-init jal ends the init phase
                            in_init_phase = False
                    if is_init:
                        init_extracted.append(line)
                        if _iidbg:
                            print(f"  [init-dbg] INIT: {s!r}", file=sys.stderr)
                    else:
                        in_init_phase = False
                        runtime_main.append(line)
                        if _iidbg:
                            print(f"  [init-dbg] END-INIT: {s!r}", file=sys.stderr)
                else:
                    runtime_main.append(line)

            # Strip trailing unreachable ret
            while runtime_main and runtime_main[-1].strip() == 'ret':
                runtime_main.pop()
            # Bus recovery at _main start: self-copy and overlay loading use
            # derefp3/deref which toggle control signals (E0=SCL), corrupting
            # the I2C bus. A STOP condition resets the bus to idle before any
            # runtime I2C transactions.
            # Main preamble: always runs, including on reset (which lands at
            # code[0] = `j _main` after self-copy overwrote stage 1).
            #
            # SP init: must re-run on every reset because stage-1's SP init
            # is discarded after self-copy. Without this, a reset re-enters
            # main with SP at whatever the halt trap left it — subsequent
            # push/pop walks the wrong memory. +4B kernel, required for the
            # "reset = PC=0 clean restart" invariant.
            #
            # Use $b (not $a) to stage the SP value: the compiler relies on
            # A=0 at main entry (MK1 CPU resets GPRs to 0) for `peek3(0)` etc.
            # to elide an explicit `clr $a`. Using $a here would clobber that.
            #
            # EEPROM preload: if any overlays are EEPROM-tier, their bytes
            # must be freshly read into page3 overlay slots before main uses
            # them. Done via kernel helper so it re-runs on every reset.
            preamble = [
                '\tldi $b,0xFF',     # SP init via $b to keep $a = 0 (reset state)
                '\tmov $b,$sp',
            ]
            if getattr(self, '_ee_preload_bytes', 0) > 0:
                preamble.append('\tjal __eeprom_preload')
            preamble.extend([
                '\tddrb_imm 0x03',   # bus recovery: SCL low, SDA low
                '\tddrb_imm 0x01',   # SCL high, SDA low
                '\tddrb_imm 0x00',   # SCL high, SDA high (STOP/idle)
            ])
            main_lines = ['_main:'] + preamble + runtime_main
            if getattr(self, '_needs_tone_init', False):
                tone_ddra = self._port_shadow_mask('DDRA') | 0x02
                for _mi, _ml in enumerate(main_lines):
                    if _ml.strip() == 'jal __tone_precompute':
                        main_lines.insert(_mi + 1, f'\tddra_imm 0x{tone_ddra:02X}')
                        break

            # ── Step 11: Generate kernel assembly (overlay_load function) ──
            # This is the runtime overlay loader that lives in the code page.
            # It checks cache, reads manifest from page3, dispatches to the
            # appropriate page's copy loop, restores args, and calls the overlay.

            p3_used = self.page3_alloc
            META_SIZE = 3   # [offset, size] + [page] = 2+1 bytes per overlay
            meta_table_size = len(overlay_meta) * META_SIZE
            note_table_size = 0  # notes now in page 1, not page 3

            # Determine which SRAM pages are actually used by overlays
            # (we'll compute this after placement, but need to generate copy loops
            # only for pages that have data — done below after placement)

            # Placeholder: generate full loader with all 3 page dispatch paths
            # Will trim unused paths after placement

            # The meta_base in page3 is after the kernel image. We'll compute
            # actual positions after determining KERNEL_SIZE.

            # For now, build the kernel + main as assembly lines, measure, then finalize.

            # ── Step 12: Compute KERNEL_SIZE ──
            # kernel = overlay_load function + main body
            # We need to know KERNEL_SIZE to:
            #   a) set OVERLAY_REGION = KERNEL_SIZE
            #   b) know how much to copy in the self-copy routine
            #   c) place manifest and overlay data in page 3

            # First pass: generate kernel loader with placeholder OVERLAY_REGION,
            # measure everything, then do a second pass with correct values.

            # We'll iterate: generate with estimate, measure, regenerate if needed.
            OVERLAY_REGION = 128  # initial estimate, will be refined

            def generate_kernel_loader(overlay_region, meta_base, has_p1, has_p2, has_p3,
                                         p3_count=0, p1_count=0, use_cache=False,
                                         has_eeprom_tier=False, eeprom_addr_hi=0):
                """Generate the overlay loader assembly.
                Layout in page3 after kernel:
                  __manifest: [offset0, size0, offset1, size1, ...]  (2B each)
                  __pages:    [page0, page1, ...]                     (1B each)
                Page values: 3=derefp3, 1=deref, 2=deref2
                Phase 5: __manifest and __pages live in page-2 kstate
                at P2_MANIFEST_BASE; loader reads via deref2.
                EEPROM-backed overlays are preloaded into freed page3 space during init,
                so the runtime loader only needs SRAM copy paths.
                """
                # ── Multi-entry overlay dispatch ──
                # Loader saves caller's $a/$b internally, copies the selected
                # slot, restores $a/$b, and returns. The call site then `jal`s
                # the specific function label inside the loaded slot. This is
                # slightly larger than a tail-dispatch loader, but it is
                # required when one merged overlay slot has multiple external
                # entry points.
                #
                # Calling convention:
                #   - Caller passes overlay index in $c
                #   - Caller's $a/$b are the overlay's args (unchanged)
                #   - Loader preserves $a/$b across the copy and returns to
                #     the dispatch site.
                num_pages = sum([has_p3, has_p1, has_p2])
                loader = [
                    '; ── overlay loader (kernel, multi-entry dispatch) ──',
                    '_overlay_load:',
                    '\tpush $b',                    # save caller's arg2
                    '\tpush $a',                    # save caller's arg1
                    '\tmov $c,$a',                  # A = index (from $c)
                    '\tsll',                        # A = index * 2
                    '\taddi __manifest,$a',          # A = __manifest + index*2
                    # Read manifest entry [offset, size] from page-2 kstate
                    '\tmov $a,$d',                  # D = manifest addr
                    '\tderef2',                     # A = src_offset
                    '\tmov $a,$c',                  # C = src offset
                    '\tmov $d,$a',                  # A = manifest addr
                    '\tinc',                        # A = manifest addr + 1
                    '\tderef2',                     # A = size
                    f'\taddi {overlay_region},$a',  # A = overlay_region + size = end
                    '\tmov $a,$d',                  # D = end addr
                    f'\tldi $b,{overlay_region}',   # B = dest start
                ]

                # Page dispatch: read __pages[index] to determine which deref to use
                if num_pages > 1:
                    # Stash page number on the stack while we do the manifest
                    # lookup (which clobbers $a, $b, $c, $d). Pop back just
                    # before the dispatch. The caller's arg1/arg2 are below
                    # the page-number on the stack — still restored correctly
                    # at function exit.
                    loader = [
                        '; ── overlay loader (kernel, multi-entry dispatch) ──',
                        '_overlay_load:',
                        '\tpush $b',                    # save caller's arg2
                        '\tpush $a',                    # save caller's arg1
                        # Compute __pages[index] once, stash on stack.
                        '\tmov $c,$a',                  # A = index
                        '\taddi __pages,$a',
                        '\tderef2',                     # A = page number (1,2,3) from p2 kstate
                        '\tpush $a',                    # stash page number
                        # Manifest lookup from page-2 kstate
                        '\tmov $c,$a',                  # A = index
                        '\tsll',                        # A = index * 2
                        '\taddi __manifest,$a',
                        '\tmov $a,$d',                  # D = manifest addr
                        '\tderef2',                     # A = src_offset
                        '\tmov $a,$c',                  # C = src offset
                        '\tmov $d,$a',                  # A = manifest addr
                        '\tinc',
                        '\tderef2',                     # A = size
                        f'\taddi {overlay_region},$a',
                        '\tmov $a,$d',                  # D = end addr
                        f'\tldi $b,{overlay_region}',   # B = dest start
                        '\tpop $a',                     # A = page number
                        '\tcmpi 2',
                    ]
                    if has_p2 and has_p3:
                        loader.append('\tjz .copy_p2')  # page == 2
                        loader.append('\tjc .copy_p3')  # page == 3 (>2)
                    elif has_p2:
                        loader.append('\tjz .copy_p2')  # page == 2
                    elif has_p3:
                        loader.append('\tjc .copy_p3')  # page >= 2 means page 3
                    # Fall through to page 1 (default)
                else:
                    # Single page — no dispatch needed.
                    pass

                # Emit copy loops
                copy_pages = []
                if has_p1: copy_pages.append(('.copy_p1', 'deref'))
                if has_p3: copy_pages.append(('.copy_p3', 'derefp3'))
                if has_p2: copy_pages.append(('.copy_p2', 'deref2'))

                for ci, (label, deref_op) in enumerate(copy_pages):
                    is_last = (ci == len(copy_pages) - 1)
                    loader.append(f'{label}:')
                    loader += [
                        '\tmov $c,$a',          # A = src addr
                        f'\t{deref_op}',        # A = page[src]
                        '\tistc_inc',           # code[B] = A; B++
                        # Pre-collapsed incc (was `mov $c,$a; inc; mov $a,$c` = 3B).
                        # The final peephole is skipped in overlay mode (it'd
                        # shift manifest offsets), so we fold it at emission.
                        '\tincc',               # C = C + 1 (src pointer advance)
                        '\tmov $b,$a',          # A = B (dest)
                        '\tcmp $d',             # dest == end?
                        f'\tjnz {label}',
                    ]
                    if not is_last:
                        loader.append('\tj .ov_done')

                # Bus recovery + return.
                # The copy loop uses `deref` which toggles SCL on each
                # iteration, generating a phantom clock edge on the I2C
                # bus. After copying an overlay the PCF8574 LCD backpack
                # is in a confused mid-transaction state. Flush it with
                # 9 SCL clocks (loop, not inline — saves ~27B vs the
                # unrolled version) + STOP.
                loader.append('.ov_done:')
                loader.append('\tldi $a,9')
                loader.append('.ov_br:')
                loader.append('\tddrb_imm 0x02')   # SCL LOW, SDA released
                loader.append('\tddrb_imm 0x00')   # SCL HIGH, SDA released
                loader.append('\tdec')
                loader.append('\tjnz .ov_br')
                loader.append('\tddrb_imm 0x01')   # SDA LOW, SCL HIGH (STOP prep)
                loader.append('\tddrb_imm 0x00')   # SDA HIGH = STOP
                # Restore caller's arg1/arg2 and return. The caller enters the
                # requested function in the loaded overlay slot.
                loader.append('\tpop $a')          # restore caller's arg1
                loader.append('\tpop $b')          # restore caller's arg2
                loader.append('\tret')
                return loader

            # ── Step 13: Two-pass sizing ──
            # Pass 1: optimistic estimate (page3 only → smallest loader)
            # If overlays don't fit in page3, we'll re-estimate with more pages
            loader_pass1 = generate_kernel_loader(OVERLAY_REGION, 0, False, False, True, 2, 0, use_cache=False,
                                                    has_eeprom_tier=False, eeprom_addr_hi=0)
            loader_size = measure_lines(loader_pass1)
            main_size = measure_lines(main_lines)

            # Extract _NO_OVERLAY helpers AND resident user functions from other_resident.
            # These live permanently in the code page kernel.
            runtime_resident_helpers = []
            runtime_helper_names = set()
            for label_s in _NO_OVERLAY:
                name = label_s.rstrip(':')
                if name.startswith('__'):
                    runtime_helper_names.add(name)
            # Also include non-overlayed functions (both user and helper) that were
            # overlay-eligible but not selected for overlay — they stay resident.
            overlay_func_names = {name for name, _, _, _ in overlay_funcs_flat}
            init_only_func_names = {name for name, _, _, _ in init_only_funcs}
            resident_funcs = set()
            for name, _, _, _ in overlay_eligible:
                if name not in overlay_func_names:
                    resident_funcs.add(name)

            all_kernel_names = runtime_helper_names | resident_funcs | merged_into_resident
            if all_kernel_names:
                rh_lines = []
                non_rh_lines = []
                in_helper = False
                for line in other_resident:
                    s = line.strip()
                    if s.endswith(':') and not s.startswith('.') and s.startswith('_'):
                        hname = s[:-1]
                        if hname in all_kernel_names:
                            in_helper = True
                            rh_lines.append(line)
                            continue
                        else:
                            in_helper = False
                    if in_helper:
                        rh_lines.append(line)
                    else:
                        non_rh_lines.append(line)
                runtime_resident_helpers = rh_lines
                other_resident = non_rh_lines

            # Split shared helpers (needed by both init and overlays) from
            # kernel-only helpers. Shared helpers go in the code page tail
            # (above KERNEL_SIZE) to survive self-copy without duplication.
            shared_helper_names = set()
            # Helpers called from init-only code AND from overlays/kernel
            init_only_callers = {'__lcd_init', '__eeprom_rd', '__tone_setup'}
            all_init_jals = set()
            for line in self.code:
                s = line.strip()
                # Collect jal targets from init-compatible code
                if s.startswith('jal __') and any(s.endswith(n) or n in s for n in []):
                    pass  # complex detection not needed — use a simpler heuristic
            # With the bundling system, helpers stay in the kernel (resident)
            # and init code uses renamed _init copies. No separate shared zone needed.
            # (shared_helper_names remains empty)

            shared_helpers_ov = []
            kernel_only_helpers = []
            hi = 0
            while hi < len(runtime_resident_helpers):
                s = runtime_resident_helpers[hi].strip()
                if s.endswith(':') and s.startswith('__') and not s.startswith('.'):
                    fname = s[:-1]
                    start = hi
                    hi += 1
                    while hi < len(runtime_resident_helpers):
                        ns = runtime_resident_helpers[hi].strip()
                        if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                            break
                        hi += 1
                    if fname in shared_helper_names:
                        shared_helpers_ov.extend(runtime_resident_helpers[start:hi])
                    else:
                        kernel_only_helpers.extend(runtime_resident_helpers[start:hi])
                else:
                    kernel_only_helpers.append(runtime_resident_helpers[hi])
                    hi += 1
            runtime_resident_helpers = kernel_only_helpers
            # Pre-peephole: apply peephole to code components before measuring.
            # This ensures measure_lines matches the assembler's actual byte count
            # (the peephole optimizes patterns like ldi+push → push_imm, saving bytes).
            runtime_resident_helpers = peephole(runtime_resident_helpers)
            shared_helpers_ov = peephole(shared_helpers_ov)
            main_lines = peephole(main_lines)
            shared_helper_size = measure_lines(shared_helpers_ov)

            # ── Cross-section procedure abstraction ──
            # Scan runtime_resident_helpers + main for instruction
            # sequences that repeat ≥2 times across different sections. Extract
            # profitable ones as kernel-resident thunks (placed at the tail of
            # runtime_resident_helpers) and rewrite all call sites to `jal`.
            #
            # Do not extract from overlay bodies. Hardware testing with tone
            # overlays showed cross-section thunks/tail-merges in overlay code
            # can return into the wrong stream or otherwise corrupt subsequent
            # overlay execution even when the byte accounting is correct.
            #
            # Savings model: sequence of S bytes occurring K times across sections
            # Cost:    thunk body S + ret(1) = S+1 bytes added to kernel
            # Savings: K occurrences × S bytes inlined, replaced with K × 2-byte jal
            # Net:     (K × S) − (K × 2) − (S + 1) = (K−1)·S − 2K − 1
            #          > 0 when S ≥ 4 AND K ≥ 3, or S ≥ 6 AND K ≥ 2.
            #
            # Constraints:
            #   - sequence stays within one "unit" (helper or overlay body)
            #   - no local `j .lbl` / `jnz .lbl` / etc. (labels are unit-local)
            #   - no `ret` / `hlt` inside body (thunk's own ret ends execution)
            #   - no `ldsp`/`stsp` (jal shifts SP by 2, breaks offsets)
            #   - push/pop balanced within the sequence
            #   - no `jal __ovthunk_N` (those are overlay-local labels)
            #   - no `jal __xsthunk_N` (don't nest our own thunks on first pass)
            import mk1ir as _ir_t2
            def _t2_byte_size(line):
                return _ir_t2.instr_byte_size(line, two_byte)

            def _t2_extractable(text, allow_ret_terminal=False):
                return _ir_t2.is_locally_extractable(
                    text,
                    allow_ret_terminal=allow_ret_terminal,
                    reject_ovthunk_jal=True,
                )

            def _t2_balanced(texts):
                return _ir_t2.seq_stack_balanced(texts)

            def _t2_strip_section_tokens(lines):
                """Yield (global_idx, text) for instruction lines only —
                skipping labels, comments, directives, and blanks."""
                for i, line in enumerate(lines):
                    s = line.strip()
                    if not s or s.endswith(':') or s.startswith(';'):
                        continue
                    if s.startswith('section') or s.startswith('org'):
                        continue
                    yield i, s

            def _t2_find_best_cross_section():
                """Build a unified view of (runtime_resident_helpers + each
                overlay body), find the most-profitable cross-section sequence.
                Returns (budget_improvement, key_tuple, occurrences, L, S) or None.

                Ranking metric is `pre_budget - post_budget` (where budget =
                kernel_size + max_overlay_size), NOT raw byte savings. In
                overlay mode the code-page fit is what matters: a thunk that
                grows the kernel by 10B while shrinking the largest overlay by
                7B is a net loss even if total-bytes-used decreases."""
                # Build per-unit instruction streams with global indices.
                # Include runtime_resident_helpers, every overlay body, AND
                # main_lines (the runtime main code). main runs at kernel
                # addresses post-self-copy, so it can call __xsthunk_N freely.
                units = {}   # unit_id → list of (global_idx, text)
                units['kernel'] = list(_t2_strip_section_tokens(runtime_resident_helpers))
                units['main'] = list(_t2_strip_section_tokens(main_lines))
                # Overlay units deliberately omitted; see safety note above.
                # Stage-1 init code runs AFTER mini-copy but BEFORE self-copy.
                # Mini-copy places every runtime_resident_helper at its kernel
                # address, so init code can call kernel-resident thunks freely.
                # Shared/bundled/init-only helpers are NOT at their kernel
                # addresses yet — they get renamed to `_init` variants at
                # emission time. A sequence extracted from init that jal's to
                # such a helper would point at the wrong address, so init
                # occurrences are filtered out for sequences containing
                # jal-to-non-mini-copied targets (see _seq_init_safe below).
                if init_extracted:
                    units['init'] = list(_t2_strip_section_tokens(init_extracted))

                # Set of helpers that mini-copy places at their kernel address
                # before init runs. These are the ONLY jal targets safe to
                # appear in init-extracted sequences.
                _mini_copied_names = set()
                for line in runtime_resident_helpers:
                    s = line.strip()
                    if s.endswith(':') and s.startswith('__'):
                        _mini_copied_names.add(s[:-1])

                def _seq_init_safe(key):
                    """A sequence is safe to extract from init iff every `jal`
                    it contains targets a label that WILL be at its kernel
                    address before the stage-1 init code executes. That means:
                    already-existing cross-section thunks (always in runtime
                    helpers) OR any label defined in runtime_resident_helpers
                    (the mini-copied set)."""
                    for t in key:
                        if not t.startswith('jal '):
                            continue
                        tgt = t.split(None, 1)[1].strip()
                        if (tgt.startswith('__xsthunk_') or
                            tgt.startswith('__tailmerge_') or
                            tgt.startswith('__param_')):
                            continue
                        if tgt in _mini_copied_names:
                            continue
                        return False
                    return True

                # Current sizes (unchanging across candidates this pass).
                # Kernel = runtime_resident_helpers + main_lines (+ loader,
                # constant across extractions). Track helpers and main
                # separately so the budget simulation can add the thunk in
                # the right place.
                cur_helper_size = measure_lines(runtime_resident_helpers)
                cur_main_size = measure_lines(main_lines)
                cur_kernel_size = cur_helper_size + cur_main_size
                cur_ov_sizes = [sz for _, _, sz in overlay_asm_blocks]
                cur_max_ov = max(cur_ov_sizes, default=0)
                cur_budget = cur_kernel_size + cur_max_ov

                MIN_LEN, MAX_LEN = 4, 12
                # Dev hook: MK1_T2_MIN_LEN lowers MIN_LEN so tests can force
                # init-touching extractions to fire (the init scan is a
                # structural no-op on the current corpus because typical init
                # blocks have no 4+ instruction sequence matching another
                # section). Set to 3 to let the three-instr I2C STOP pattern
                # (`ddrb_imm 0x03/0x01/0x00`) match between init and __i2c_sp.
                import os as _t2_env_os
                _env_min_len = _t2_env_os.environ.get('MK1_T2_MIN_LEN')
                if _env_min_len:
                    try:
                        MIN_LEN = max(2, int(_env_min_len))
                    except ValueError:
                        pass
                # Three sequence flavours scanned in a unified hash:
                #   - "call": flow ends normally; extraction adds ret; call = 2B jal
                #   - "tail": last instruction is `ret`; call = 2B j (no extra ret)
                #   - "param": first instruction is `ldi $a, N`; the N varies
                #     across occurrences. Thunk replaces the ldi with `mov $b,$a`
                #     at entry. Caller emits `ldi $b, N; jal thunk`. Sub-modes:
                #     param_call (+ret), param_tail (sequence ends in ret).
                seqs = {}   # (mode, key_tuple) → [(uid, stream_start, gidx, param_val), …]
                for uid, stream in units.items():
                    texts = [t for (_g, t) in stream]
                    for i in range(len(texts)):
                        for L in range(MIN_LEN, min(MAX_LEN, len(texts) - i) + 1):
                            # Tail-merge candidate: the sequence's last instruction
                            # may be `ret`. Intermediate instructions follow normal
                            # extractability rules.
                            last = texts[i + L - 1]
                            ret_terminal = (last == 'ret')
                            ok = True
                            for k in range(L - 1):   # all but last
                                if not _t2_extractable(texts[i + k]):
                                    ok = False; break
                            if ok and not _t2_extractable(last, allow_ret_terminal=True):
                                ok = False
                            if not ok:
                                continue
                            key = tuple(texts[i:i + L])
                            if not _t2_balanced(key):
                                continue
                            mode = 'tail' if ret_terminal else 'call'
                            seqs.setdefault((mode, key), []).append((uid, i, stream[i][0], None))
                            # Parametric variant: position-0 `ldi $a, N`, rest
                            # identical. Canonicalize by stripping the imm from
                            # position 0. The parameter value is recorded in the
                            # occurrence tuple for use at extraction time.
                            p0 = texts[i]
                            if p0.startswith('ldi $a,') or p0.startswith('ldi $a ,'):
                                try:
                                    imm_s = p0.split(',', 1)[1].strip()
                                    imm_val = int(imm_s, 0)
                                except Exception:
                                    imm_val = None
                                if imm_val is not None:
                                    # Canonical: replace position 0 with a placeholder
                                    param_key = ('<ldi $a, *>',) + tuple(texts[i+1:i+L])
                                    pmode = 'param_tail' if ret_terminal else 'param_call'
                                    seqs.setdefault((pmode, param_key), []).append(
                                        (uid, i, stream[i][0], imm_val))

                # Optional debug for parametric scan — set MK1_DEBUG_T2=1 to
                # see candidate histograms, top parametric opportunities, and
                # near-miss candidates (positive raw savings but rejected by
                # budget check).
                import os as _t2os
                _debug_t2 = _t2os.environ.get('MK1_DEBUG_T2')
                if _debug_t2:
                    mode_counts = {}
                    for (m, k), occ in seqs.items():
                        mode_counts[m] = mode_counts.get(m, 0) + 1
                    param_candidates = [(len(occ), len(k), k, occ)
                                         for (m, k), occ in seqs.items()
                                         if m == 'param_call' and len(occ) >= 2]
                    param_candidates.sort(key=lambda x: (-x[0], -x[1]))
                    print(f"  [T2 debug] seq counts: {mode_counts}", file=sys.stderr)
                    for K, L, k, occ in param_candidates[:3]:
                        pvals = sorted(set(o[3] for o in occ if o[3] is not None))
                        print(f"    param K={K} L={L} vals={pvals} head: {k[:3]}",
                              file=sys.stderr)
                    # Also collect near-miss candidates (positive raw savings but
                    # rejected because budget_improvement < 0) — print after the
                    # best-finder runs below.
                    _t2_near_miss = []
                best = None   # (budget_improvement, raw_savings, mode, key, accepted, L, S)
                for (mode, key), raw_occ in seqs.items():
                    if len(raw_occ) < 2:
                        continue
                    # Do not extract a sequence that ends by setting flags
                    # while the following instruction remains at the call
                    # site. Classic bad case:
                    #   ldi 9; .loop: ddrb; ddrb; dec; jnz .loop
                    # Extracting through `dec` leaves `jnz` behind, so the
                    # branch can spin without re-running the decrement.
                    if mode in ('call', 'param_call'):
                        last_mn = key[-1].split()[0] if key else ''
                        if last_mn in {'cmp', 'cmpi', 'tst', 'dec', 'inc',
                                       'decb', 'decc', 'decd',
                                       'incb', 'incc', 'incd',
                                       'sll', 'slr'}:
                            continue
                    # Init-safety: sequences with jal to non-mini-copied labels
                    # cannot run from stage-1 init. Prune init occurrences; if
                    # that leaves <2 occurrences, drop the candidate.
                    if not _seq_init_safe(key):
                        raw_occ = [o for o in raw_occ if o[0] != 'init']
                        if len(raw_occ) < 2:
                            continue
                    # For parametric modes: only consider if the imm varies
                    # across occurrences. If all occurrences have the same imm,
                    # this is a byte-identical match already covered by call/tail
                    # mode at strictly lower cost (2B call vs 4B param call).
                    if mode.startswith('param_'):
                        param_vals = set(o[3] for o in raw_occ)
                        if len(param_vals) < 2:
                            continue
                    # Accept same-unit and cross-unit equally; budget check below
                    # handles the cost/benefit. Non-overlap within each unit: greedy
                    by_unit = {}
                    for o in raw_occ:
                        by_unit.setdefault(o[0], []).append(o)
                    accepted = []
                    L = len(key)
                    for uid, occs in by_unit.items():
                        occs.sort(key=lambda o: o[1])
                        last_end = -1
                        for o in occs:
                            if o[1] >= last_end:
                                accepted.append(o); last_end = o[1] + L
                    if len(accepted) < 2:
                        continue
                    # Byte size of the sequence — for parametric modes, the
                    # placeholder is a 2B ldi $a, N (same as an actual one);
                    # don't count the placeholder string as free.
                    S = 0
                    for text in key:
                        if text == '<ldi $a, *>':
                            S += 2   # the original was `ldi $a, N` = 2B
                            continue
                        s = text.strip()
                        mn = s.split()[0]
                        if mn == 'cmp':
                            parts = s.split()
                            S += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
                        elif mn == 'ddrb2_imm':
                            S += 3
                        elif mn == 'ddrb3_imm':
                            S += 4
                        elif mn in two_byte:
                            S += 2
                        else:
                            S += 1
                    if S < 4:
                        continue
                    K = len(accepted)
                    # Cost models differ by mode:
                    #   call: thunk = sequence + ret (S+1), each call = jal (2B)
                    #     savings = K*S - K*2 - (S+1) = (K-1)*S - 2K - 1
                    #   tail: thunk = sequence (S, ends with the sequence's own ret),
                    #     each call = j (2B) — caller's flow bypasses the inline body
                    #     savings = K*S - K*2 - S = (K-1)*S - 2K
                    #   param_call: thunk = (mov $b,$a = 1B) + (rest S-2B) + ret (1B) = S
                    #     each call = ldi $b,N (2B) + jal (2B) = 4B
                    #     savings = K*S - 4K - S = (K-1)*S - 4K
                    #   param_tail: thunk = mov $b,$a (1B) + rest S-2 (ending in ret) = S-1
                    #     each call = ldi $b,N + j = 4B
                    #     savings = K*S - 4K - (S-1) = (K-1)*S - 4K + 1
                    if mode == 'call':
                        raw_savings = (K - 1) * S - 2 * K - 1
                        thunk_cost = S + 1
                    elif mode == 'tail':
                        raw_savings = (K - 1) * S - 2 * K
                        thunk_cost = S
                    elif mode == 'param_call':
                        raw_savings = (K - 1) * S - 4 * K
                        thunk_cost = S
                    else:   # param_tail
                        raw_savings = (K - 1) * S - 4 * K + 1
                        thunk_cost = S - 1
                    if raw_savings <= 0:
                        continue
                    # Parametric modes need a size delta per occurrence too:
                    # each inline occurrence was S bytes, each call site is 4B
                    # (vs 2B for non-param). Adjust per-unit size delta.
                    param = mode.startswith('param_')
                    per_call_bytes = 4 if param else 2
                    # Simulate post-extraction budget. Per-occurrence size delta
                    # = S (inline removed) - per_call_bytes (call site emitted).
                    # Thunk is placed in kernel (runtime_resident_helpers); each
                    # occurrence in kernel/main shrinks their respective unit.
                    # Overlays shrink their own body sizes.
                    occ_delta = S - per_call_bytes
                    unit_occ = {}
                    for o in accepted:
                        unit_occ[o[0]] = unit_occ.get(o[0], 0) + 1
                    # All in-kernel units (kernel helpers + main) contribute to
                    # the single kernel_size figure that the budget cares about.
                    kernel_side_occ = unit_occ.get('kernel', 0) + unit_occ.get('main', 0)
                    post_kernel = cur_kernel_size + thunk_cost - kernel_side_occ * occ_delta
                    new_ov_sizes = []
                    for oi, sz in enumerate(cur_ov_sizes):
                        n = unit_occ.get(f'ov{oi}', 0)
                        new_ov_sizes.append(sz - n * occ_delta)
                    post_max_ov = max(new_ov_sizes, default=0)
                    post_budget = post_kernel + post_max_ov
                    # Init-code shrinkage is a separate stage-1 credit: the
                    # thunk body was already billed to the kernel above, so
                    # per-occurrence init savings directly offset that cost.
                    # Stage-1 also fits on the 250B code page, so init bytes
                    # saved ≈ budget bytes saved to a first approximation.
                    init_delta = unit_occ.get('init', 0) * occ_delta
                    budget_improvement = (cur_budget - post_budget) + init_delta
                    # Strict gate: kernel + max_overlay must not grow.
                    # Relaxed gate (when strict fails): accept if raw_savings > 0
                    # AND the post-state still fits the 250B code page. Catches
                    # cross-overlay extractions that shrink a non-largest
                    # overlay — kernel grows a little but total code-page
                    # utilization fits. Shrinking overlays can also be the
                    # difference between fitting all overlays in SRAM vs.
                    # spilling the last one to EEPROM (which is slower and
                    # depends on I2C hardware the simulator doesn't model).
                    LOADER_APPROX = 42
                    post_total = post_kernel + post_max_ov + LOADER_APPROX
                    CODE_PAGE_LIMIT = 250
                    accept = (budget_improvement >= 0) or (
                        raw_savings > 0 and post_total <= CODE_PAGE_LIMIT
                    )
                    if not accept:
                        if _debug_t2 and raw_savings >= 1:
                            _t2_near_miss.append((raw_savings, budget_improvement, K, S, mode, key[:3]))
                        continue
                    score = (budget_improvement, raw_savings)
                    if best is None or score > best[:2]:
                        best = (budget_improvement, raw_savings, mode, key, accepted, L, S)
                if _debug_t2 and _t2_near_miss:
                    _t2_near_miss.sort(reverse=True)
                    print(f"  [T2 debug] near-miss (rejected, raw_savings > 4):", file=sys.stderr)
                    for rs, bi, K, S, mode, head in _t2_near_miss[:3]:
                        print(f"    raw={rs}B budget={bi}B K={K} S={S} mode={mode} head={head}",
                              file=sys.stderr)
                if best is None:
                    return None
                _bi, raw_savings, mode, key, accepted, L, S = best
                return (raw_savings, key, accepted, L, S, mode)

            def _t2_rewrite_unit(lines, call_at_global, replace_at_global, thunk_name,
                                  call_insn='jal', param_at_global=None):
                """Replace lines at replace_at_global by deletion, emit
                `<call_insn> thunk_name` at each call_at_global.
                param_at_global: dict {global_idx → imm_val}; when provided,
                emits `ldi $b, N` BEFORE the call at that global idx (parametric
                modes), making the full call site `ldi $b, N` + jal/j."""
                call_at = set(call_at_global)
                replace_at = set(replace_at_global)
                params = param_at_global or {}
                out = []
                for i, line in enumerate(lines):
                    if i in call_at:
                        if i in params:
                            out.append(f'\tldi $b,{params[i]}')
                        out.append(f'\t{call_insn} {thunk_name}')
                    elif i in replace_at:
                        continue
                    else:
                        out.append(line)
                return out

            _t2_thunk_counter = 0
            _t2_total_saved = 0
            # Budget gate is strict: only accept extractions that don't grow
            # the kernel+max_overlay sum. Attempted "best-effort" relaxation
            # for already-overflow programs was a net loss — the relaxed
            # extractions traded overlay bytes for kernel bytes at ~1:1,
            # moving the fit constraint further from passing. See
            # _t2_find_best_cross_section() for the scoring details.
            for _t2_pass in range(8):   # up to 8 cross-section thunks per program
                best = _t2_find_best_cross_section()
                if best is None:
                    break
                savings, key, accepted, L, S, mode = best
                if mode == 'tail':
                    thunk_name = f'__tailmerge_{_t2_thunk_counter}'
                elif mode in ('param_call', 'param_tail'):
                    thunk_name = f'__param_{_t2_thunk_counter}'
                else:
                    thunk_name = f'__xsthunk_{_t2_thunk_counter}'
                _t2_thunk_counter += 1
                _t2_total_saved += savings

                # Build the thunk body.
                # call mode:       thunk = key lines + ret (caller uses jal)
                # tail mode:       thunk = key lines (last is ret) (caller uses j)
                # param_call mode: thunk = `mov $b,$a` + key[1:] + ret (caller uses jal, with `ldi $b,N`)
                # param_tail mode: thunk = `mov $b,$a` + key[1:] (caller uses j, with `ldi $b,N`)
                thunk_lines = [f'{thunk_name}:']
                if mode.startswith('param_'):
                    # Replace the placeholder with `mov $b,$a` at entry
                    thunk_lines.append('\tmov $b,$a')
                    for t in key[1:]:
                        thunk_lines.append(f'\t{t}')
                else:
                    thunk_lines.extend(f'\t{t}' for t in key)
                if mode in ('call', 'param_call'):
                    thunk_lines.append('\tret')
                call_insn = 'j' if mode in ('tail', 'param_tail') else 'jal'

                # For each unit with occurrences, rewrite: replace L instructions
                # starting at each accepted position with a single `jal thunk_name`.
                # `accepted` is a list of (unit_id, stream_start_idx, global_start_idx).
                # We need to map stream positions back to `lines` indices.
                by_unit = {}
                for o in accepted:
                    by_unit.setdefault(o[0], []).append(o)

                for uid, occs in by_unit.items():
                    # Pick target list by unit
                    if uid == 'kernel':
                        target_lines = runtime_resident_helpers
                    elif uid == 'main':
                        target_lines = main_lines
                    elif uid == 'init':
                        target_lines = init_extracted
                    else:
                        oi = int(uid[2:])
                        target_lines = overlay_asm_blocks[oi][1]

                    # Re-derive stream (same as collection step) for global indexing
                    stream = list(_t2_strip_section_tokens(target_lines))
                    # For each occurrence, collect the global line indices of the
                    # L instructions in the sequence
                    call_at_global = []
                    replace_at_global = []
                    param_at_global = {}
                    for occ in occs:
                        _uid, stream_start, _g0 = occ[0], occ[1], occ[2]
                        param_val = occ[3] if len(occ) > 3 else None
                        for k in range(L):
                            gidx = stream[stream_start + k][0]
                            if k == 0:
                                call_at_global.append(gidx)
                                if param_val is not None:
                                    param_at_global[gidx] = f'0x{param_val:02X}'
                            else:
                                replace_at_global.append(gidx)
                        # Position 0's line gets replaced by the jal/j emission
                        replace_at_global.append(stream[stream_start][0])
                    new_target = _t2_rewrite_unit(target_lines,
                                                   call_at_global,
                                                   replace_at_global,
                                                   thunk_name,
                                                   call_insn=call_insn,
                                                   param_at_global=param_at_global)

                    if uid == 'kernel':
                        runtime_resident_helpers = new_target
                    elif uid == 'main':
                        main_lines = new_target
                    elif uid == 'init':
                        init_extracted = new_target
                    else:
                        overlay_asm_blocks[oi] = (overlay_asm_blocks[oi][0],
                                                   new_target,
                                                   measure_lines(new_target))

                # Append thunk to runtime_resident_helpers (kernel-resident so
                # callable from everywhere)
                runtime_resident_helpers.extend(thunk_lines)

                _tag_map = {
                    'call': 'xs-abstract',
                    'tail': 'tail-merge',
                    'param_call': 'param-abstract',
                    'param_tail': 'param-tail',
                }
                _tag = _tag_map.get(mode, 'xs')
                _sections = set(a[0] for a in accepted)
                _init_flag = ' [INIT-TOUCHES]' if 'init' in _sections else ''
                print(f"  {_tag}: extracted {L}-instr {S}B sequence × "
                      f"{len(accepted)} across "
                      f"{len(_sections)} sections "
                      f"(saves {savings}B) as {thunk_name}{_init_flag}",
                      file=sys.stderr)

            if _t2_total_saved:
                # Re-peephole after edits (thunk body + call sites may enable new
                # peephole patterns); re-measure shared/main just in case.
                runtime_resident_helpers = peephole(runtime_resident_helpers)
                print(f"  Cross-section total savings: {_t2_total_saved}B",
                      file=sys.stderr)

            # ── Liveness-based overlay call-site push/pop elision ──
            # Overlay call sites emit the uniform pattern:
            #   push $b; push $a; ldi $a, IDX; jal _overlay_load
            #   pop $a; pop $b; jal __ov_entry
            # The push/pop pairs are belt-and-suspenders: they preserve caller's
            # $a/$b across the overlay call. But if the caller's next use of $a
            # (or $b) after __ov_entry is an OVERWRITE (ldi / clr / pop / mov),
            # the save was wasted — the register was dead across the call.
            #
            # Also: the overlay function itself may not take all args. A 0-arg
            # overlay doesn't need $a/$b passed; the pop before __ov_entry is
            # wasted. Combined with caller-side liveness, we can drop both
            # push AND pop for dead registers. Savings: 2B per elided pair.
            # Map __xsthunk_N / __param_N / __tailmerge_N to the overlay index
            # they wrap. This lets jal-liveness see through the thunks to the
            # actual overlay's arg count.
            _t3_thunk_to_ov_idx = {}
            for line in runtime_resident_helpers:
                # Lines look like: __xsthunk_0:, then `ldi $a, IDX`, then jal
                pass
            # Build the map by scanning runtime_resident_helpers
            _i = 0
            _rhlp = runtime_resident_helpers
            while _i < len(_rhlp):
                s = _rhlp[_i].strip()
                if (s.endswith(':') and
                    (s.startswith('__xsthunk_') or s.startswith('__param_') or
                     s.startswith('__tailmerge_'))):
                    name = s[:-1]
                    # Look ahead a few lines for `ldi $a, IDX; jal _overlay_load`
                    for _k in range(1, 8):
                        if _i + _k >= len(_rhlp): break
                        ks = _rhlp[_i + _k].strip()
                        if ks.startswith('ldi $a,') and _i + _k + 1 < len(_rhlp):
                            if _rhlp[_i + _k + 1].strip() == 'jal _overlay_load':
                                try:
                                    idx_str = ks.split(',', 1)[1].strip()
                                    _t3_thunk_to_ov_idx[name] = int(idx_str, 0)
                                except Exception:
                                    pass
                                break
                _i += 1

            # Liveness primitives live in mk1ir. These thin wrappers
            # bind in compiler-specific context (func_params, thunk map).
            import mk1ir as _ir_t3

            def _t3_thunk_lookup(tgt):
                return _t3_thunk_to_ov_idx.get(tgt)

            def _t3_thunk_ov_arg_count(tgt):
                idx = _t3_thunk_to_ov_idx.get(tgt)
                if idx is None:
                    return None
                return _t3_overlay_arg_count(idx)

            def _t3_reg_used_by_instr(text, reg):
                return _ir_t3.reg_used_by_instr(
                    text, reg,
                    func_params=self.func_params,
                    thunk_ov_arg_count=_t3_thunk_ov_arg_count,
                )

            def _t3_reg_written_by_instr(text, reg):
                return _ir_t3.reg_written_by_instr(text, reg)

            def _t3_reg_live_after(lines, idx, reg):
                return _ir_t3.reg_live_after(
                    lines, idx, reg,
                    func_params=self.func_params,
                    thunk_ov_arg_count=_t3_thunk_ov_arg_count,
                )

            # Map overlay index → entry_name so we can look up arg count.
            # overlay_meta was rebuilt at line ~3854 after size changes, so it
            # currently reflects the final set of overlay slots.
            _t3_idx_to_name = {idx: name for (idx, name, _sz) in overlay_meta}

            def _t3_overlay_arg_count(idx):
                """Best-effort arg count for overlay index. Returns 2 if
                unknown (conservative — keeps push/pop)."""
                name = _t3_idx_to_name.get(idx)
                if name is None:
                    return 2
                # Strip leading _ (user functions are `_name` in asm)
                bare = name.lstrip('_')
                # Check func_params. For bundled helpers like `__lcd_chr_ov0`
                # or phase-7 splits like `_show_temp_phase2`, func_params
                # may not have the name; return 2 (conservative).
                if bare in self.func_params:
                    return self.func_params[bare]
                # Phase-7 auto-split: name may be `<orig>_phase1/2`
                if bare.endswith('_phase1') or bare.endswith('_phase2'):
                    base = bare.rsplit('_phase', 1)[0]
                    if base in self.func_params:
                        return self.func_params[base]
                return 2   # unknown → conservative

            def _t3_elide_overlay_saves(lines):
                """Find each overlay call pattern and elide push/pop pairs
                whose register is dead across the call AND not needed as an
                arg by the overlay. Returns (new_lines, bytes_saved)."""
                saved = 0
                out = []
                i = 0
                while i < len(lines):
                    # Match: push $b / push $a / ldi $a, IDX / jal _overlay_load /
                    #        pop $a / pop $b / jal __ov_entry
                    if (i + 6 < len(lines) and
                        lines[i].strip() == 'push $b' and
                        lines[i+1].strip() == 'push $a' and
                        lines[i+2].strip().startswith('ldi $a,') and
                        lines[i+3].strip() == 'jal _overlay_load' and
                        lines[i+4].strip() == 'pop $a' and
                        lines[i+5].strip() == 'pop $b' and
                        lines[i+6].strip() == 'jal __ov_entry'):
                        # Extract overlay index from `ldi $a, IDX`
                        idx_str = lines[i+2].strip().split(',', 1)[1].strip()
                        try:
                            ov_idx = int(idx_str, 0)
                        except ValueError:
                            ov_idx = None
                        # Determine if each register is "needed" (arg to
                        # overlay OR live after the whole call-site returns).
                        arg_count = _t3_overlay_arg_count(ov_idx) if ov_idx is not None else 2
                        a_needed_by_overlay = arg_count >= 1
                        b_needed_by_overlay = arg_count >= 2
                        # Caller-side liveness: check AFTER `jal __ov_entry`
                        a_live_after = _t3_reg_live_after(lines, i+6, 'a')
                        b_live_after = _t3_reg_live_after(lines, i+6, 'b')
                        # Elide $a pair if: not an arg AND not live after
                        elide_a = (not a_needed_by_overlay) and (not a_live_after)
                        elide_b = (not b_needed_by_overlay) and (not b_live_after)
                        # Emit pattern with elisions
                        if not elide_b:
                            out.append(lines[i])       # push $b
                        if not elide_a:
                            out.append(lines[i+1])     # push $a
                        out.append(lines[i+2])         # ldi $a, IDX
                        out.append(lines[i+3])         # jal _overlay_load
                        if not elide_a:
                            out.append(lines[i+4])     # pop $a
                        if not elide_b:
                            out.append(lines[i+5])     # pop $b
                        out.append(lines[i+6])         # jal __ov_entry
                        if elide_a: saved += 2         # push $a + pop $a = 2B total
                        if elide_b: saved += 2         # push $b + pop $b = 2B total
                        i += 7
                    else:
                        out.append(lines[i])
                        i += 1
                return out, saved

            # Apply push/pop elision to main_lines, runtime_resident_helpers
            # (which contains __xsthunk_N wrappers that make overlay calls),
            # and every overlay body. The same overlay-call pattern + liveness
            # logic applies — anywhere the uniform pattern appears, we can
            # elide when the caller's flow doesn't need the saved registers.
            main_lines, _t3_main_saved = _t3_elide_overlay_saves(main_lines)
            runtime_resident_helpers, _t3_hlp_saved = _t3_elide_overlay_saves(runtime_resident_helpers)
            _t3_ov_saved = 0
            for oi in range(len(overlay_asm_blocks)):
                oname, olines, ofsize = overlay_asm_blocks[oi]
                new_olines, n = _t3_elide_overlay_saves(olines)
                if n:
                    overlay_asm_blocks[oi] = (oname, new_olines, measure_lines(new_olines))
                    _t3_ov_saved += n
            _t3_saved = _t3_main_saved + _t3_hlp_saved + _t3_ov_saved
            if _t3_saved:
                print(f"  Elided {_t3_saved}B of dead push/pop around overlay calls "
                      f"(main={_t3_main_saved}B, helpers={_t3_hlp_saved}B, "
                      f"overlays={_t3_ov_saved}B)",
                      file=sys.stderr)

            # ── General push/pop pair elision (experimental) ──
            # Finds matching `push $X` / `pop $X` pairs where the register is
            # dead across the pair. Currently disabled because the label-as-
            # jump-target check aborts most candidates in practice (many
            # labels are branch targets in the intervening region). Kept here
            # for future refinement with a more precise reachability check.
            def _t3_elide_generic_push_pop(lines):
                saved = 0
                # Find labels that are jump targets (anything after the start
                # of scan could be jumped to, breaking the locality assumption).
                labels_with_targets = set()
                for ln in lines:
                    s = ln.strip()
                    if (s.startswith(('j ', 'jz ', 'jnz ', 'jc ', 'jnc ', 'jal '))
                        and ' ' in s):
                        tgt = s.split(' ', 1)[1].strip()
                        labels_with_targets.add(tgt)
                # Build a list of indices to potentially remove
                to_remove = set()
                n = len(lines)
                for i, line in enumerate(lines):
                    s = line.strip()
                    # Only match raw `push $X` (no ;!keep comments)
                    if not (s == 'push $a' or s == 'push $b'):
                        continue
                    if ';!keep' in line:
                        continue
                    reg = s.split()[1][1]   # 'a' or 'b'
                    dollar = f'${reg}'
                    # Scan forward finding the matching pop. Track stack depth
                    # starting at 1 (our own push). Each push on same reg adds,
                    # each pop subtracts until depth=0 (pair's pop).
                    depth = 1
                    j = i + 1
                    broke = False
                    saw_stsp = False
                    saw_jump_in = False
                    while j < n and depth > 0:
                        js = lines[j].strip()
                        if ';!keep' in lines[j]:
                            broke = True; break
                        mn = js.split()[0] if js.split() else ''
                        # A label (`foo:` or `.foo:`) — if it's a jump target,
                        # other code could land inside our window and the
                        # stack-balance no longer holds locally.
                        if js.endswith(':') and not js.startswith('section'):
                            lbl = js[:-1]
                            if lbl in labels_with_targets:
                                saw_jump_in = True; break
                            j += 1; continue
                        if mn == 'stsp' or mn == 'ldsp' or mn == 'ldsp_b':
                            saw_stsp = True; break
                        if mn in ('push', 'push_b', 'push_imm'):
                            depth += 1
                        elif mn in ('pop', 'pop_b'):
                            depth -= 1
                            if depth == 0:
                                # Matching pop. Verify it's `pop $X` for same reg.
                                if mn == 'pop' and len(js.split()) > 1 and js.split()[1] == dollar:
                                    # Check liveness of reg after the pop
                                    if not _t3_reg_live_after(lines, j, reg):
                                        # Check reg is also dead BEFORE the pair
                                        # (i.e., the push's reg value wasn't
                                        # "the interesting value" — the pair
                                        # is a no-op). Actually the push
                                        # preserves the reg value around
                                        # clobbering ops between push and pop.
                                        # So we need: is the reg LIVE at any
                                        # point between push and pop where it
                                        # gets READ? If yes, the push was
                                        # preserving something useful that
                                        # the code reads after the pop (we
                                        # already verified dead-after-pop).
                                        # OR reads in between (pops it, uses,
                                        # pushes back? no — we tracked only
                                        # matching pair).
                                        # Simpler: liveness-after-pop is what
                                        # matters. If dead, the push/pop pair
                                        # can't affect observable behaviour.
                                        to_remove.add(i)
                                        to_remove.add(j)
                                        saved += 2
                                break
                        j += 1
                    if broke or saw_stsp or saw_jump_in:
                        continue
                out = [line for idx, line in enumerate(lines) if idx not in to_remove]
                return out, saved

            # Apply generic push/pop elision to all sections
            main_lines, _t3b_main = _t3_elide_generic_push_pop(main_lines)
            runtime_resident_helpers, _t3b_hlp = _t3_elide_generic_push_pop(runtime_resident_helpers)
            _t3b_ov = 0
            for oi in range(len(overlay_asm_blocks)):
                oname, olines, ofsize = overlay_asm_blocks[oi]
                new_olines, n = _t3_elide_generic_push_pop(olines)
                if n:
                    overlay_asm_blocks[oi] = (oname, new_olines, measure_lines(new_olines))
                    _t3b_ov += n
            _t3b_saved = _t3b_main + _t3b_hlp + _t3b_ov
            if _t3b_saved:
                print(f"  Elided {_t3b_saved}B of dead push/pop pairs "
                      f"(main={_t3b_main}B, helpers={_t3b_hlp}B, "
                      f"overlays={_t3b_ov}B)",
                      file=sys.stderr)

            def _elide_redundant_overlay_loads(lines):
                """Remove proven-redundant consecutive loads of the same slot.

                Safe window is deliberately narrow: straight-line code only,
                no branches, and no non-overlay calls between loads. Overlay
                calls themselves do not change the loaded slot because merged
                user call graphs are self-contained inside that slot.
                """
                out = []
                loaded_idx = None
                saved = 0
                i = 0
                while i < len(lines):
                    s = lines[i].strip()
                    if (i + 2 < len(lines)
                            and (s.startswith('ldi $c,') or s.startswith('ldi $a,'))
                            and lines[i + 1].strip() == 'jal _overlay_load'
                            and lines[i + 2].strip().startswith('jal ')):
                        target = lines[i + 2].strip().split()[1]
                        try:
                            idx = int(s.split(',', 1)[1])
                        except ValueError:
                            idx = None
                        if idx is not None and func_to_overlay_idx.get(target) == idx:
                            if loaded_idx == idx:
                                out.append(lines[i + 2])
                                saved += measure_lines([lines[i], lines[i + 1]])
                            else:
                                out.extend([lines[i], lines[i + 1], lines[i + 2]])
                                loaded_idx = idx
                            i += 3
                            continue

                    if s.endswith(':') or s.startswith('j ') or s.startswith('jz ') \
                            or s.startswith('jnz ') or s.startswith('jc ') \
                            or s.startswith('jnc ') or s.startswith('ret') \
                            or s.startswith('hlt'):
                        loaded_idx = None
                    elif s.startswith('jal '):
                        target = s.split()[1]
                        if func_to_overlay_idx.get(target) != loaded_idx:
                            loaded_idx = None
                    out.append(lines[i])
                    i += 1
                return out, saved

            main_lines, _rl_main = _elide_redundant_overlay_loads(main_lines)
            runtime_resident_helpers, _rl_hlp = _elide_redundant_overlay_loads(runtime_resident_helpers)
            _rl_saved = _rl_main + _rl_hlp
            if _rl_saved:
                print(f"  Elided {_rl_saved}B of redundant overlay loads "
                      f"(main={_rl_main}B, helpers={_rl_hlp}B)",
                      file=sys.stderr)

            runtime_helper_size = measure_lines(runtime_resident_helpers)
            main_size = measure_lines(main_lines)
            KERNEL_SIZE_est = loader_size + main_size + runtime_helper_size
            OVERLAY_REGION_est = KERNEL_SIZE_est

            # ── Step 14: Place overlay data in SRAM pages ──
            # Page 3: after kernel image + manifest
            # Page 1: from data_alloc to 255
            # Page 2: from 0 to 195 (stack uses 196-255)

            # Kernel image occupies page3[0..KERNEL_SIZE-1] via page3_code
            # Manifest follows at page3[KERNEL_SIZE..KERNEL_SIZE+meta_table_size-1]
            # Overlay data follows manifest

            # But wait — the kernel image is stored in page3 temporarily (for self-copy).
            # After self-copy, that page3 space is freed. However, the manifest and overlay
            # data need to persist at runtime. So:
            #   page3[0..KERNEL_SIZE-1] = kernel image (freed after init)
            #   page3[p3_used..p3_used+meta_table_size-1] = manifest (persists)
            #   page3 overlay data starts at p3_used + meta_table_size
            #
            # Actually, the kernel image goes via section page3_code with org 0,
            # which uses page3 addresses 0..KERNEL_SIZE-1 in the page3 buffer.
            # The manifest and overlay data must not collide with the kernel image.
            #
            # Key insight: page3_alloc (p3_used) already accounts for app globals etc.
            # The kernel image also uses page3 space (addresses 0..KERNEL_SIZE-1 in
            # the page3_code section). But page3_code addresses are CODE page addresses,
            # not page3 addresses! The assembler maps page3_code PC to code page addresses
            # but stores bytes in the page3 buffer at the same offset.
            #
            # So the kernel image occupies page3 buffer [0..KERNEL_SIZE-1].
            # App globals start at page3[0] too — collision!
            #
            # Resolution: the kernel image uses page3_code which writes to page3 buffer
            # at addresses matching code PC. Since kernel starts at org 0, it uses
            # page3 buffer [0..KERNEL_SIZE-1]. App globals also start at page3[0].
            # This IS a collision.
            #
            # Fix: bump page3_alloc to max(page3_alloc, KERNEL_SIZE) so manifest
            # and overlay data start after the kernel image.
            # Then the manifest meta_base = max(p3_used, KERNEL_SIZE).

            # Shared helpers go into page3 at [KERNEL_SIZE..OVERLAY_REGION-1].
            # This reserves page3 space for them — overlay data must start after.
            # Conservative kernel estimate: the knapsack may keep helpers resident
            # (adding up to ~100B), and multi-page dispatch adds ~20B to the loader.
            # Use a generous estimate to avoid page3 overflow after sizing converges.
            kernel_max_est = KERNEL_SIZE_est + shared_helper_size + 35  # headroom for multi-page dispatch
            meta_base = max(p3_used, kernel_max_est)
            p3_code_offset = meta_base + meta_table_size
            p3_capacity = 256 - p3_code_offset - note_table_size
            # When shared helpers occupy page3, force ALL overlays to page1.
            # Mixing page3+page1 overlays requires multi-page dispatch in the loader
            # (adds ~19B), which shrinks the overlay region and often doesn't fit.
            if shared_helper_size > 0:
                p3_capacity = 0


            p1_code_offset = self.data_alloc
            p1_capacity = 256 - self.data_alloc
            p2_code_offset = 0
            # Page 2 = stack page. Phase 4 of the kernel-state allocator
            # refactor (see WORKLOG.md) re-enabled overlay storage here
            # at offsets 0x00..(P2_OVERLAY_HI), with the page-2 partition
            # ensuring stack and overlays cannot overlap:
            #   0x00..0x7F  overlay storage   (this region)
            #   0x80..0xAF  kernel state      (KERNEL_STATE entries)
            #   0xB0..0xBF  guard band
            #   0xC0..0xFF  reserved stack
            # The previous defensive `p2_capacity = 0` was for a suspected
            # corruption of overlay bytes by stack pushes; the partition
            # now makes that impossible by construction (stack-depth gate
            # in `_analyze_stack_depth` enforces depth ≤ 64 B which keeps
            # SP at or above 0xC0, with the 16-byte guard band catching
            # any near-overflow before kernel state — let alone overlays
            # — could be touched).
            p2_capacity = P2_OVERLAY_BYTES

            p3_overlays = []
            p1_overlays = []
            p2_overlays = []
            ee_overlays = []  # EEPROM overflow tier

            # EEPROM overlay tier: align base to 256B boundary so addr_hi is constant
            # and manifest offset = addr_lo directly.
            ee_overlay_base = (self.eeprom_alloc + 0xFF) & ~0xFF  # align up to 256B
            if ee_overlay_base == 0:
                ee_overlay_base = 0x0100  # minimum: page 1
            ee_overlay_hi = (ee_overlay_base >> 8) & 0xFF
            ee_code_offset = 0  # offset from ee_overlay_base (= addr_lo)
            ee_capacity = 256   # 8-bit offset range

            # Page-concentrating placement: fill one SRAM page before opening the
            # next. Each additional page adds a dispatch arm (~17B) to the overlay
            # loader and shrinks the overlay region, so concentrating on as few
            # pages as possible is a direct win for nearly every program.
            # Priority order: p1 (data, largest & no SP init cost) → p2 (needs SP
            # init, only 196B) → p3 (shared with kernel image, smallest).
            def place_overlay(idx, name, asm_lines, fsize):
                nonlocal p3_code_offset, p3_capacity, p1_code_offset, p1_capacity, p2_code_offset, p2_capacity
                nonlocal ee_code_offset, ee_capacity
                if fsize <= p1_capacity:
                    p1_overlays.append((idx, name, asm_lines, fsize, p1_code_offset))
                    p1_code_offset += fsize; p1_capacity -= fsize
                    return True
                if fsize <= p2_capacity:
                    p2_overlays.append((idx, name, asm_lines, fsize, p2_code_offset))
                    p2_code_offset += fsize; p2_capacity -= fsize
                    return True
                if fsize <= p3_capacity:
                    p3_overlays.append((idx, name, asm_lines, fsize, p3_code_offset))
                    p3_code_offset += fsize; p3_capacity -= fsize
                    return True
                # SRAM full — try EEPROM overflow tier
                if fsize <= ee_capacity:
                    ee_overlays.append((idx, name, asm_lines, fsize, ee_code_offset))
                    ee_code_offset += fsize; ee_capacity -= fsize
                    return True
                return False

            _placement_retries = 0
            # Sort by size descending — largest overlays first for better bin-packing
            sorted_overlays = sorted(enumerate(overlay_asm_blocks),
                                      key=lambda x: x[1][2], reverse=True)
            for idx, (name, asm_lines, fsize) in sorted_overlays:
                placed = place_overlay(idx, name, asm_lines, fsize)
                if not placed:
                    caps = [(p3_capacity, 3), (p1_capacity, 1), (p2_capacity, 2)]
                    placed = False
                    for cap, page in caps:
                        if fsize <= cap:
                            if page == 3:
                                p3_overlays.append((idx, name, asm_lines, fsize, p3_code_offset))
                                p3_code_offset += fsize
                                p3_capacity -= fsize
                            elif page == 1:
                                p1_overlays.append((idx, name, asm_lines, fsize, p1_code_offset))
                                p1_code_offset += fsize
                                p1_capacity -= fsize
                            elif page == 2:
                                p2_overlays.append((idx, name, asm_lines, fsize, p2_code_offset))
                                p2_code_offset += fsize
                                p2_capacity -= fsize
                            placed = True
                            break
                    # Try EEPROM overflow tier
                    if not placed and fsize <= ee_capacity:
                        ee_overlays.append((idx, name, asm_lines, fsize, ee_code_offset))
                        ee_code_offset += fsize
                        ee_capacity -= fsize
                        placed = True
                    if not placed:
                        # Placement failed. First, try to find the largest bundled
                        # helper IN THIS overlay and force it resident to shrink it.
                        biggest_helper = None
                        biggest_size = 0
                        for oline in asm_lines:
                            os = oline.strip()
                            if os.endswith(':') and os.startswith('__') and not os.startswith('.'):
                                hname_check = os[:-1]
                                # Strip overlay suffix to get base name
                                for suffix in [f'_ov{i}' for i in range(len(overlay_asm_blocks))]:
                                    if hname_check.endswith(suffix):
                                        hname_check = hname_check[:-len(suffix)]
                                        break
                                if hname_check in bundleable_helpers:
                                    hsize_check = measure_lines(helper_bodies.get(hname_check, (0, 0, []))[2])
                                    if hsize_check > biggest_size:
                                        biggest_size = hsize_check
                                        biggest_helper = hname_check
                        # If the failing overlay has no bundled helpers of its own,
                        # promote the largest globally-bundled helper: this shrinks
                        # OTHER overlays and frees up SRAM/EEPROM space.
                        if biggest_helper is None and bundleable_helpers:
                            for hname_g in bundleable_helpers:
                                hsize_g = measure_lines(helper_bodies.get(hname_g, (0, 0, []))[2])
                                if hsize_g > biggest_size:
                                    biggest_size = hsize_g
                                    biggest_helper = hname_g
                        if biggest_helper and _placement_retries < 5:
                            import sys
                            print(f"  Placement retry: making {biggest_helper} ({biggest_size}B) resident to fit {name} ({fsize}B)",
                                  file=sys.stderr)
                            _NO_OVERLAY.add(f'{biggest_helper}:')
                            bundleable_helpers = [h for h in bundleable_helpers if h != biggest_helper]
                            _placement_retries += 1
                            _retry_bundling = True
                            break  # break out of placement loop to retry
                        raise Exception(
                            f"overlay '{name}' ({fsize}B) exceeds ALL storage. "
                            f"p3={p3_capacity}B, p1={p1_capacity}B, p2={p2_capacity}B, "
                            f"eeprom={ee_capacity}B free"
                        )

            # Post-placement wrap safety: an overlay that exceeds the overlay
            # region WRAPS into kernel space when copied. That is never a
            # valid placement; even "loaded last" relies on kernel corruption
            # not being observed, which is not a durable compiler contract.
            # Recompute kernel size with ACTUAL page dispatch flags so the
            # available overlay region matches what step 15 will finalize.
            _has_p1_chk = len(p1_overlays) > 0
            _has_p2_chk = len(p2_overlays) > 0
            _has_p3_chk = (len(p3_overlays) > 0) or (len(ee_overlays) > 0)
            _loader_chk = generate_kernel_loader(
                OVERLAY_REGION_est, 0,
                _has_p1_chk, _has_p2_chk, _has_p3_chk,
                len(p3_overlays) + len(ee_overlays), len(p1_overlays),
                use_cache=False,
                has_eeprom_tier=(len(ee_overlays) > 0),
                eeprom_addr_hi=ee_overlay_hi)
            _kernel_chk = measure_lines(_loader_chk) + main_size + runtime_helper_size
            _ov_avail = 250 - _kernel_chk
            _wrapping_indices = {i for i, (_, _, sz) in enumerate(overlay_asm_blocks)
                                 if sz > _ov_avail}
            # Determine call order from main_lines. Multi-entry dispatch puts
            # the overlay index in $c (ldi $c,N) right before jal _overlay_load.
            # (Older form used $a; tolerated as fallback.)
            _call_order = []
            for _mli in range(len(main_lines) - 1):
                _s = main_lines[_mli].strip()
                _ns = main_lines[_mli + 1].strip()
                if _ns != 'jal _overlay_load':
                    continue
                if _s.startswith('ldi $c,') or _s.startswith('ldi $a,'):
                    try:
                        _call_order.append(int(_s.split(',')[1]))
                    except ValueError:
                        pass
            _unsafe_wrap = False
            if _wrapping_indices and _call_order:
                _last_call = _call_order[-1]
                for _wi in _wrapping_indices:
                    # Keep the old unsafe classification for diagnostics, but
                    # any wrap below is treated as a placement failure.
                    if _wi != _last_call or _call_order.count(_wi) > 1:
                        _unsafe_wrap = True
                        break
            elif len(_wrapping_indices) > 1:
                _unsafe_wrap = True
            if _wrapping_indices and _placement_retries < 5 and bundleable_helpers:
                # Find the helper that, if promoted, best reduces the number
                # of wrapping overlays. Reject promotions that would
                # make the overlay region too small to hold any current overlay
                # (net loss — promoting eats region faster than it shrinks ovs).
                #
                # Safety invariant (no wrap ever emitted): the terminal
                # no-wrap check immediately below (the `raise` at line ~6488)
                # runs after every retry and hard-fails if any overlay still
                # wraps. This retry loop can only *try* wrap-reducing
                # promotions; the `proj_region < 16` gate caps each step's
                # overlay-region shrink, and `_placement_retries < 5` bounds
                # the chain length. If the chain exhausts without reaching
                # zero wraps, the terminal raise fires and compilation
                # fails — no wrapping overlay is ever emitted.
                #
                # (Earlier a magic `_kernel_cap = 210` rejected promotions
                # that would grow the kernel past 210B. Under the hard
                # no-wrap policy the alternative is compile failure, which
                # is strictly worse than a larger kernel that still fits
                # every overlay. The cap is now the implicit ceiling from
                # `proj_region >= 16` — i.e. kernel <= 234B.)
                best_helper = None
                best_unsafe_count = len(_wrapping_indices)
                for h in bundleable_helpers:
                    hsize_h = measure_lines(helper_bodies.get(h, (0, 0, []))[2]) + 1
                    proj_region = _ov_avail - hsize_h
                    if proj_region < 16:
                        continue
                    proj_wraps = 0
                    for oi, (_, olines, _s) in enumerate(overlay_asm_blocks):
                        has_h = any(ol.strip().startswith(f'{h}_ov') and
                                    ol.strip().endswith(':') for ol in olines)
                        new_sz = _s - (hsize_h if has_h else 0)
                        if new_sz > proj_region:
                            proj_wraps += 1
                    if proj_wraps < best_unsafe_count:
                        best_unsafe_count = proj_wraps
                        best_helper = h
                if best_helper:
                    hsize_h = measure_lines(helper_bodies.get(best_helper, (0, 0, []))[2])
                    import sys
                    print(f"  Wrap-safety retry: {len(_wrapping_indices)} overlays wrap; "
                          f"promoting {best_helper} ({hsize_h}B) resident "
                          f"→ projected {best_unsafe_count} wraps",
                          file=sys.stderr)
                    _NO_OVERLAY.add(f'{best_helper}:')
                    bundleable_helpers = [h for h in bundleable_helpers if h != best_helper]
                    _placement_retries += 1
                    _retry_bundling = True
                    continue  # continue outer while — rebundle & re-place
            if _wrapping_indices:
                details = ', '.join(
                    f"{overlay_asm_blocks[i][0]}={overlay_asm_blocks[i][2]}B"
                    for i in sorted(_wrapping_indices)
                )
                import os as _dbg_os
                if _dbg_os.environ.get('MK1_DUMP_WRAPPING') == '1':
                    import sys as _dbg_sys
                    for _oi, (_on, _olines, _osz) in enumerate(overlay_asm_blocks):
                        print(f"[DUMP] overlay idx={_oi} name={_on} size={_osz}B",
                              file=_dbg_sys.stderr)
                        for _ol in _olines:
                            print(f"[DUMP]   {_ol.rstrip()}", file=_dbg_sys.stderr)
                raise Exception(
                    f"overlay placement would wrap into kernel; refusing codegen. "
                    f"available={_ov_avail}B, wrapping={details}"
                )

        has_p1 = len(p1_overlays) > 0
        has_p2 = len(p2_overlays) > 0
        has_p3 = len(p3_overlays) > 0
        has_eeprom = len(ee_overlays) > 0

        if not has_p1 and not has_p2 and not has_p3 and not has_eeprom:
            return  # nothing placed

        # EEPROM-backed overlays: stored in EEPROM, preloaded into freed page3
        # space during init (after self-copy frees page3[0..KERNEL_SIZE-1]).
        # At runtime they're accessed via derefp3, same as regular page3 overlays.
        # Re-assign EEPROM overlay offsets to page3 addresses in the freed range.
        if has_eeprom:
            # Freed page3 range after self-copy: [0 .. meta_base-1]
            # (kernel image is no longer needed after self-copy copies it to code)
            # Place EEPROM-backed overlays at page3[0..].
            p3_ee_offset = 0
            for ei in range(len(ee_overlays)):
                idx, name, asm_lines, fsize, _ee_offset = ee_overlays[ei]
                ee_overlays[ei] = (idx, name, asm_lines, fsize, p3_ee_offset)
                p3_ee_offset += fsize
            # These are now page3 overlays from the loader's perspective
            p3_overlays = list(ee_overlays) + list(p3_overlays)
            has_p3 = True
            # EEPROM preload runs after self-copy. __i2c_sb is typically
            # resident (mini-copied). __i2c_rb uses _init copy (at init
            # addresses, intact after self-copy). Preload code in self_copy
            # uses _init suffixed labels set below.
            import sys
            print(f"  EEPROM-backed: {len(ee_overlays)} overlays ({p3_ee_offset}B) "
                  f"preloaded from EEPROM 0x{ee_overlay_base:04X} → page3[0..{p3_ee_offset-1}]",
                  file=sys.stderr)

        # ── Step 15: Final kernel sizing ──
        # Generate kernel with only the page copy loops actually needed
        for _pass in range(3):

            loader = generate_kernel_loader(OVERLAY_REGION_est, meta_base,
                                            has_p1, has_p2, has_p3,
                                            len(p3_overlays), len(p1_overlays), use_cache=False,
                                            has_eeprom_tier=has_eeprom, eeprom_addr_hi=ee_overlay_hi)
            loader_size = measure_lines(loader)
            main_size = measure_lines(main_lines)
            KERNEL_SIZE = loader_size + main_size + runtime_helper_size
            OVERLAY_REGION = KERNEL_SIZE + shared_helper_size
            meta_base_new = max(p3_used, OVERLAY_REGION)
            if meta_base_new == meta_base and OVERLAY_REGION == OVERLAY_REGION_est:
                break
            meta_base = meta_base_new
            OVERLAY_REGION_est = OVERLAY_REGION

        # Final regeneration with converged values
        loader = generate_kernel_loader(OVERLAY_REGION, meta_base,
                                        has_p1, has_p2, has_p3,
                                        len(p3_overlays), len(p1_overlays), use_cache=False,
                                        has_eeprom_tier=has_eeprom, eeprom_addr_hi=ee_overlay_hi)
        loader_size = measure_lines(loader)
        KERNEL_SIZE = loader_size + main_size + runtime_helper_size
        OVERLAY_REGION = KERNEL_SIZE + shared_helper_size

        def _resolve_overlay_jal_targets(lines):
            """Overlay body labels live in storage sections, whose assembler
            label addresses are page offsets. Resident code must call the
            runtime address the loader copies the body to instead."""
            out = []
            for line in lines:
                s = line.strip()
                if s.startswith('jal '):
                    target = s.split()[1]
                    off = func_to_overlay_offset.get(target)
                    if off is not None:
                        out.append(f'\tjal {OVERLAY_REGION + off}')
                        continue
                out.append(line)
            return out

        # Fix p3 overlay offsets if meta_base changed from estimate.
        # IMPORTANT: only shift REGULAR p3 overlays, not EEPROM-backed ones.
        # EEPROM overlays are placed at page3[0..] and don't move with meta_base
        # (they sit in the space freed by self-copy of the kernel image).
        final_meta_base = max(p3_used, OVERLAY_REGION)
        final_p3_start = final_meta_base + meta_table_size
        ee_names_for_shift = {name for _, name, _, _, _ in ee_overlays}
        if p3_overlays:
            # First non-EEPROM entry establishes the regular-p3 base offset.
            reg_first = next(((n, off) for _, n, _, _, off in p3_overlays
                              if n not in ee_names_for_shift), None)
            if reg_first is not None and reg_first[1] != final_p3_start:
                delta = final_p3_start - reg_first[1]
                p3_overlays = [(idx, n, a, f,
                                 off if n in ee_names_for_shift else off + delta)
                               for idx, n, a, f, off in p3_overlays]
        meta_base = final_meta_base

        # Validate overlay region
        # Overlay region starts after kernel + shared helpers
        if max_slot_size > 250:  # single overlay can't exceed entire code page
            raise Exception(
                f"largest overlay ({max_slot_size}B) exceeds overlay region "
                f"({252 - OVERLAY_REGION}B). Kernel={KERNEL_SIZE}B "
                f"(loader={loader_size}B + main={main_size}B + helpers={runtime_helper_size}B)"
                f" + shared={shared_helper_size}B"
            )

        # ── Step 16: Generate init code (stage 1) ──
        # Init code extracted from main in step 10b (init_extracted).
        # Stage 1 = init helpers + extracted init code + self-copy + j _main.

        # Init was already extracted in step 10b
        init_code_lines = init_extracted
        runtime_main_lines = main_lines  # already has _main: prefix

        # Recompute with extracted sizes (step 10b already did this via main_lines)
        actual_main_size = measure_lines(runtime_main_lines)
        RESET_STUB = 2
        KERNEL_SIZE = RESET_STUB + loader_size + actual_main_size + runtime_helper_size
        OVERLAY_REGION = KERNEL_SIZE + shared_helper_size
        meta_base = max(p3_used, OVERLAY_REGION)
        final_p3_start = meta_base + meta_table_size
        ee_names_for_shift2 = {name for _, name, _, _, _ in ee_overlays}
        if p3_overlays:
            reg_first2 = next(((n, off) for _, n, _, _, off in p3_overlays
                               if n not in ee_names_for_shift2), None)
            if reg_first2 is not None and reg_first2[1] != final_p3_start:
                delta = final_p3_start - reg_first2[1]
                p3_overlays = [(i, n, a, f,
                                 off if n in ee_names_for_shift2 else off + delta)
                               for i, n, a, f, off in p3_overlays]
        # Regenerate loader with correct OVERLAY_REGION
        loader = generate_kernel_loader(OVERLAY_REGION, meta_base,
                                        has_p1, has_p2, has_p3,
                                        len(p3_overlays), len(p1_overlays), use_cache=False,
                                        has_eeprom_tier=has_eeprom, eeprom_addr_hi=ee_overlay_hi)
        loader_size = measure_lines(loader)
        KERNEL_SIZE = RESET_STUB + loader_size + actual_main_size + runtime_helper_size
        OVERLAY_REGION = KERNEL_SIZE + shared_helper_size
        main_lines = _resolve_overlay_jal_targets(main_lines)
        runtime_main_lines = main_lines
        runtime_resident_helpers = _resolve_overlay_jal_targets(runtime_resident_helpers)
        # Verify kernel sizing
        actual_total = RESET_STUB + measure_lines(loader + runtime_resident_helpers + runtime_main_lines)
        if actual_total != KERNEL_SIZE:
            raise Exception(
                f"KERNEL SIZE MISMATCH: sum components={KERNEL_SIZE} vs measured={actual_total}. "
                f"loader={loader_size}, main={actual_main_size}, helpers={runtime_helper_size}"
            )

        # ── Step 17: Build the self-copy routine ──
        # Copies page3[0..KERNEL_SIZE-1] to code[0..KERNEL_SIZE-1] via derefp3+istc_inc
        self_copy = [
            '; ── self-copy: page3 → code page ──',
            f'\tldi $b,0',                      # B = dest/src = 0
            f'\tldi $d,{OVERLAY_REGION}',        # D = byte count (kernel + shared helpers)
            '.selfcopy_loop:',
            '\tmov $b,$a',                      # A = B (offset for derefp3)
            '\tderefp3',                        # A = page3[B]
            '\tistc_inc',                       # code[B] = A; B++
            # Pre-collapsed: was `mov $d,$a; dec; mov $a,$d` (3B) → `decd` (1B).
            # Stage-1 code isn't reached by the final peephole in overlay mode.
            '\tdecd',                           # D--, sets flags
            '\tjnz .selfcopy_loop',
        ]

        # EEPROM-backed overlays: MK1 reads them from EEPROM at init time.
        # Kept in stage 1 (inline), not moved to kernel helper because:
        # (a) moving to kernel bloats the kernel image past 200B, which
        #     pushes overlay slots out of page 3 (256B total budget)
        # (b) page 3 overlay slots are not written to at runtime, so they
        #     survive reset — preload doesn't NEED to re-run on reset for
        #     correctness (only for robustness against theoretical corruption)
        # Reset reliability is achieved by the SP-init preamble in main,
        # which DOES re-run on reset — sufficient for correctness.
        ee_preload_bytes = sum(fsize for _, _, _, fsize, _ in ee_overlays)
        if ee_preload_bytes > 0:
            ee_hi_byte = (ee_overlay_base >> 8) & 0xFF
            self_copy.extend([
                '; ── EEPROM preload (inline sequential read) ──',
                '\texrw 2',
                '\tddrb_imm 0x01',
                '\tddrb_imm 0x03',
                '\tldi $a,0xAE',
                '\tjal __i2c_sb',
                f'\tldi $a,{ee_hi_byte}',
                '\tjal __i2c_sb',
                '\tclr $a',
                '\tjal __i2c_sb',
                # Inline repeated-START (was STOP+START = 11B; this is 8B).
                # Saves 3B in stage-1 — enough to fit test_eeprom_overlay
                # cleanly. Same edge sequence as __i2c_rs helper without
                # the jal/ret + needing the helper resident.
                '\tddrb_imm 0x02',        # SCL LOW, SDA released
                '\tddrb_imm 0x00',        # SCL rises → idle
                '\tddrb_imm 0x01',        # SDA falls with SCL HIGH → REP START
                '\tddrb_imm 0x03',        # SCL falls → ready for address byte
                '\tldi $a,0xAF',
                '\tjal __i2c_sb',
                f'\tldi $d,{ee_preload_bytes}',
                # $c is the page3 write address counter, starts at 0. CPU
                # reset zeroes all GPRs; the preceding self_copy loop and
                # __i2c_sb calls clobber A/B/D but not C — so C is still 0
                # here without an explicit ldi. Saves 2B in stage-1.
                '.ee_byte:',
                # Sentinel-bit inner loop: B starts at 0x01. Each iteration
                # rotates B left through CF and ORs in the SDA bit. After
                # 8 iterations the sentinel (original bit 0) has been shifted
                # out into CF — `jc` exits. Saves the push/pop counter
                # dance (~12B per call). Relies on sll setting CF post-flash.
                '\tldi $b,0x01',          # sentinel at bit 0
                '.ee_bit:',
                '\tmov $b,$a',
                '\tsll',                  # A = B << 1, CF = old bit 7
                '\tmov $a,$b',            # B = shifted
                '\tjc .ee_byte_end',      # sentinel shifted out → 8 bits done
                '\tddrb_imm 0x00',        # SCL HIGH (release, slave drives SDA)
                '\texrw 0',               # A = port B (bit 0 = SDA)
                '\ttst 0x01',
                '\tjz .ee_bz',
                '\tmov $b,$a',
                '\tori 0x01,$a',          # B |= 1
                '\tmov $a,$b',
                '.ee_bz:',
                '\tddrb_imm 0x02',        # SCL LOW
                '\tj .ee_bit',
                '.ee_byte_end:',
                # B=read_byte (from sentinel loop), C=address counter.
                # Need: A=byte, B=address for iderefp3.
                # Old code used push/pop to save byte across C→B; the two-mov
                # version saves 2B by routing byte through A directly.
                '\tmov $b,$a',            # A = B (byte)
                '\tmov $c,$b',            # B = C (address)
                '\tiderefp3',             # page3[B=addr] = A=byte
                '\tddrb_imm 0x03',
                '\tddrb_imm 0x01',
                '\tddrb_imm 0x03',
                # Pre-collapsed to `incc` / `decd` (would also be caught by
                # the section-aware final peephole on this `code` section).
                '\tincc',
                '\tdecd',
                '\tjnz .ee_byte',
                '\tddrb_imm 0x03',
                '\tddrb_imm 0x01',
                '\tddrb_imm 0x00',
            ])
        self._ee_preload_bytes = 0  # disable kernel-side preload emission
        self_copy.append('\tj 0')                        # jump to kernel at code[0]

        # ── Step 18: Assemble everything ──
        assembled = []

        # Phase 1: Kernel image FIRST — must go to page3[0..KERNEL_SIZE-1]
        # before manifest/overlay data advances page3_size past those addresses.
        assembled.append('\tsection code')
        assembled.append('\torg 0')
        assembled.append('\tsection page3_code')
        # Kernel = j _main + overlay_load + runtime helpers + main
        # The jump at code[0] ensures physical reset (PC=0) lands in _main,
        # not in overlay_load (which would crash without call context).
        assembled.append('\tj _main')
        assembled.extend(loader)
        assembled.extend(runtime_resident_helpers)
        # EEPROM preload helper: reads ee_preload_bytes from EEPROM
        # sequentially and stores them in page3[0..ee_preload_bytes-1] as
        # overlay slots. Called from main's preamble on every entry
        # (including reset). Uses __i2c_sb (resident) for address bytes.
        if getattr(self, '_ee_preload_bytes', 0) > 0:
            _ee_hi_byte = (self._ee_overlay_base >> 8) & 0xFF
            assembled.extend([
                '; ── EEPROM preload (kernel helper; called from main preamble) ──',
                '__eeprom_preload:',
                '\tout_imm 250',   # DEBUG: preload entered
                # Setup: START + write-addr + hi + lo, STOP, RESTART + read-addr
                '\texrw 2',
                '\tddrb_imm 0x01',
                '\tddrb_imm 0x03',
                '\tldi $a,0xAE',
                '\tjal __i2c_sb',
                f'\tldi $a,{_ee_hi_byte}',
                '\tjal __i2c_sb',
                '\tclr $a',
                '\tjal __i2c_sb',
                '\tddrb_imm 0x03',
                '\tddrb_imm 0x01',
                '\tddrb_imm 0x00',
                '\texrw 2',
                '\tddrb_imm 0x01',
                '\tddrb_imm 0x03',
                '\tldi $a,0xAF',
                '\tjal __i2c_sb',
                # Outer: init byte counter + page3 offset
                f'\tldi $d,{self._ee_preload_bytes}',
                '\tldi $c,0',
                '.eepl_byte:',
                # Inner: 8-bit read into $b
                '\tldi $b,0',
                '\tpush_imm 8',
                '.eepl_bit:',
                '\tmov $b,$a',
                '\tsll',
                '\tmov $a,$b',
                '\tddrb_imm 0x00',
                '\texrw 0',
                '\ttst 0x01',
                '\tjz .eepl_bz',
                '\tmov $b,$a',
                '\tori 0x01,$a',
                '\tmov $a,$b',
                '.eepl_bz:',
                '\tddrb_imm 0x02',
                '\tpop $a',
                '\tdec',
                '\tpush $a',
                '\tjnz .eepl_bit',
                '\tpop $a',
                # Store: page3[$c] = $b. Need A=byte, B=offset for iderefp3.
                '\tmov $b,$a',
                '\tpush $a',
                '\tmov $c,$a',
                '\tmov $a,$b',
                '\tpop $a',
                '\tiderefp3',
                # ACK pulse
                '\tddrb_imm 0x03',
                '\tddrb_imm 0x01',
                '\tddrb_imm 0x03',
                # C++
                '\tmov $c,$a',
                '\tinc',
                '\tmov $a,$c',
                # D--
                '\tmov $d,$a',
                '\tdec',
                '\tmov $a,$d',
                '\tjnz .eepl_byte',
                # Final STOP
                '\tddrb_imm 0x03',
                '\tddrb_imm 0x01',
                '\tddrb_imm 0x00',
                '\tret',
            ])
        assembled.extend(runtime_main_lines)
        # Shared helpers in page3 at [KERNEL_SIZE..OVERLAY_REGION-1].
        # These survive self-copy (self-copy copies OVERLAY_REGION bytes).
        # During init, a mini-copy bootstraps them from page3 to code page.
        assembled.extend(shared_helpers_ov)

        # Phase 5: Manifest + pages array live at P2_MANIFEST_BASE in
        # the page-2 kstate region. The C++ assembler in this commit
        # has been extended to honour `org` in raw-data sections so
        # the manifest bytes land at the right offset; the overlay
        # loader was updated in lockstep to use `deref2` for manifest
        # and pages reads. Moving them off page-3 frees the bytes
        # they used to squat in at the tail of the kernel image
        # (15-30 B per program), letting page-3 overlay storage start
        # earlier and giving the partitioner more room.
        assembled.append('\tsection stack')
        assembled.append(f'\torg {P2_MANIFEST_BASE}')
        assembled.append('__manifest:')
        # Overlays ordered: page3 first (includes EEPROM-backed), then page1, then page2.
        # EEPROM-backed overlays were already merged into p3_overlays above.
        all_placed_ordered = list(p3_overlays) + list(p1_overlays) + list(p2_overlays)

        # Rebuild func_to_overlay_idx with new ordering
        # (the call rewriting at step 8 already used the old indices,
        # but the manifest is emitted in the new order. We need to
        # re-map the ldi $a,N values in the assembled code.)
        old_to_new = {}
        for new_idx, (old_idx, name, asm_lines, fsize, offset) in enumerate(all_placed_ordered):
            old_to_new[old_idx] = new_idx

        # Fix up overlay indices in assembled code. Multi-entry dispatch
        # puts the index in $c (ldi $c,N) immediately before the load.
        # (Older convention used $a; kept as fallback for any stale
        # emission path.)
        for ci in range(len(assembled) - 1):
            s = assembled[ci].strip()
            nxt = assembled[ci + 1].strip()
            if nxt != 'jal _overlay_load':
                continue
            for reg in ('$c', '$a'):
                prefix = f'ldi {reg},'
                if s.startswith(prefix):
                    try:
                        old_idx = int(s.split(',')[1])
                        if old_idx in old_to_new:
                            assembled[ci] = f'\tldi {reg},{old_to_new[old_idx]}'
                    except ValueError:
                        pass
                    break

        # Manifest: [offset, size] per overlay (2 bytes each)
        for _, name, asm_lines, fsize, offset in all_placed_ordered:
            assembled.append(f'\tbyte {offset}')
            assembled.append(f'\tbyte {fsize}')

        # Page table: [page] per overlay (1 byte each)
        # page values: 3=page3(derefp3), 1=page1(deref), 2=page2(deref2)
        # EEPROM-backed overlays are preloaded into page3, so they're page 3.
        assembled.append('__pages:')
        p3_set = {name for _, name, _, _, _ in p3_overlays}
        p1_set = {name for _, name, _, _, _ in p1_overlays}
        p2_set = {name for _, name, _, _, _ in p2_overlays}
        for _, name, asm_lines, fsize, offset in all_placed_ordered:
            if name in p3_set:
                assembled.append('\tbyte 3')
            elif name in p1_set:
                assembled.append('\tbyte 1')
            elif name in p2_set:
                assembled.append('\tbyte 2')
            else:
                assembled.append('\tbyte 3')  # fallback: page3

        # __ov_entry: marks the start of the overlay region (address OVERLAY_REGION).
        # Emitted after manifest so it resolves to the correct code address.
        # Resident dispatch calls the requested function label after
        # _overlay_load copies the code.
        assembled.append('\tsection code')
        assembled.append(f'\torg {OVERLAY_REGION}')
        assembled.append('\tsection page3_code')
        assembled.append('__ov_entry:')

        # Phase 3: Overlay code at OVERLAY_REGION addresses into storage pages
        # Skip EEPROM-backed overlays (they go to EEPROM section, not page3_code)
        ee_names = {name for _, name, _, _, _ in ee_overlays} if has_eeprom else set()
        for idx, name, asm_lines, fsize, _ in p3_overlays:
            if name in ee_names:
                continue  # emitted in EEPROM section below
            assembled.append('\tsection code')
            assembled.append(f'\torg {OVERLAY_REGION}')
            assembled.append('\tsection page3_code')
            assembled.extend(asm_lines)

        # Emit globals FIRST in the data section so they occupy the offsets the
        # allocator assigned them (p1[0..data_alloc-1]). Without this, overlay
        # bodies emitted by data_code sections would consume those low bytes
        # and push globals out to the tail of the buffer, breaking every
        # `ld $a,_global` read in the program.
        if p1_overlays and self.data_alloc > 0:
            # Emit page-1 storage in the order the allocator assigned:
            # user globals first, then LCD init data (if any), then padding.
            # Previously the LCD init data was emitted SEPARATELY later in the
            # compile pipeline, which pushed it past data_alloc into a later
            # offset — __lcd_init's ldi $d,{data_base} then read zeros at
            # data[data_base], never hitting the real LCD init bytes.
            assembled.append('\tsection data')
            lcd_init_info = getattr(self, '_lcd_init_data', None)
            note_entries = sorted(getattr(self, '_note_table', []), key=lambda x: x[0])
            note_i = 0
            page1_vars = getattr(self, '_page1_globals_cache', None)
            emitted = 0
            if page1_vars is not None:
                for name, init in page1_vars:
                    assembled.append(f'_{name}:')
                    if isinstance(init, list):
                        for v in init:
                            assembled.append(f'\tbyte {v}')
                            emitted += 1
                    else:
                        assembled.append(f'\tbyte {init}')
                        emitted += 1
            def emit_allocated_until(limit):
                nonlocal emitted, note_i
                while emitted < limit:
                    if note_i < len(note_entries) and note_entries[note_i][0] == emitted:
                        p1_off, ratio, cyc_lo, cyc_hi = note_entries[note_i]
                        assembled.append(f'; note at data offset {p1_off}: ratio={ratio} cyc={cyc_hi}:{cyc_lo}')
                        assembled.append(f'\tbyte {ratio}')
                        assembled.append(f'\tbyte {cyc_lo}')
                        assembled.append(f'\tbyte {cyc_hi}')
                        emitted += 3
                        note_i += 1
                    else:
                        assembled.append('\tbyte 0')
                        emitted += 1
            # Pad up to data_base (where LCD init data is expected) with zeros.
            if lcd_init_info is not None:
                lcd_base, lcd_bytes = lcd_init_info
                emit_allocated_until(lcd_base)
                assembled.append(f'; LCD init data at data[{lcd_base}..{lcd_base+len(lcd_bytes)-1}]')
                for b in lcd_bytes:
                    assembled.append(f'\tbyte {b}')
                    emitted += 1
                # Mark that we've already emitted LCD init data here, so the
                # later emission in compile() can skip.
                self._lcd_init_already_emitted = True
            # Any remaining slots → zero-fill
            emit_allocated_until(self.data_alloc)
            if note_entries:
                self._note_table_already_emitted = True

        for idx, name, asm_lines, fsize, _ in p1_overlays:
            assembled.append('\tsection code')
            assembled.append(f'\torg {OVERLAY_REGION}')
            assembled.append('\tsection data_code')
            assembled.extend(asm_lines)

        for idx, name, asm_lines, fsize, _ in p2_overlays:
            assembled.append('\tsection code')
            assembled.append(f'\torg {OVERLAY_REGION}')
            assembled.append('\tsection stack_code')
            assembled.extend(asm_lines)

        # EEPROM-backed overlay code: stored in EEPROM at ee_overlay_base.
        # At init time, the self_copy preload loop reads these bytes back
        # into page3. MK1 is standalone — no ESP32 runtime role.
        # The ESP32 writes er.eeprom (which this section appends to) to the
        # AT24C32 EEPROM during upload; after that the ESP32 has no role.
        if ee_overlays:
            # Pad from self.eeprom_alloc to ee_overlay_base with zeros so
            # overlays land at the known base address the preload uses.
            pad_count = ee_overlay_base - self.eeprom_alloc
            if pad_count > 0:
                assembled.append('\tsection eeprom')
                assembled.append(f'; pad {pad_count}B from 0x{self.eeprom_alloc:04X} → 0x{ee_overlay_base:04X}')
                for _ in range(pad_count):
                    assembled.append('\tbyte 0')
                # Advance the compiler's eeprom_alloc so downstream eeprom
                # emission (line ~1208) doesn't overlap the overlay region.
                self.eeprom_alloc = ee_overlay_base
            for idx, name, asm_lines, fsize, p3_offset in ee_overlays:
                # Set code PC to OVERLAY_REGION so labels resolve to the
                # runtime address the overlay will execute from after the
                # loader copies it into the overlay slot.
                assembled.append(f'; eeprom overlay: {name} → page3[{p3_offset}] ({fsize}B)')
                assembled.append('\tsection code')
                assembled.append(f'\torg {OVERLAY_REGION}')
                assembled.append('\tsection eeprom')
                assembled.extend(asm_lines)
                self.eeprom_alloc += fsize

        # Phase 4: Emit init code in section code (the actual code page at upload)
        # Order: init calls FIRST (VIA init, jal __delay_cal, etc.), then
        # helper function bodies (called via jal from the init sequence),
        # then note precomputation, then self-copy.
        phase4_start = len(assembled)
        assembled.append('\tsection code')
        assembled.append('\torg 0')

        # SP init: only needed when page2 stores overlay data.
        if has_p2:
            assembled.append('\tldi $a,0xFF')
            assembled.append('\tmov $a,$sp')

        # Mini-copy: copy resident kernel helpers from page3 to their code
        # addresses so init code can call them directly (no _init copies needed).
        # Saves ~30B by eliminating duplicated helper code in init.
        helper_start = RESET_STUB + loader_size  # helpers come after j_main + loader
        mini_copied_helpers = set()
        if runtime_helper_size > 0:
            mc_lbl = self.label('mc')
            assembled.append(f'\tldi $b,{helper_start}')
            assembled.append(f'\tldi $d,{runtime_helper_size}')
            assembled.append(f'{mc_lbl}:')
            assembled.append('\tmov $b,$a')
            assembled.append('\tderefp3')
            assembled.append('\tistc_inc')
            # Pre-collapsed: was `mov $d,$a; dec; mov $a,$d` (3B) → `decd` (1B).
            # The final peephole is skipped for overlay programs (it would
            # shift kernel offsets), so we have to fold the pattern here.
            assembled.append('\tdecd')
            assembled.append(f'\tjnz {mc_lbl}')
            for line in runtime_resident_helpers:
                s = line.strip()
                if s.endswith(':') and s.startswith('__'):
                    mini_copied_helpers.add(s[:-1])

        # delay_calibrate auto-insertion now handled in lcd_init builtin
        # NOTE: init_code_lines emit was moved DOWN (after pre-mc helper
        # placement) so the pre-mc slack budget = helper_start - cur_addr is
        # measured BEFORE init code is emitted. With init code emitted first,
        # cur_addr was inflated by init code size and slack shrank — typical
        # programs got 15B slack and could only fit one tiny init helper,
        # forcing the rest to post-mc with padding cost. Pre-emitting helpers
        # gives ~25-30B slack on a typical LCD program.

        # Init-only helpers (their function bodies, called via jal from above)
        # Rename bundled/shared helper references to _init versions so init code
        # calls its own copies (at init addresses), not the overlay copies
        # (which have labels at OVERLAY_REGION+ addresses).
        # includes both shared helpers AND bundled helpers (which have labels in overlay sections)
        # ALL helpers with labels in page3_code need init-specific copies.
        # This includes: shared helpers (if any), bundled helpers, AND resident
        # kernel helpers (whose labels resolve to kernel addresses that aren't
        # available during init because init code occupies those addresses).
        kernel_helper_names = set()
        for line in runtime_resident_helpers:
            s = line.strip()
            if s.endswith(':') and s.startswith('__'):
                kernel_helper_names.add(s[:-1])
        # Exclude mini-copied helpers — they're available at kernel addresses
        kernel_helper_names -= mini_copied_helpers
        init_rename_set = shared_helper_names | set(bundleable_helpers) | kernel_helper_names
        def _rename_shared_refs(lines):
            """Rename helper labels/refs to _init suffix for init code."""
            result = []
            for line in lines:
                l = line
                for sh_name in init_rename_set:
                    # Replace label definitions: '__i2c_sb:' → '__i2c_sb_init:'
                    l = l.replace(f'{sh_name}:', f'{sh_name}_init:')
                    # Replace jal targets: 'jal __i2c_sb' → 'jal __i2c_sb_init'
                    l = l.replace(f'jal {sh_name}', f'jal {sh_name}_init')
                result.append(l)
            return result

        # All init-only funcs emitted normally — the inline preload doesn't
        # need any of them at code[>= KERNEL_SIZE].
        # CRITICAL: pad the stage-1 code here so init-only helper bodies land
        # AT OR AFTER the mini-copy target range end. Otherwise mini-copy
        # (which writes code[helper_start..helper_start+runtime_helper_size-1]
        # with page3 kernel helpers at boot) overwrites the middle of init-only
        # helpers. When init code does `jal __lcd_init` at runtime, the CPU
        # executes the first few un-overwritten bytes of __lcd_init, then falls
        # into mini-copied garbage. Symptom: __lcd_init "half works" — enters
        # correctly, may fire early out_imm, but can't reach the loop body.
        if init_only_funcs:
            _mc_end = helper_start + runtime_helper_size
            # Measure only bytes in section `code` (stage 1). Page3_code bytes
            # don't contribute to the code-page address space at stage 1.
            def _measure_code_section(lns):
                sz = 0
                sec = 'code'
                for ln in lns:
                    s = ln.strip()
                    if s.startswith('section '):
                        sec = s.split()[1]
                        continue
                    if s.startswith('org '):
                        if sec == 'code':
                            sz = int(s.split()[1])
                        continue
                    if sec != 'code':
                        continue
                    if not s or s.endswith(':') or s.startswith(';'):
                        continue
                    mn = s.split()[0]
                    if mn == 'cmp':
                        parts = s.split()
                        sz += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
                    elif mn == 'ddrb2_imm':
                        sz += 3
                    elif mn == 'ddrb3_imm':
                        sz += 4
                    elif mn in two_byte:
                        sz += 2
                    else:
                        sz += 1
                return sz
            _cur_addr = _measure_code_section(assembled)
            import sys

            # Optimisation: the region [cur_addr..helper_start-1] is BEFORE
            # the mini-copy target range [helper_start..mc_end-1], so init
            # helpers placed there won't be overwritten by mini-copy. Fit as
            # many init helpers as possible into that slack before padding
            # up to mc_end for the rest.
            _pre_mc_budget = helper_start - _cur_addr
            _pre_mc_helpers = []        # (name, start, end, fsize) placed pre-mini-copy
            _post_mc_helpers = []
            if _pre_mc_budget > 0:
                # Pick the SUBSET of init helpers with maximum total bytes
                # that fits in the pre-mini-copy slack. For small N (typical:
                # ≤ 5 init helpers), exhaustive subset enumeration is fine.
                # Falls back to greedy-largest-first for larger N to avoid
                # 2^N blowup. Placement is address-independent (helpers are
                # jal'd by label), so any subset is valid.
                _n_iof = len(init_only_funcs)
                if _n_iof <= 12:
                    _best_subset = None
                    _best_bytes = -1
                    for _mask in range(1 << _n_iof):
                        _sel = [init_only_funcs[i] for i in range(_n_iof) if (_mask >> i) & 1]
                        _total = sum(h[3] for h in _sel)
                        if _total <= _pre_mc_budget and _total > _best_bytes:
                            _best_bytes = _total
                            _best_subset = _sel
                    _pre_mc_helpers = _best_subset or []
                    _post_mc_helpers = [h for h in init_only_funcs if h not in _pre_mc_helpers]
                else:
                    # Fallback: greedy largest-first (better for byte-packing
                    # than smallest-first). O(N log N).
                    _sorted_iof = sorted(init_only_funcs, key=lambda h: -h[3])
                    _remaining = _pre_mc_budget
                    for h in _sorted_iof:
                        if h[3] <= _remaining:
                            _pre_mc_helpers.append(h)
                            _remaining -= h[3]
                        else:
                            _post_mc_helpers.append(h)
            else:
                _post_mc_helpers = list(init_only_funcs)

            if _pre_mc_helpers:
                print(f"  Init-helper pre-mini-copy placement: {len(_pre_mc_helpers)} "
                      f"helper(s) ({sum(h[3] for h in _pre_mc_helpers)}B) fit in "
                      f"{_pre_mc_budget}B slack before mini-copy target — saves "
                      f"equivalent bytes of padding", file=sys.stderr)
                for name, start, end, fsize in _pre_mc_helpers:
                    body = _rename_shared_refs(self.code[start:end])
                    assembled.extend(body)

            # The mini-copy has already overwritten
            # [helper_start.._mc_end) with runtime helper bytes. Init code
            # executes after that copy, so it must not occupy the overwritten
            # interval. Pre-mc helpers are safe because they are only jumped
            # to by label; skip over the copied runtime-helper target before
            # emitting linear init code.
            _cur_addr = _measure_code_section(assembled)
            if _cur_addr < _mc_end:
                _pad_needed = _mc_end - _cur_addr
                print(f"  Init-code placement: jumping over mini-copy target "
                      f"({_cur_addr}->{_mc_end}) before init code",
                      file=sys.stderr)
                assembled.append('\tj __init_after_mc')
                if _pad_needed > 2:
                    assembled.append('__init_mc_pad:')
                    for _ in range(_pad_needed - 2):
                        assembled.append('\tnop')
                assembled.append('__init_after_mc:')

            # Emit init code after the mini-copy target. It may call pre-mc
            # helper bodies by label, but it will not be overwritten before
            # execution.
            assembled.extend(init_code_lines)
            if init_only_funcs:
                assembled.append('\tj __selfcopy')

            # Recompute cur_addr after init code + j __selfcopy emitted
            _cur_addr = _measure_code_section(assembled)

            if _post_mc_helpers and _cur_addr < _mc_end:
                _pad_needed = _mc_end - _cur_addr
                print(f"  Init-helper placement: padding {_pad_needed}B so init helpers"
                      f" land past mini-copy target ({_mc_end}B, currently at {_cur_addr})", file=sys.stderr)
                # Label prevents peephole dead-code elimination of the padding —
                # the nops are semantically load-bearing (they push init
                # helpers past the mini-copy target range so mini-copy doesn't
                # overwrite them mid-execution).
                assembled.append('__mc_pad:')
                for _ in range(_pad_needed):
                    assembled.append('\tnop')
        else:
            # No init-only helpers → no pre-mc / post-mc dance, just emit
            # init code straight after mini-copy. Init code is unconditional;
            # the reorder block above only runs when init helpers exist.
            assembled.extend(init_code_lines)

        _deferred_init_only_bodies = []
        # Emit the init helpers that didn't fit pre-mini-copy
        _iof_to_emit = _post_mc_helpers if init_only_funcs else []
        for name, start, end, fsize in _iof_to_emit:
            body = _rename_shared_refs(self.code[start:end])
            assembled.extend(body)

        # Emit init-specific copies of shared helpers (with _init suffix).
        if shared_helpers_ov:
            init_shared = []
            for line in shared_helpers_ov:
                l = line
                for sh_name in init_rename_set:
                    l = l.replace(f'{sh_name}:', f'{sh_name}_init:')
                    l = l.replace(f'jal {sh_name}', f'jal {sh_name}_init')
                if l.strip().startswith('.') and l.strip().endswith(':'):
                    lbl = l.strip()[:-1]
                    l = l.replace(lbl, lbl + '_i')
                else:
                    import re as _re
                    for m in _re.finditer(r'\.([\w]+)', l):
                        ref = '.' + m.group(1)
                        if any(ref + ':' in sh.strip() for sh in shared_helpers_ov):
                            l = l.replace(ref, ref + '_i')
                init_shared.append(l)
            assembled.extend(init_shared)

        # Defaults — must always be defined, even when the init-scan block
        # below is skipped (programs with no init code and no EEPROM preload
        # fall through and line ~5338 reads these).
        _deferred_preload_helpers = []
        _preload_survivors = set()

        # Also include any non-shared, non-init-only helpers needed by init
        # OR by the self_copy routine (which runs as stage-1 init code and
        # may call __eeprom_rd for EEPROM preload).
        if init_code_lines or ee_preload_bytes > 0:
            # Precompute helper bodies in self.code once for transitive scan.
            _helper_bodies_in_code = {}  # name → list of lines
            _hci = 0
            while _hci < len(self.code):
                _hs = self.code[_hci].strip()
                if _hs.endswith(':') and not _hs.startswith('.') and _hs.startswith('_'):
                    _hn = _hs[:-1]
                    _hstart = _hci
                    _hci += 1
                    while _hci < len(self.code):
                        _ns = self.code[_hci].strip()
                        if _ns.endswith(':') and not _ns.startswith('.') and _ns.startswith('_'):
                            break
                        _hci += 1
                    _helper_bodies_in_code[_hn] = self.code[_hstart:_hci]
                else:
                    _hci += 1

            init_needs = set()
            all_init_lines = list(init_code_lines) + list(self_copy)
            for name, start, end, fsize in init_only_funcs:
                all_init_lines.extend(self.code[start:end])
            def _scan_jals(lines, out):
                for line in lines:
                    s = line.strip()
                    if s.startswith('jal '):
                        target = s.split()[1]
                        if target.startswith('__'):
                            out.add(target)
            _scan_jals(all_init_lines, init_needs)
            # Transitive: follow jal edges through helper bodies until fixed-point.
            _frontier = set(init_needs)
            while _frontier:
                _next = set()
                for _hn in _frontier:
                    _body = _helper_bodies_in_code.get(_hn, [])
                    _tmp = set()
                    _scan_jals(_body, _tmp)
                    for _t in _tmp:
                        if _t not in init_needs:
                            init_needs.add(_t)
                            _next.add(_t)
                _frontier = _next

            needed_but_not_init = init_needs - {n for n, _, _, _ in init_only_funcs}
            # Exclude mini-copied helpers (available at kernel addresses)
            needed_but_not_init -= mini_copied_helpers
            # Exclude helpers that are still resident (don't need init copies)
            needed_but_not_init -= {n for n in needed_but_not_init
                                    if f'{n}:' in _NO_OVERLAY and n not in init_rename_set}
            # The EEPROM preload is inline (uses only __i2c_sb which is
            # resident). No preload-survivor helpers need deferring to
            # code[KERNEL_SIZE..] — __i2c_sb is in the kernel already.
            # (_preload_survivors and _deferred_preload_helpers are initialised
            # unconditionally above so the tail-emission block works even
            # when this init-scan branch is skipped.)
            import sys as _emdbg
            if needed_but_not_init or _preload_survivors:
                i = 0
                while i < len(self.code):
                    s = self.code[i].strip()
                    if s.endswith(':') and not s.startswith('.'):
                        hname = s[:-1]
                        hstart = i
                        i += 1
                        while i < len(self.code):
                            ns = self.code[i].strip()
                            if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                                break
                            i += 1
                        if hname in needed_but_not_init or hname in _preload_survivors:
                            # Rename if this helper has labels in overlay sections
                            helper_lines = self.code[hstart:i]
                            if hname in init_rename_set:
                                helper_lines = _rename_shared_refs(helper_lines)
                                renamed = []
                                for hl in helper_lines:
                                    l = hl
                                    if l.strip().startswith('.') and l.strip().endswith(':'):
                                        lbl = l.strip()[:-1]
                                        l = l.replace(lbl, lbl + '_i')
                                    elif '.' in l and not l.strip().startswith(';'):
                                        import re as _re
                                        for m in _re.finditer(r'\.([\w]+)', l):
                                            ref = '.' + m.group(1)
                                            if any(ref + ':' in h2.strip() for h2 in self.code[hstart:i]):
                                                l = l.replace(ref, ref + '_i')
                                    renamed.append(l)
                                helper_lines = renamed
                            if hname in _preload_survivors:
                                _deferred_preload_helpers.extend(helper_lines)
                            else:
                                assembled.extend(helper_lines)
                    else:
                        i += 1

        # Label: init helpers done, continue to precomputation / self-copy
        if init_only_funcs:
            assembled.append('__init_done:')

        # DDRA for tone is configured once in runtime main after calibration.
        # __tone only toggles ORA/PA1; repeatedly bracketing DDRA around each
        # note was hardware-unstable.

        # Note table precomputation: convert ratio → half_period during init.
        # After delay_cal stores ipm to page3[240], iterate over note entries
        # and replace each ratio byte with (ipm * ratio) >> 4 = half_period.
        # This eliminates __tone_setup (~35B) from runtime overlays.
        # __tone_setup is emitted in init code and called per-note.
        # Note precomputation: convert ratio → half_period in page 1 note table.
        # tone_setup: B=ratio → C=half_period (reads ipm from page3[240], clobbers A,B,D)
        # Notes are in page 1 at data[note_base..note_base+num*3-1].
        # Note precomputation: convert ratio → half_period in page 1 note table.
        # tone_setup (init-only) converts B=ratio → C=half_period using ipm from page3[240].
        # Must run AFTER delay_cal stores ipm and BEFORE self-copy.
        # Only precompute if tone_setup is INIT-ONLY (not available at runtime).
        # If play_note calls tone_setup at runtime, precomp is unnecessary.
        _helpers = getattr(self, '_lcd_helpers', set())
        tone_setup_is_init_only = '__tone_setup' in {n for n, _, _, _ in init_only_funcs}
        if hasattr(self, '_note_table') and self._note_table and tone_setup_is_init_only:
            unique_notes = self._note_table
            note_base = unique_notes[0][0]
            note_end = unique_notes[-1][0] + 3
            assembled.append('; ── precompute: ratio → half_period in note table ──')
            # Use D as note pointer, stack for loop control
            assembled.append(f'\tldi $a,{note_base}')
            assembled.append('\tmov $a,$d')                # D = note ptr
            assembled.append('.precomp_loop:')
            assembled.append('\tmov $d,$a')                # A = note ptr
            assembled.append('\tderef')                    # A = data[ptr] = ratio
            assembled.append('\ttst 0xFF')
            assembled.append('\tjz .precomp_skip')         # silence (ratio=0)
            # A = ratio, D = note ptr
            # tone_setup: B=ratio → C=half_period (clobbers A,B,D)
            # push/pop manage SP correctly (verified: SD/SU in microcode)
            assembled.append('\tmov $a,$b')                # B = ratio
            assembled.append('\tpush $d')                  # save note ptr
            assembled.append('\tjal __tone_setup')         # C = half_period
            assembled.append('\tpop $d')                   # D = note ptr (restored)
            assembled.append('\tmov $c,$a')                # A = half_period
            assembled.append('\tmov $d,$b')                # B = note ptr
            assembled.append('\tideref')                   # data[B] = half_period
            assembled.append('.precomp_skip:')
            assembled.append('\tmov $d,$a')                # A = note ptr
            assembled.append('\taddi 3,$a')                # A += 3 (next entry)
            assembled.append('\tmov $a,$d')                # D = new ptr
            assembled.append(f'\tcmpi {note_end}')         # done?
            assembled.append('\tjnz .precomp_loop')

        # Self-copy must start at or after KERNEL_SIZE to avoid overwriting itself.
        # The copy writes to code[0..KERNEL_SIZE-1], so the loop code must be
        # at addresses >= KERNEL_SIZE. Pad with a jump + NOPs if init is short.
        # EEPROM-preload helpers (__eeprom_rd, __i2c_rb) go HERE (at or after
        # KERNEL_SIZE) so they survive self-copy — the preload loop calls them
        # after self-copy has overwritten code[0..KERNEL_SIZE-1].
        init_lines_filtered = [l for l in assembled[phase4_start:]
                               if l.strip() and not l.strip().startswith(';')
                               and 'section' not in l and 'org' not in l]
        init_size = measure_lines(init_lines_filtered)
        if ee_preload_bytes > 0:
            import sys as _l1dbg
            tail_bytes = measure_lines(_deferred_preload_helpers) \
                       + measure_lines(_deferred_init_only_bodies) \
                       + measure_lines(self_copy)
            print(f"  [stage1] init_pre_pad={init_size}B, kernel={KERNEL_SIZE}B, "
                  f"tail(helpers+selfcopy+preload)={tail_bytes}B, "
                  f"budget={250-KERNEL_SIZE}B → "
                  f"{'FITS' if KERNEL_SIZE + tail_bytes <= 250 else 'OVERFLOW'}",
                  file=_l1dbg.stderr)
        if init_size < KERNEL_SIZE:
            pad_needed = KERNEL_SIZE - init_size
            if pad_needed > 2:
                assembled.append('\tj __selfcopy')
                # Label prevents peephole dead-code elimination of padding
                assembled.append('__pad:')
                for _ in range(pad_needed - 2):
                    assembled.append('\tnop')
            elif pad_needed > 0:
                for _ in range(pad_needed):
                    assembled.append('\tnop')

        # Preload-survivor helpers: emitted at code[KERNEL_SIZE..] so they
        # survive the self-copy that overwrites code[0..KERNEL_SIZE-1].
        if _deferred_preload_helpers:
            assembled.extend(_deferred_preload_helpers)
        if _deferred_init_only_bodies:
            assembled.extend(_deferred_init_only_bodies)

        assembled.append('__selfcopy:')
        assembled.extend(self_copy)

        # Update page3_alloc
        total_p3_overlay = meta_table_size + sum(f for _, _, _, f, _ in p3_overlays)
        self.page3_alloc = max(p3_used, OVERLAY_REGION) + total_p3_overlay

        # Note: note table offsets don't need fixup since notes are in page 1
        # (data page has its own allocation counter, not affected by page 3 overlays)

        self.code = assembled
        self._overlay_kernel_size = KERNEL_SIZE
        self._overlay_region = OVERLAY_REGION
        self._overlay_shared_size = shared_helper_size

        # Named size breakdown for --why-not-smaller. Captured here rather
        # than re-derived from `assembled` later because we have the
        # partition data in hand (runtime_resident_helpers, overlay_meta).
        _why = {
            'mode': 'overlay',
            'loader': loader_size,
            'main': actual_main_size,
            'helpers_total': runtime_helper_size,
            'shared_helpers_total': shared_helper_size,
            'kernel_total': KERNEL_SIZE,
            'stage1_init': None,      # set by caller after emit
            'resident_helpers': {},   # name -> bytes
            'overlay_slots': [],      # (name, page, size, offset)
        }
        # Walk runtime_resident_helpers to name each top-level helper.
        _cur_name = None
        _cur_size = 0
        _two_byte_set = {'ldsp','stsp','push_imm','jal','jc','jz','jnc','jnz','j','ldi',
                         'cmp','addi','subi','andi','ori','ld','st','ldsp_b','ldp3','stp3',
                         'ocall','tst','out_imm','cmpi','ddrb_imm','ddra_imm',
                         'ora_imm','orb_imm'}
        for _line in runtime_resident_helpers:
            _s = _line.strip().split(';')[0].strip()
            if _s.endswith(':') and not _s.startswith('.'):
                if _cur_name is not None:
                    _why['resident_helpers'][_cur_name] = _cur_size
                _cur_name = _s[:-1]
                _cur_size = 0
                continue
            if not _s or _s.endswith(':') or _s.startswith('section') or _s.startswith('org'):
                continue
            _mn = _s.split()[0]
            if _mn == 'cmp':
                _parts = _s.split()
                _cur_size += 1 if (len(_parts) > 1 and _parts[1].startswith('$')) else 2
            elif _mn == 'ddrb2_imm':
                _cur_size += 3
            elif _mn == 'ddrb3_imm':
                _cur_size += 4
            elif _mn in _two_byte_set:
                _cur_size += 2
            else:
                _cur_size += 1
        if _cur_name is not None:
            _why['resident_helpers'][_cur_name] = _cur_size
        # Overlay slot info for the report.
        for _idx, _name, _sz in overlay_meta:
            _why['overlay_slots'].append({'idx': _idx, 'name': _name, 'size': _sz})
        self._why_breakdown = _why

        # ── Diagnostics ──
        print(f"Two-stage boot overlay system:", file=sys.stderr)
        print(f"  Kernel: {KERNEL_SIZE}B (loader={loader_size}B + main={actual_main_size}B"
              f" + helpers={runtime_helper_size}B)"
              f" + shared={shared_helper_size}B", file=sys.stderr)
        print(f"  Overlay region: 0x{OVERLAY_REGION:02X}-0xFF ({256-OVERLAY_REGION}B)",
              file=sys.stderr)
        print(f"  {len(overlay_slots)} overlay slots, largest={max_bundled_size}B"
              f" (func={max_slot_size}B + bundled helpers)", file=sys.stderr)
        avail = 250 - OVERLAY_REGION
        if max_bundled_size > avail:
            wrap = max_bundled_size - avail
            # Find which overlay(s) wrap and check if they're loaded last
            wrapping_overlays = [(n, fs) for _, n, fs in overlay_meta
                                 if fs > avail]  # pre-bundling check
            # Check assembled code for overlay call order. Multi-entry
            # dispatch uses $c; older form used $a (still tolerated).
            call_order = []
            for ci, line in enumerate(assembled):
                s = line.strip()
                if s == 'jal _overlay_load' and ci > 0:
                    prev = assembled[ci - 1].strip()
                    if prev.startswith('ldi $c,') or prev.startswith('ldi $a,'):
                        try:
                            idx = int(prev.split(',')[1])
                            call_order.append(idx)
                        except ValueError:
                            pass
            # Find wrapping overlay indices
            wrapping_indices = set()
            for oi, (_, olines, ofsize) in enumerate(overlay_asm_blocks):
                if ofsize > avail:
                    wrapping_indices.add(oi)
            # Map old indices to new indices (after placement reordering)
            wrapping_new = {old_to_new.get(oi, oi) for oi in wrapping_indices}
            safe = True
            if call_order and wrapping_new:
                last_call_idx = call_order[-1]
                for wi in wrapping_new:
                    if wi in call_order and call_order.index(wi) < len(call_order) - 1:
                        safe = False
            if safe:
                print(f"  WARNING: largest overlay wraps {wrap}B (safe: loaded last)",
                      file=sys.stderr)
            else:
                print(f"  WARNING: wrapping overlay NOT loaded last! "
                      f"{wrap}B wrap may corrupt kernel for subsequent loads",
                      file=sys.stderr)
        print(f"  Init-only: {[n for n,_,_,_ in init_only_funcs]}", file=sys.stderr)
        if has_p3:
            print(f"  Page 3: {len(p3_overlays)} overlays, "
                  f"{sum(f for _,_,_,f,_ in p3_overlays)}B", file=sys.stderr)
        if has_p1:
            print(f"  Page 1: {len(p1_overlays)} overlays, "
                  f"{sum(f for _,_,_,f,_ in p1_overlays)}B", file=sys.stderr)
        if has_p2:
            print(f"  Page 2: {len(p2_overlays)} overlays, "
                  f"{sum(f for _,_,_,f,_ in p2_overlays)}B", file=sys.stderr)
        if has_eeprom:
            print(f"  EEPROM: {len(ee_overlays)} overlays, "
                  f"{sum(f for _,_,_,f,_ in ee_overlays)}B [MK1 init preload]",
                  file=sys.stderr)


    def _emit_user_call(self, name, args):
        """Emit code to call a user-defined function by name. Factored out of
        gen_expr so that user functions with names matching a builtin (e.g. a
        user's own lcd_cmd) can be dispatched without going through the
        builtin code path."""
        nparams = self.func_params.get(name, len(args))
        use_regcall = self._use_regcall(nparams) and len(args) == nparams
        if use_regcall:
            stack_args = args[2:]
            for arg in reversed(stack_args):
                if arg[0] == 'num':
                    self.emit(f'\tpush_imm {arg[1] & 0xFF}')
                else:
                    self.gen_expr(arg)
                    self.emit('\tpush $a')
                self.local_count += 1
            if len(args) >= 2:
                if args[1][0] == 'num':
                    self.emit(f'\tldi $b,{args[1][1] & 0xFF}')
                else:
                    self.gen_expr(args[1])
                    self.emit('\tmov $a,$b')
            if len(args) >= 1:
                if args[0][0] == 'num':
                    self.emit(f'\tldi $a,{args[0][1] & 0xFF}')
                else:
                    self.gen_expr(args[0])
            self.emit(f'\tjal _{name}')
            for _ in stack_args:
                self.emit('\tpop $d')
                self.local_count -= 1
        else:
            for arg in reversed(args):
                if arg[0] == 'num':
                    self.emit(f'\tpush_imm {arg[1] & 0xFF}')
                else:
                    self.gen_expr(arg)
                    self.emit('\tpush $a')
                self.local_count += 1
            self.emit(f'\tjal _{name}')
            for _ in args:
                self.emit('\tpop $d')
                self.local_count -= 1

    def _use_regcall(self, nparams):
        """Always pass first 2 args in A/B (hybrid register calling).
        Functions with >2 params: first 2 in A/B, rest on stack."""
        return nparams > 0  # all functions with params use regcall

    def compile_function(self, name, params, body, ret_type):
        self.current_func = name
        self.locals = {}
        self.local_count = 0
        self.param_count = len(params)
        self.read_vars = self._find_dead_locals(body, params)
        # Register allocation: try to put locals in C and D
        if body[0] == 'block':
            self.reg_c_var, self.reg_d_var = self._find_reg_candidates(body[1])
        else:
            self.reg_c_var = self.reg_d_var = None

        self.regcall = self._use_regcall(len(params))

        if self.regcall:
            # Hybrid register calling: first 2 in A/B, rest on stack
            # Param 0 (A) is fragile — A gets clobbered by almost everything.
            # Save to D at entry if param 0 is used and D is available.
            # Param 1 (B) is safer — only clobbered by explicit mov/pop to B.
            # If D is taken by a local but the param needs saving, evict the
            # local from D (it'll go to stack instead). Parameter preservation
            # takes priority over local register allocation.
            #
            # IMPORTANT: several compiler-emitted helpers clobber $d without
            # restoring it. Saving param 0 to $d across a call that clobbers
            # $d produces a silent miscompile (e.g. `t` saved to $d before
            # `printf(...)` came back as the end-of-string pointer value
            # instead of `t`). We use `self._d_clobber_funcs` (computed in
            # compile() as a transitive closure over user functions) to
            # decide precisely whether the body contains any $d-clobbering
            # call. Calls that do NOT clobber $d (e.g. a user function that
            # only uses $a/$b/$c) still allow the cheaper save-to-$d path.
            has_locals = any(s[0] == 'local' for s in body[1]) if body[0] == 'block' else False
            has_calls = self._has_calls(body)
            has_d_clobber_calls = self._body_calls_any(
                body, getattr(self, '_d_clobber_funcs', set()))
            # Param 0 stays in $a only if it's read at most once. A second
            # read sees a clobbered $a (e.g. `buf[0]=x+1; buf[1]=x+2;` —
            # the `inc` for the first expression turns A from x into x+1,
            # so the second `addi 2,$a` produces x+3 instead of x+2). Force
            # a save in that case so subsequent reads can reload from D or
            # the stack.
            param0_reads = (self._count_reads(body, params[0])
                            if len(params) >= 1 else 0)
            param1_reads = (self._count_reads(body, params[1])
                            if len(params) >= 2 else 0)
            # Param 0 stays in $a only if it's read at most once. A second
            # read sees a clobbered $a (e.g. `buf[0]=x+1; buf[1]=x+2;` —
            # the `inc` for the first expression turns A from x into x+1,
            # so the second `addi 2,$a` produces x+3 instead of x+2). Force
            # a save in that case so subsequent reads can reload from D or
            # the stack.
            needs_save = (len(params) >= 1
                          and params[0] in self.read_vars
                          and self.reg_d_var is None
                          and (has_locals or len(params) > 2
                               or has_calls
                               or param0_reads > 1))
            # Save to $d only if no call in the body clobbers $d.
            save_a_to_d = needs_save and not has_d_clobber_calls
            # If $d is unavailable OR a $d-clobbering call exists, save to stack.
            save_a_to_stack = (not save_a_to_d
                               and len(params) >= 1
                               and params[0] in self.read_vars
                               and (has_locals or has_calls
                                    or param0_reads > 1))
            # Param 1 (in $b) has the same multi-read hazard: B gets
            # clobbered by `ldi $b, N` for array indexing, by mov $b
            # in argument shuffles, etc. Save to stack if read more
            # than once. ($d is not used for param 1 because it may
            # already be holding param 0's save, and because the
            # save-to-D path needs the precise d-clobber analysis the
            # existing code only does for param 0.)
            save_b_to_stack = (len(params) >= 2
                               and params[1] in self.read_vars
                               and param1_reads > 1)
            for i, pname in enumerate(params):
                if i == 0:
                    if save_a_to_d:
                        self.locals[pname] = ('reg', 'd')  # will be saved at entry
                    elif save_a_to_stack:
                        # Track as local; the push $a is emitted AFTER the
                        # function label (below) so the peephole's dead-code
                        # pass doesn't strip it as unreachable code between
                        # the previous function's terminator and this label.
                        self.local_count += 1
                        self.locals[pname] = ('local', self.local_count - 1)
                    else:
                        self.locals[pname] = ('regparam', 'a')
                elif i == 1:
                    if save_b_to_stack:
                        # Track as a local so gen_expr emits ldsp loads
                        # from the saved stack slot. push $b is emitted
                        # in the entry preamble below, after the optional
                        # push $a so offsets line up with _sp_offset's
                        # `local_count - idx` formula.
                        self.local_count += 1
                        self.locals[pname] = ('local', self.local_count - 1)
                    else:
                        self.locals[pname] = ('regparam', 'b')
                else:
                    self.locals[pname] = ('param', i - 2)
        else:
            for i, pname in enumerate(params):
                self.locals[pname] = ('param', i)

        # Constant propagation: track variables with known constant values
        self.const_vars = {}

        self.emit(f'_{name}:')

        # Save regparam A to D if needed
        if self.regcall and len(params) >= 1:
            loc = self.locals[params[0]]
            if loc == ('reg', 'd'):
                self.emit('\tmov $a,$d')  # save param 0 before A gets clobbered
            elif loc[0] == 'local' and loc[1] == 0:
                # save_a_to_stack path: emit the push HERE, after the label,
                # so dead-code elimination can't strip it.
                self.emit('\tpush $a')
        if self.regcall and len(params) >= 2:
            loc = self.locals[params[1]]
            if loc[0] == 'local':
                # save_b_to_stack path: param 1 in $b is multi-read, so
                # save it before any expression clobbers B (e.g. `ldi
                # $b, N` for array indexing). The push goes AFTER push
                # $a (if any) so _sp_offset's `local_count - idx`
                # ordering matches the runtime stack: param 0 deeper,
                # param 1 on top.
                self.emit('\tpush $b')

        self.compile_stmt(body)

        for _ in range(self.local_count):
            self.emit('\tpop $d')
        # Don't emit ret if the last instruction is already ret (from a return statement)
        if not self.code or self.code[-1].strip() != 'ret':
            # Tail-call a final subroutine when there is no local cleanup. A
            # direct jump lets the callee return to our caller, saving 1B and
            # avoiding a redundant return hop. This is especially useful for
            # compiler-lowered device primitives whose wrapper function body is
            # just one helper call.
            if (self.local_count == 0 and self.code
                    and self.code[-1].strip().startswith('jal ')
                    and self.code[-1].strip() != 'jal _overlay_load'
                    and not self.code[-1].strip().split()[1].startswith('_')):
                self.code[-1] = self.code[-1].replace('\tjal ', '\tj ', 1)
                return
            self.emit('\tret')

    def _sp_offset(self, name):
        kind, idx = self.locals[name]
        if kind == 'param':
            return self.local_count + 2 + idx
        else:
            return self.local_count - idx

    def _sp_offset_raw(self, name, byte_idx=0):
        """Get ldsp/stsp offset for a variable. byte_idx=0 for low, 1 for high (u16)."""
        kind, idx = self.locals[name][:2]
        if kind == 'param':
            return self.local_count + 2 + idx + byte_idx
        elif kind == 'local16':
            # local16 occupies 2 slots: idx = low byte, idx+1 = high byte
            return self.local_count - idx - byte_idx
        else:
            return self.local_count - idx

    def _is_dead(self, name):
        """True if this variable is never read (only written)."""
        return name not in self.read_vars

    def compile_stmt(self, stmt):
        if stmt is None:
            return
        kind = stmt[0]

        if kind == 'block':
            saved = self.local_count
            stmts = stmt[1]
            i = 0
            while i < len(stmts):
                s = stmts[i]
                # Merge "type x; x = expr;" into "type x = expr;"
                if (s[0] == 'local' and s[2] is None
                        and i + 1 < len(stmts) and stmts[i+1][0] == 'expr_stmt'
                        and stmts[i+1][1][0] == 'assign' and stmts[i+1][1][1] == '='
                        and stmts[i+1][1][2] == ('var', s[1])):
                    init_val = stmts[i+1][1][3]
                    merge_typ = s[3] if len(s) > 3 else 'u8'
                    self.compile_stmt(('local', s[1], init_val, merge_typ))
                    i += 2
                    continue
                self.compile_stmt(s)
                i += 1
            while self.local_count > saved:
                self.emit('\tpop $d')
                self.local_count -= 1

        elif kind == 'local':
            name = stmt[1]
            init = stmt[2]
            typ = stmt[3] if len(stmt) > 3 else 'u8'

            # Register allocation: put in C or D register (no stack slot)
            # 16-bit vars can't be register-allocated (registers are 8-bit)
            reg = None
            if typ != 'u16':
                if name == self.reg_c_var:
                    reg = 'c'
                elif name == self.reg_d_var:
                    reg = 'd'
            if reg:
                # Use ldi $reg, N directly — 2 bytes, preserves A
                c = self._const_eval(init) if init else 0
                if c is not None:
                    self.emit(f'\tldi ${reg},{c & 0xFF}')
                    self.const_vars[name] = c & 0xFF
                else:
                    self.gen_expr(init)
                    self.emit(f'\tmov $a,${reg}')
                self.locals[name] = ('reg', reg)
                return  # no stack slot, no local_count increment

            # Dead variable: no stack slot needed if no other locals depend on offset
            if self._is_dead(name):
                if init and init[0] != 'num':
                    self.gen_expr(init)  # compute for side effects (function calls)
                    # Don't push — keep A for subsequent use (e.g., out())
                else:
                    pass  # no side effects, skip entirely
                self.locals[name] = ('dead', 0)
                return  # no stack slot

            if typ == 'u16':
                # 16-bit local: 2 stack slots (high byte pushed first, then low)
                c = self._const_eval(init) if init else 0
                if c is not None:
                    self.emit(f'\tpush_imm {(c >> 8) & 0xFF}')  # high byte
                    self.emit(f'\tpush_imm {c & 0xFF}')          # low byte
                    self.const_vars[name] = c & 0xFFFF
                else:
                    # For non-constant 16-bit init, gen_expr returns low byte in A
                    # TODO: 16-bit expression evaluation
                    self.gen_expr(init)
                    self.emit('\tpush $a')       # low byte
                    self.emit('\tpush_imm 0')    # high byte = 0 for now
                self.local_count += 2
                self.locals[name] = ('local16', self.local_count - 2)
            else:
                c = self._const_eval(init) if init else 0
                if c is not None:
                    self.emit(f'\tpush_imm {c & 0xFF}')
                    self.const_vars[name] = c & 0xFF
                elif init:
                    self.gen_expr(init)
                    self.emit('\tpush $a')
                else:
                    self.emit('\tpush_imm 0')
                    self.const_vars[name] = 0
                self.local_count += 1
                self.locals[name] = ('local', self.local_count - 1)

        elif kind == 'local_arr':
            name = stmt[1]; size = stmt[2]
            init_vals = stmt[3] if len(stmt) > 3 else None
            self.globals[name] = self.data_alloc
            self.data_alloc += size
            if init_vals:
                for idx, val_expr in enumerate(init_vals):
                    if idx >= size: break
                    # Generate: arr[idx] = val
                    self.gen_expr(val_expr)
                    self.emit(f'\tldi $b,{self.globals[name] + idx}')
                    self.emit('\tideref')

        elif kind == 'expr_stmt':
            # Track constant assignments for propagation
            e = stmt[1]
            if e[0] == 'assign' and e[1] == '=' and e[2][0] == 'var':
                c = self._const_eval(e[3])
                if c is not None:
                    self.const_vars[e[2][1]] = c
                else:
                    self.const_vars.pop(e[2][1], None)
            elif e[0] == 'assign' and e[1] in ('+=','-=','*=','/=','%=','&=','|=','^=','<<=','>>=') and e[2][0] == 'var':
                self.const_vars.pop(e[2][1], None)  # compound assign invalidates
            elif e[0] in ('postinc','postdec','preinc','predec') and e[1][0] == 'var':
                self.const_vars.pop(e[1][1], None)
            self.gen_expr(stmt[1], discard=True)

        elif kind == 'return':
            # Tail call optimization: return f(x) → pop locals, then j f
            if (stmt[1] and stmt[1][0] == 'call'
                    and stmt[1][1] not in ('out', 'halt')
                    and self.local_count == 0):
                # No locals to pop — can tail-call directly
                call = stmt[1]
                name = call[1]
                args = call[2]
                nparams = self.func_params.get(name, len(args))
                if self._use_regcall(nparams) and len(args) == nparams and len(args) <= 2:
                    # Tail call with register args: load args, then j (not jal)
                    if len(args) >= 2:
                        if args[1][0] == 'num':
                            self.emit(f'\tldi $b,{args[1][1] & 0xFF}')
                        else:
                            self.gen_expr(args[1])
                            self.emit('\tmov $a,$b')
                    if len(args) >= 1:
                        if args[0][0] == 'num':
                            self.emit(f'\tldi $a,{args[0][1] & 0xFF}')
                        else:
                            self.gen_expr(args[0])
                    self.emit(f'\tj _{name}')  # jump, not jal — reuse caller's return addr
                    return
            if stmt[1]:
                self.gen_expr(stmt[1])
            for _ in range(self.local_count):
                self.emit('\tpop $d')
            self.emit('\tret')

        elif kind == 'if':
            _, cond, then, els = stmt
            # Invalidate const_vars after branches (conservative)
            saved_consts = dict(self.const_vars)
            if els:
                else_l, end_l = self.label('else'), self.label('endif')
                self.gen_branch_false(cond, else_l)
                self.compile_stmt(then)
                self.emit(f'\tj {end_l}')
                self.emit(f'{else_l}:')
                self.compile_stmt(els)
                self.emit(f'{end_l}:')
            else:
                end_l = self.label('endif')
                self.gen_branch_false(cond, end_l)
                self.compile_stmt(then)
                self.emit(f'{end_l}:')
            self.const_vars = saved_consts  # invalidate changes from branches

        elif kind == 'while':
            _, cond, body = stmt
            loop_l, end_l = self.label('while'), self.label('endwhile')
            saved_break = getattr(self, '_break_label', None)
            saved_cont = getattr(self, '_continue_label', None)
            self._break_label = end_l
            self._continue_label = loop_l
            # Invalidate const_vars at loop header — variables change across iterations
            saved_consts = dict(self.const_vars)
            self.const_vars.clear()
            self.emit(f'{loop_l}:')
            self.gen_branch_false(cond, end_l)
            self.compile_stmt(body)
            self.emit(f'\tj {loop_l}')
            self.emit(f'{end_l}:')
            self._break_label = saved_break
            self._continue_label = saved_cont

        elif kind == 'for':
            _, init, cond, update, body = stmt
            loop_l, end_l = self.label('for'), self.label('endfor')
            cont_l = self.label('for_cont')
            saved_break = getattr(self, '_break_label', None)
            saved_cont = getattr(self, '_continue_label', None)
            self._break_label = end_l
            self._continue_label = cont_l
            if init:
                self.compile_stmt(init)
            # Invalidate all const_vars at loop header — variables modified in the
            # body/update aren't constant across iterations. Without this, the
            # condition gets folded using initial values and the branch is eliminated.
            saved_consts = dict(self.const_vars)
            self.const_vars.clear()
            self.emit(f'{loop_l}:')
            self.gen_branch_false(cond, end_l)
            self.compile_stmt(body)
            self.emit(f'{cont_l}:')
            if update:
                self.gen_expr(update, discard=True)
            self.emit(f'\tj {loop_l}')
            self.emit(f'{end_l}:')
            self._break_label = saved_break
            self._continue_label = saved_cont

        elif kind == 'do_while':
            _, body, cond = stmt
            loop_l = self.label('dowhile')
            end_l = self.label('dowhile_end')
            saved_break = getattr(self, '_break_label', None)
            saved_cont = getattr(self, '_continue_label', None)
            self._break_label = end_l
            self._continue_label = loop_l  # continue goes to condition check
            # Invalidate const_vars — variables change across iterations
            saved_consts = dict(self.const_vars)
            self.const_vars.clear()
            self.emit(f'{loop_l}:')
            self.compile_stmt(body)
            # Use gen_branch_true to jump BACK to loop (1 jump, not 2)
            self.gen_branch_true(cond, loop_l)
            self.emit(f'{end_l}:')
            self._break_label = saved_break
            self._continue_label = saved_cont

        elif kind == 'break':
            if hasattr(self, '_break_label') and self._break_label:
                self.emit(f'\tj {self._break_label}')

        elif kind == 'continue':
            if hasattr(self, '_continue_label') and self._continue_label:
                self.emit(f'\tj {self._continue_label}')

        elif kind == 'switch':
            _, expr, cases, default = stmt
            self.gen_expr(expr)
            end_l = self.label('endswitch')
            saved_break = getattr(self, '_break_label', None)
            self._break_label = end_l
            for val, body in cases:
                skip_l = self.label('case_skip')
                self.emit(f'\tcmp {val}')
                self.emit(f'\tjnz {skip_l}')
                self.compile_stmt(body)
                self.emit(f'\tj {end_l}')
                self.emit(f'{skip_l}:')
            if default:
                self.compile_stmt(default)
            self.emit(f'{end_l}:')
            self._break_label = saved_break

    def _const_eval(self, expr):
        """Try to evaluate expr as a compile-time constant. Returns int or None.
        Values are NOT masked — callers mask to 0xFF or 0xFFFF as appropriate."""
        if expr[0] == 'num':
            return expr[1]
        if expr[0] == 'var' and hasattr(self, 'const_vars') and expr[1] in self.const_vars:
            return self.const_vars[expr[1]]
        if expr[0] == 'binop':
            l = self._const_eval(expr[2])
            r = self._const_eval(expr[3])
            if l is not None and r is not None:
                op = expr[1]
                if op == '+': return (l + r) & 0xFFFF
                if op == '-': return (l - r) & 0xFFFF
                if op == '*': return (l * r) & 0xFFFF
                if op == '/' and r != 0: return (l // r) & 0xFFFF
                if op == '%' and r != 0: return (l % r) & 0xFFFF
                if op == '&': return l & r
                if op == '|': return l | r
                if op == '^': return l ^ r
                if op == '<<': return (l << r) & 0xFFFF
                if op == '>>': return (l >> r) & 0xFFFF
                if op == '==': return 1 if l == r else 0
                if op == '!=': return 1 if l != r else 0
                if op == '<': return 1 if (l & 0xFFFF) < (r & 0xFFFF) else 0
                if op == '>': return 1 if (l & 0xFFFF) > (r & 0xFFFF) else 0
                if op == '<=': return 1 if (l & 0xFFFF) <= (r & 0xFFFF) else 0
                if op == '>=': return 1 if (l & 0xFFFF) >= (r & 0xFFFF) else 0
        if expr[0] == 'log_and':
            l = self._const_eval(expr[1])
            r = self._const_eval(expr[2])
            if l is not None and r is not None:
                return 1 if (l and r) else 0
        if expr[0] == 'log_or':
            l = self._const_eval(expr[1])
            r = self._const_eval(expr[2])
            if l is not None and r is not None:
                return 1 if (l or r) else 0
        if expr[0] == 'ternary':
            c = self._const_eval(expr[1])
            if c is not None:
                return self._const_eval(expr[2]) if c else self._const_eval(expr[3])
        if expr[0] == 'unop':
            v = self._const_eval(expr[2])
            if v is not None:
                if expr[1] == '-': return (-v) & 0xFFFF
                if expr[1] == '~': return (~v) & 0xFFFF
                if expr[1] == '!': return 1 if v == 0 else 0
        if expr[0] == 'call':
            # Inline-evaluate pure functions with constant args
            name = expr[1]
            args = expr[2]
            if all(self._const_eval(a) is not None for a in args):
                result = self._try_inline_eval(name, [self._const_eval(a) for a in args])
                if result is not None:
                    return result & 0xFFFF
        return None

    def _has_runtime_deps(self, node):
        """Check if AST node depends on runtime state (arrays, globals, I/O)."""
        if not isinstance(node, tuple) or len(node) == 0:
            return False
        if node[0] == 'index':
            return True
        if node[0] == 'switch':
            return True  # interpreter can't handle switch yet
        if node[0] == 'do_while':
            # Check if body/cond have deps
            if self._has_runtime_deps(node[1]) or self._has_runtime_deps(node[2]):
                return True
        if node[0] == 'call' and node[1] in ('out', 'halt', 'exw', 'exr'):
            return True
        if node[0] == 'var' and (node[1] in self.globals or node[1] in self.page3_globals):
            return True
        for child in node[1:]:
            if isinstance(child, tuple) and self._has_runtime_deps(child):
                return True
            if isinstance(child, list):
                for item in child:
                    if isinstance(item, tuple) and self._has_runtime_deps(item):
                        return True
        return False

    _CANT_EVAL = object()  # sentinel: interpreter can't evaluate this

    def _try_inline_eval(self, name, const_args):
        """Try to evaluate a function call with constant args at compile time."""
        if name not in self.func_bodies:
            return None
        params, body = self.func_bodies[name]
        if len(params) != len(const_args):
            return None
        if self._has_runtime_deps(body):
            return None  # depends on runtime state, can't fold
        env = {params[i]: const_args[i] for i in range(len(params))}
        result = self._interpret(body, env)
        if result is self._CANT_EVAL:
            return None
        return result

    def _interpret(self, stmt, env):
        """Interpret an AST node with constant environment. Returns value or None."""
        if stmt is None:
            return self._CANT_EVAL
        kind = stmt[0]
        if kind == 'block':
            for s in stmt[1]:
                r = self._interpret(s, env)
                if r is self._CANT_EVAL:
                    return self._CANT_EVAL  # propagate failure
                if r is not None:
                    return r
            return None
        if kind == 'return':
            if stmt[1]:
                return self._eval_const_expr(stmt[1], env)
            return 0
        if kind == 'if':
            _, cond, then, els = stmt
            c = self._eval_const_expr(cond, env)
            if c is None:
                return self._CANT_EVAL
            if c:
                return self._interpret(then, env)
            elif els:
                return self._interpret(els, env)
            return None
        if kind == 'local':
            name = stmt[1]
            init = stmt[2] if len(stmt) > 2 else None
            if init:
                v = self._eval_const_expr(init, env)
                if v is None:
                    return self._CANT_EVAL
                env[name] = v
            else:
                env[name] = 0
            return None
        if kind == 'expr_stmt':
            e = stmt[1]
            if e[0] == 'assign' and e[2][0] == 'var':
                name = e[2][1]
                op = e[1]
                if op == '=':
                    v = self._eval_const_expr(e[3], env)
                    if v is None:
                        return self._CANT_EVAL
                    env[name] = v & 0xFFFF
                    return None
                elif op in ('+=', '-=', '*=', '&=', '|=', '^='):
                    cur = env.get(name)
                    rhs = self._eval_const_expr(e[3], env)
                    if cur is None or rhs is None:
                        return self._CANT_EVAL
                    base_op = op[0]
                    ops = {'+': lambda a,b: a+b, '-': lambda a,b: a-b,
                           '*': lambda a,b: a*b, '&': lambda a,b: a&b,
                           '|': lambda a,b: a|b, '^': lambda a,b: a^b}
                    env[name] = ops[base_op](cur, rhs) & 0xFFFF
                    return None
            if e[0] in ('postinc', 'preinc') and e[1][0] == 'var':
                name = e[1][1]
                if name in env:
                    env[name] = (env[name] + 1) & 0xFFFF
                    return None
            if e[0] in ('postdec', 'predec') and e[1][0] == 'var':
                name = e[1][1]
                if name in env:
                    env[name] = (env[name] - 1) & 0xFFFF
                    return None
            return None
        if kind == 'break':
            return ('__break__',)  # sentinel
        if kind == 'continue':
            return ('__continue__',)  # sentinel
        if kind == 'while':
            _, cond, body = stmt
            for _ in range(1000):
                c = self._eval_const_expr(cond, env)
                if c is None:
                    return self._CANT_EVAL
                if not c:
                    return None
                r = self._interpret(body, env)
                if r is not None:
                    if isinstance(r, tuple) and r[0] == '__break__':
                        return None  # break exits loop
                    if isinstance(r, tuple) and r[0] == '__continue__':
                        continue
                    return r  # real return value
            return None
        if kind == 'do_while':
            _, body, cond = stmt
            for _ in range(1000):
                r = self._interpret(body, env)
                if r is not None:
                    if isinstance(r, tuple) and r[0] == '__break__':
                        return None
                    if isinstance(r, tuple) and r[0] == '__continue__':
                        pass  # continue to condition check
                    else:
                        return r
                c = self._eval_const_expr(cond, env)
                if c is None:
                    return self._CANT_EVAL
                if not c:
                    return None
            return None
        if kind == 'for':
            _, init, cond, update, body = stmt
            if init:
                r = self._interpret(init, env)
                if r is not None:
                    return r
            for _ in range(1000):  # safety limit
                c = self._eval_const_expr(cond, env)
                if c is None:
                    return self._CANT_EVAL
                if not c:
                    return None
                r = self._interpret(body, env)
                if r is not None:
                    if isinstance(r, tuple) and r[0] == '__break__':
                        return None  # break exits loop
                    if isinstance(r, tuple) and r[0] == '__continue__':
                        pass  # continue skips to update
                    else:
                        return r  # real return value
                if update:
                    # Handle all update expressions: assign, compound, pre/post inc/dec
                    if update[0] == 'assign' and update[2][0] == 'var':
                        op = update[1]
                        name = update[2][1]
                        rhs = self._eval_const_expr(update[3], env)
                        if rhs is None:
                            return self._CANT_EVAL
                        if op == '=':
                            env[name] = rhs & 0xFFFF
                        elif op in ('+=','-=','*=','/=','%=','&=','|=','^=','<<=','>>='):
                            cur = env.get(name, 0)
                            ops = {'+':lambda a,b:a+b, '-':lambda a,b:a-b,
                                   '*':lambda a,b:a*b, '&':lambda a,b:a&b,
                                   '|':lambda a,b:a|b, '^':lambda a,b:a^b,
                                   '/':lambda a,b:a//b if b else 0,
                                   '%':lambda a,b:a%b if b else 0,
                                   '<':lambda a,b:a<<b, '>':lambda a,b:a>>b}
                            env[name] = ops[op[0]](cur, rhs) & 0xFFFF
                    else:
                        v = self._eval_const_expr(update, env)
                        if v is None:
                            return self._CANT_EVAL
            return None
        return None

    def _eval_const_expr(self, expr, env):
        """Evaluate expression with variable environment. 16-bit intermediate values."""
        if expr[0] == 'num':
            return expr[1]
        if expr[0] == 'var':
            return env.get(expr[1])
        if expr[0] == 'binop':
            l = self._eval_const_expr(expr[2], env)
            r = self._eval_const_expr(expr[3], env)
            if l is None or r is None:
                return self._CANT_EVAL
            op = expr[1]
            if op == '+': return (l + r) & 0xFFFF
            if op == '-': return (l - r) & 0xFFFF
            if op == '*': return (l * r) & 0xFFFF
            if op == '/' and r != 0: return (l // r) & 0xFFFF
            if op == '%' and r != 0: return (l % r) & 0xFFFF
            if op == '&': return l & r
            if op == '|': return l | r
            if op == '^': return l ^ r
            if op == '<<': return (l << r) & 0xFFFF
            if op == '>>': return (l >> r) & 0xFFFF
            if op == '==': return 1 if l == r else 0
            if op == '!=': return 1 if l != r else 0
            if op == '<': return 1 if (l & 0xFFFF) < (r & 0xFFFF) else 0
            if op == '>': return 1 if (l & 0xFFFF) > (r & 0xFFFF) else 0
            if op == '<=': return 1 if (l & 0xFFFF) <= (r & 0xFFFF) else 0
            if op == '>=': return 1 if (l & 0xFFFF) >= (r & 0xFFFF) else 0
        if expr[0] == 'unop':
            v = self._eval_const_expr(expr[2], env)
            if v is None:
                return self._CANT_EVAL
            if expr[1] == '-': return (-v) & 0xFFFF
            if expr[1] == '~': return (~v) & 0xFFFF
            if expr[1] == '!': return 1 if v == 0 else 0
        if expr[0] == 'log_and':
            l = self._eval_const_expr(expr[1], env)
            if l is not None and not l:
                return 0  # short-circuit
            r = self._eval_const_expr(expr[2], env)
            if l is not None and r is not None:
                return 1 if (l and r) else 0
            return None
        if expr[0] == 'log_or':
            l = self._eval_const_expr(expr[1], env)
            if l is not None and l:
                return 1  # short-circuit
            r = self._eval_const_expr(expr[2], env)
            if l is not None and r is not None:
                return 1 if (l or r) else 0
            return None
        if expr[0] == 'ternary':
            c = self._eval_const_expr(expr[1], env)
            if c is not None:
                return self._eval_const_expr(expr[2] if c else expr[3], env)
            return None
        if expr[0] == 'postinc' and expr[1][0] == 'var' and expr[1][1] in env:
            old = env[expr[1][1]]
            env[expr[1][1]] = (old + 1) & 0xFFFF
            return old
        if expr[0] == 'postdec' and expr[1][0] == 'var' and expr[1][1] in env:
            old = env[expr[1][1]]
            env[expr[1][1]] = (old - 1) & 0xFFFF
            return old
        if expr[0] == 'preinc' and expr[1][0] == 'var' and expr[1][1] in env:
            env[expr[1][1]] = (env[expr[1][1]] + 1) & 0xFFFF
            return env[expr[1][1]]
        if expr[0] == 'predec' and expr[1][0] == 'var' and expr[1][1] in env:
            env[expr[1][1]] = (env[expr[1][1]] - 1) & 0xFFFF
            return env[expr[1][1]]
        if expr[0] == 'call':
            # Recursive compile-time evaluation
            name = expr[1]
            args = expr[2]
            const_args = [self._eval_const_expr(a, env) for a in args]
            if all(v is not None for v in const_args):
                result = self._try_inline_eval(name, const_args)
                if result is not None:
                    return result & 0xFFFF
        return None

    def _expr_type(self, expr):
        """Infer expression type: 'u8' or 'u16'."""
        if expr[0] == 'num':
            return 'u16' if expr[1] > 255 else 'u8'
        if expr[0] == 'var':
            name = expr[1]
            if name in self.locals and self.locals[name][0] == 'local16':
                return 'u16'
            if hasattr(self, 'const_vars') and name in self.const_vars:
                return 'u16' if self.const_vars[name] > 255 else 'u8'
            return 'u8'
        if expr[0] == 'binop':
            lt = self._expr_type(expr[2])
            rt = self._expr_type(expr[3])
            op = expr[1]
            # Shift right by constant >= 8 produces u8
            if op == '>>' and expr[3][0] == 'num' and expr[3][1] >= 8:
                return 'u8'
            return 'u16' if (lt == 'u16' or rt == 'u16') else 'u8'
        if expr[0] == 'call':
            # Check return type of function
            name = expr[1]
            if name in self.func_bodies:
                params, body = self.func_bodies[name]
                # Simple: check if function was declared with u16 return
                # For now, infer from function name convention or default u8
            return 'u8'
        return 'u8'

    def _gen_expr16(self, expr):
        """Generate 16-bit expression. Result: A=low byte, B=high byte."""
        kind = expr[0]

        # Constant folding
        const = self._const_eval(expr)
        if const is not None:
            val = const & 0xFFFF
            self.emit(f'\tldi $a,{val & 0xFF}')
            self.emit(f'\tldi $b,{(val >> 8) & 0xFF}')
            return

        if kind == 'num':
            val = expr[1] & 0xFFFF
            self.emit(f'\tldi $a,{val & 0xFF}')
            self.emit(f'\tldi $b,{(val >> 8) & 0xFF}')

        elif kind == 'var':
            name = expr[1]
            if name in self.locals and self.locals[name][0] == 'local16':
                off_lo = self._sp_offset_raw(name, 0)
                off_hi = self._sp_offset_raw(name, 1)
                self.emit(f'\tldsp_b {off_hi}')  # B = hi (clobbers A)
                self.emit(f'\tldsp {off_lo}')     # A = lo

        elif kind == 'binop':
            op = expr[1]
            left, right = expr[2], expr[3]

            if op in ('+', '-'):
                # Evaluate right, push both bytes
                self._gen_expr16(right)
                self.emit('\tpush $b')      # right_hi
                self.emit('\tpush $a')      # right_lo
                self.local_count += 2

                # Evaluate left
                self._gen_expr16(left)      # A=left_lo, B=left_hi

                # Add/sub low bytes
                self.emit('\tmov $b,$d')    # D = left_hi (save, preserves CF)
                self.emit('\tpop_b')       # B = right_lo
                self.local_count -= 1
                if op == '+':
                    self.emit('\tadd $b,$a')   # A = left_lo + right_lo, CF set
                else:
                    self.emit('\tsub $b,$a')   # A = left_lo - right_lo, CF set

                self.emit('\tmov $a,$c')    # C = result_lo (preserves CF)

                # Add/sub high bytes with carry
                self.emit('\tpop $a')       # A = right_hi (preserves CF!)
                self.local_count -= 1
                self.emit('\tmov $d,$b')    # B = left_hi (preserves CF!)
                # For add: adc does A = A + B + CF. But we want left_hi + right_hi + CF.
                # A = right_hi, B = left_hi. adc → A = right_hi + left_hi + CF. ✓
                # For sub: sbc does A = A - B - !CF.
                # A = right_hi... wait, we want left_hi - right_hi - borrow.
                # Need A = left_hi, B = right_hi for sbc.
                if op == '+':
                    self.emit('\tadc')      # A = right_hi + left_hi + CF
                else:
                    # For subtraction: swap so A=left_hi, B=right_hi
                    self.emit('\tswap')     # A = left_hi, B = right_hi
                    self.emit('\tsbc')      # A = left_hi - right_hi - !CF

                self.emit('\tmov $a,$b')    # B = result_hi
                self.emit('\tmov $c,$a')    # A = result_lo

            elif op == '>>' and right[0] == 'num':
                # Shift right by constant — common for byte extraction
                shift = right[1]
                self._gen_expr16(left)
                if shift >= 8:
                    # High byte becomes low, shift remaining
                    self.emit('\tmov $b,$a')  # A = hi byte
                    for _ in range(shift - 8):
                        self.emit('\tslr')
                    self.emit('\tldi $b,0')   # B = 0
                else:
                    for _ in range(shift):
                        # Shift B:A right by 1: B>>1 carry into A
                        # This is complex without a rotate-through-carry
                        # Simplify: for small shifts, use paired shift
                        self.emit('\tslr')    # A >>= 1 (simplified, loses cross-byte carry)
                    # TODO: proper 16-bit shift with carry between bytes

            elif op == '<<' and right[0] == 'num':
                shift = right[1]
                self._gen_expr16(left)
                if shift >= 8:
                    # Low byte becomes high, clear low
                    self.emit('\tmov $a,$b')    # B = old low byte
                    for _ in range(shift - 8):
                        self.emit('\tsll')      # shift B left... actually B isn't in A
                    # Need to shift B. Move to A, shift, move back
                    if shift > 8:
                        self.emit('\tmov $b,$a')
                        for _ in range(shift - 8):
                            self.emit('\tsll')
                        self.emit('\tmov $a,$b')
                    self.emit('\tclr $a')       # A = 0 (low byte is 0)
                else:
                    for _ in range(shift):
                        # 16-bit left shift: shift A left, carry into B
                        self.emit('\tsll')       # A <<= 1, CF = old MSB
                        # TODO: proper carry into B
                    # Simplified: for small shifts just shift A
                    # B handling deferred

            elif op == '&' and right[0] == 'num' and right[1] == 0xFF:
                self._gen_expr16(left)
                self.emit('\tldi $b,0')

            else:
                # Can't handle this 16-bit op — evaluate 8-bit and zero-extend
                # Use the 8-bit gen_expr path WITHOUT the 16-bit dispatch
                self._gen_expr_8bit(expr)
                self.emit('\tldi $b,0')

    def _gen_expr_8bit(self, expr):
        """Force 8-bit evaluation, bypassing 16-bit dispatch."""
        # Save and temporarily clear the 16-bit override
        saved = self._expr_type
        self._expr_type = lambda e: 'u8'
        self.gen_expr(expr)
        self._expr_type = saved

    def gen_expr(self, expr, discard=False):
        """Generate code to evaluate expr, result in A."""
        kind = expr[0]

        # Check if this is a 16-bit expression that should use _gen_expr16
        if self._expr_type(expr) == 'u16':
            const = self._const_eval(expr)
            if const is not None:
                # 16-bit constant — emit as 8-bit (low byte) for 8-bit context
                self.emit(f'\tldi $a,{const & 0xFF}')
                return
            # For 16-bit in 8-bit context, evaluate full 16-bit (A=lo, B=hi)
            self._gen_expr16(expr)
            return

        # Constant folding: evaluate at compile time if possible
        const = self._const_eval(expr)
        if const is not None and kind != 'num':  # avoid double-emit for literals
            if const == 0:
                self.emit('\tclr $a')
            else:
                self.emit(f'\tldi $a,{const & 0xFF}')
            return

        if kind == 'num':
            if expr[1] == 0:
                self.emit('\tclr $a')
            else:
                self.emit(f'\tldi $a,{expr[1] & 0xFF}')

        elif kind == 'string':
            # General string-literal codegen: emit the string into page 1
            # (data) null-terminated, return its page-1 offset in $a.
            # Strings are deduped by content. The address is referenced
            # by a symbolic label (`__str_N`) — the assembler resolves
            # it at link time, so the compiler never bakes a magic
            # numeric offset into `ldi $a, ...`.
            #
            # Why page 1 and not page 3: page 3's [0..KERNEL_SIZE-1]
            # range is reserved for the kernel image (which gets
            # self-copied to page 0 at boot). The original code
            # allocated string offsets via `page3_alloc` starting at 0,
            # which collided with the kernel image and corrupted the
            # first 4 bytes of page 0 after self-copy. Page 1 has no
            # such reserved area; data_alloc grows sequentially from 0
            # alongside app globals / note table / lcd-init blocks.
            s = expr[1]
            if not hasattr(self, '_lcd_print_strings'):
                self._lcd_print_strings = []
            # Dedupe by content — reuse an existing label if the same
            # string was already stored.
            for label, existing, _off in self._lcd_print_strings:
                if existing == s:
                    self.emit(f'\tldi $a,{label}')
                    return
            d_off = self.data_alloc
            label = f'__str_{len(self._lcd_print_strings)}'
            self._lcd_print_strings.append((label, s, d_off))
            self.data_alloc += len(s) + 1
            self.emit(f'\tldi $a,{label}')

        elif kind == 'var':
            name = expr[1]
            if name in self.locals:
                loc = self.locals[name]
                if loc[0] == 'reg':
                    self.emit(f'\tmov ${loc[1]},$a')
                elif loc[0] == 'regparam':
                    if loc[1] != 'a':
                        self.emit(f'\tmov ${loc[1]},$a')
                    # If param is in A, no instruction needed
                elif loc[0] == 'dead':
                    self.emit('\tclr $a')
                else:
                    self.emit(f'\tldsp {self._sp_offset(name)}')
            elif name in self.globals:
                self.emit(f'\tld $a,_{name}')
            elif name in self.page3_globals:
                self.emit(f'\tldp3 _{name}')
            elif name in self.locals and self.locals[name][0] == 'local16':
                # 16-bit local: load low byte into A, high byte into B
                idx = self.locals[name][1]
                off_lo = self.local_count - idx
                off_hi = self.local_count - idx - 1
                self.emit(f'\tldsp {off_lo}')    # A = low byte
                self.emit(f'\tldsp_b {off_hi}')  # B = high byte (clobbers A!)
                # Reload A after ldsp_b clobber
                self.emit(f'\tldsp {off_lo}')    # A = low byte again
            else:
                raise SyntaxError(f'Undefined: {name}')

        elif kind == 'assign':
            op, left, right = expr[1], expr[2], expr[3]
            if left[0] == 'index':
                arr_name = left[1][1] if left[1][0] == 'var' else None
                is_page3 = arr_name and arr_name in self.page3_globals
                if is_page3:
                    base = self.page3_globals.get(arr_name, 0)
                else:
                    base = self.globals.get(arr_name, 0)
                # Fast path: constant index → `ldi $b,addr` preserves A, so we
                # can skip the push/clr/mov/pop save-restore (saves 4B). Also
                # folds base+index at compile time. Works because ldi $b doesn't
                # touch A, and gen_expr(right) is evaluated BEFORE we load B.
                const_idx = self._const_eval(left[2])
                if const_idx is not None:
                    addr = (base + const_idx) & 0xFF
                    self.gen_expr(right)
                    self.emit(f'\tldi $b,{addr}')
                    self.emit('\tiderefp3' if is_page3 else '\tideref')
                    return
                self.gen_expr(right)
                self.emit('\tpush $a')
                self.local_count += 1
                self.gen_expr(left[2])
                if base:
                    self.emit(f'\taddi {base},$a')
                self.emit('\tmov $a,$b')
                self.emit('\tpop $a')
                self.local_count -= 1
                self.emit('\tiderefp3' if is_page3 else '\tideref')
                return

            if op != '=':
                # Compound assign: rewrite `x OP= y` as `x = x OP y` so all
                # operators (including <<, >>, /, %) go through the general
                # binop codegen which has constant-folding, strength reduction,
                # and reg-reg fast paths.
                op_bare = op[:-1]
                self.gen_expr(('binop', op_bare, left, right))
            else:
                self.gen_expr(right)

            # Dead or register variable: _store handles both
            if left[0] == 'var' and left[1] in self.locals:
                loc = self.locals[left[1]]
                if loc[0] == 'dead':
                    return  # skip store entirely
            self._store(left)

        elif kind == 'binop':
            op = expr[1]
            left, right = expr[2], expr[3]

            if op in ('==', '!=', '<', '>', '<=', '>='):
                self._gen_compare_value(op, left, right)
                return

            # Constant right operand optimizations
            if right[0] == 'num':
                val = right[1] & 0xFF
                self.gen_expr(left)
                if op == '+':
                    if val == 1:
                        self.emit('\tinc')
                    else:
                        self.emit(f'\taddi {val},$a')
                elif op == '-':
                    if val == 1:
                        self.emit('\tdec')
                    else:
                        self.emit(f'\tsubi {val},$a')
                elif op == '&':
                    self.emit(f'\tandi {val},$a')
                elif op == '|':
                    self.emit(f'\tori {val},$a')
                elif op == '*':
                    # Strength reduction: constant multiply → shifts + adds
                    if val == 0:
                        self.emit('\tclr $a')
                    elif val == 1:
                        pass  # identity
                    elif val == 2:
                        self.emit('\tsll')
                    elif val == 4:
                        self.emit('\tsll')
                        self.emit('\tsll')
                    elif val == 8:
                        self.emit('\tsll')
                        self.emit('\tsll')
                        self.emit('\tsll')
                    elif val == 3:
                        self.emit('\tmov $a,$b')
                        self.emit('\tsll')
                        self.emit('\tadd $b,$a')
                    elif val == 5:
                        self.emit('\tmov $a,$b')
                        self.emit('\tsll')
                        self.emit('\tsll')
                        self.emit('\tadd $b,$a')
                    elif val == 6:
                        self.emit('\tsll')  # *2
                        self.emit('\tmov $a,$b')
                        self.emit('\tsll')  # *4
                        self.emit('\tadd $b,$a')  # *2 + *4 = *6
                    elif val == 10:
                        self.emit('\tsll')  # *2
                        self.emit('\tmov $a,$b')
                        self.emit('\tsll')  # *4
                        self.emit('\tsll')  # *8
                        self.emit('\tadd $b,$a')  # *2 + *8 = *10
                    elif val == 7:           # *7 = (x<<3) - x
                        self.emit('\tmov $a,$b')
                        self.emit('\tsll'); self.emit('\tsll'); self.emit('\tsll')
                        self.emit('\tsub $b,$a')
                    elif val == 9:           # *9 = (x<<3) + x
                        self.emit('\tmov $a,$b')
                        self.emit('\tsll'); self.emit('\tsll'); self.emit('\tsll')
                        self.emit('\tadd $b,$a')
                    elif val == 12:          # *12 = (3x) << 2
                        self.emit('\tmov $a,$b')
                        self.emit('\tsll')
                        self.emit('\tadd $b,$a')
                        self.emit('\tsll'); self.emit('\tsll')
                    elif val == 16:          # *16 = 4 × sll
                        self.emit('\tsll'); self.emit('\tsll')
                        self.emit('\tsll'); self.emit('\tsll')
                    elif val == 32:          # *32 = 5 × sll
                        self.emit('\tsll'); self.emit('\tsll')
                        self.emit('\tsll'); self.emit('\tsll'); self.emit('\tsll')
                    else:
                        pass  # fall through to general multiply
                    if val in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 16, 32):
                        return
                elif op == '/':
                    if val == 1: pass  # identity
                    elif val == 2: self.emit('\tslr')
                    elif val == 4: self.emit('\tslr'); self.emit('\tslr')
                    elif val == 8: self.emit('\tslr'); self.emit('\tslr'); self.emit('\tslr')
                    else: pass  # fall through to general divide
                    if val in (1, 2, 4, 8): return
                elif op == '%':
                    if val == 2: self.emit('\tandi 1,$a')
                    elif val == 4: self.emit('\tandi 3,$a')
                    elif val == 8: self.emit('\tandi 7,$a')
                    elif val == 16: self.emit('\tandi 15,$a')
                    elif val == 128: self.emit('\tandi 127,$a')
                    elif val == 256: pass  # x % 256 = x for u8
                    else: pass  # fall through to general modulo
                    if val in (2, 4, 8, 16, 128, 256): return
                elif op == '<<':
                    for _ in range(val):
                        self.emit('\tsll')
                elif op == '>>':
                    for _ in range(val):
                        self.emit('\tslr')
                else:
                    pass  # fall through to general case for ^, *
                # Fall through to the general register-based path for ops
                # that don't have a complete constant-only expansion above:
                #   ^ and *  → always (general case handles via B)
                #   / and %  → when val is not a power-of-2 handled above
                if op not in ('^', '*', '/', '%'):
                    return

            # Optimization: if right operand is in register C or D, use direct
            # register-register ALU op instead of push/pop through B.
            # This avoids clobbering B (which may hold a function parameter).
            right_reg = None
            if right[0] == 'var' and right[1] in self.locals:
                loc = self.locals[right[1]]
                if loc in (('reg', 'c'), ('regvar', 'c')):
                    right_reg = 'c'
                elif loc in (('reg', 'd'), ('regvar', 'd')):
                    right_reg = 'd'
                elif loc == ('regparam', 'b'):
                    right_reg = 'b'
            if right_reg and op in ('+', '-', '&', '|'):
                # Left → A, then ALU with C or D directly
                self.gen_expr(left)
                alu_ops = {'+': 'add', '-': 'sub', '&': 'and', '|': 'or'}
                self.emit(f'\t{alu_ops[op]} ${right_reg},$a')
            else:
                # General binary: eval right, push, eval left, pop into B, op
                self.gen_expr(right)
                self.emit('\tpush $a')
                self.local_count += 1
                self.gen_expr(left)
                self.emit('\tpop_b')
                self.local_count -= 1
                self._emit_binop(op)

        elif kind == 'unop':
            self.gen_expr(expr[2])
            if expr[1] == '~':
                self.emit('\tnot')
            elif expr[1] == '-':
                self.emit('\tneg')
            elif expr[1] == '!':
                self.emit('\tcmp 0')
                self.emit('\tsetz')

        elif kind == 'call':
            name, args = expr[1], expr[2]
            # Invalidate the lcd_init-just-emitted peephole marker whenever
            # a call other than lcd_init / lcd_cmd / lcd_char runs. The marker
            # only covers the literal "lcd_init(); lcd_cmd(0x01);" sequence
            # with no intervening calls.
            if name not in ('lcd_init', 'lcd_cmd', 'lcd_char'):
                self._lcd_init_just_emitted = False
            # User-defined function with a name matching a builtin takes
            # precedence over the builtin. Dispatch directly to the user's
            # code without consulting the builtin table.
            if name in self.func_params:
                self._emit_user_call(name, args)
                return
            if name == 'out':
                if args:
                    arg = args[0]
                    # out_imm N: output any compile-time constant (2 bytes vs 3)
                    const = self._const_eval(arg)
                    if const is not None:
                        self.emit(f'\tout_imm {const & 0xFF}')
                        return
                    # out $reg: output register variable directly (1 byte vs 2)
                    if arg[0] == 'var' and arg[1] in self.locals:
                        loc = self.locals[arg[1]]
                        if loc[0] == 'reg' or loc[0] == 'regparam':
                            reg = loc[1]
                            if reg == 'a':
                                self.emit('\tout')
                            else:
                                self.emit(f'\tmov ${reg},$a')
                                self.emit('\tout')
                            return
                    self.gen_expr(arg)
                self.emit('\tout')
                return
            if name == 'halt':
                # Release I2C bus before halting — prevents ESP32 upload from
                # creating spurious I2C activity via VIA's retained DDRB state
                self.emit_ddrb(0x00)   # DDRB = 0 (SDA/SCL idle)
                # Safety trap: if the ESP32 clock keeps ticking past hlt
                # (HLT_GRACE_CYCLES=100000 ignores HLT for short programs),
                # PC would advance, fall through helpers, wrap around the
                # code page and re-run _main — which re-runs lcd_init and
                # CLEARS the display. The label-after-hlt shields the
                # backward `j` from peephole dead-code elimination so it
                # survives into the final asm. PC hits hlt, advances one
                # byte, hits j, loops back — infinite self-halt.
                lbl_trap = self.label('hlt_trap')
                lbl_after = self.label('hlt_after')
                self.emit(f'{lbl_trap}:')
                self.emit('\thlt')
                self.emit(f'{lbl_after}:')
                self.emit(f'\tj {lbl_trap}')
                return
            if name == 'nop':
                n = 1
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        n = c
                for _ in range(n):
                    self.emit('\tnop')
                return

            # External bus I/O builtins
            # exw(value) — write value to D0-D7 via E0 (default)
            # exw(value, mode) — write with E0/E1 + U0/U1 select
            # exr() — read external input into A
            if name == 'exw':
                # Determine which exw variant
                mode = 0
                enable = 0
                value_const = self._const_eval(args[0]) if args else None
                if len(args) >= 2:
                    c = self._const_eval(args[1])
                    if c is not None:
                        mode = c & 3
                if len(args) >= 3:
                    c = self._const_eval(args[2])
                    if c is not None:
                        enable = c & 1
                if value_const is not None and mode == 2 and enable == 0:
                    # Constant writes to VIA DDRB can use the compact
                    # immediate opcode. This also routes user-written low-level
                    # I2C/LCD code through the port-shadow preserving emitter.
                    self.emit_ddrb(value_const)
                    return
                if args:
                    self.const_vars.pop('__a_known', None)  # force reload
                    self.gen_expr(args[0])
                self.emit(f'\texw {enable} {mode}')
                return
            if name == 'exr':
                channel = 0
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        channel = c & 1
                self.emit(f'\texr {channel}')
                return

            # IRQ clear — exr clears IRQ when EXT_IN is HIGH
            # clr_irq0() / clr_irq1() reads and discards to trigger the clear
            if name == 'clr_irq0':
                self.emit('\texr 0')  # E0 asserted, clears IRQ0 if EXT_IN HIGH
                return
            if name == 'clr_irq1':
                self.emit('\texr 1')  # E1 asserted, clears IRQ1 if EXT_IN HIGH
                return

            # IRQ polling — returns 0 or 1 based on external IRQ line state
            if name in ('irq0', 'irq1'):
                set_l = self.label('irq_set')
                done_l = self.label('irq_done')
                instr = 'je0' if name == 'irq0' else 'je1'
                self.emit(f'\t{instr} {set_l}')
                self.emit('\tclr $a')
                self.emit(f'\tj {done_l}')
                self.emit(f'{set_l}:')
                self.emit('\tldi $a,1')
                self.emit(f'{done_l}:')
                return

            # 82C55 PPI port reads
            if name == 'exr_port_a':
                self.emit('\texr 1 0')
                return
            if name == 'exr_port_b':
                self.emit('\texr 1 1')
                return
            if name == 'exr_port_c':
                self.emit('\texr 1 2')
                return

            # W65C22S VIA reads (E0+E1: PHI2+R/W for read cycle)
            if name == 'exrw':
                mode = 0
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        mode = c & 3
                self.emit(f'\texrw {mode}')
                return
            if name == 'via_read_portb':
                self.emit('\texrw 0')
                return
            if name == 'via_read_porta':
                self.emit('\texrw 1')
                return

            # ── VIA I2C builtins (emit known-good assembly patterns) ──

            if name == 'i2c_init':
                # Ensure I2C helpers are available (needed for EEPROM preload
                # even if program doesn't explicitly use I2C send/read).
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                # Mark that i2c_init was called — ensures I2C helpers are
                # available for EEPROM preload (unconditionally resident).
                self._i2c_init_called = True
                # VIA init for I2C: delay for VIA ~RES settle (RC = 1ms),
                # then set ORB=0, DDRB=0, DDRA=0, init SP.
                # Loop counter lives in $a: dec underflows 0 → 0xFF, iterates
                # 256 times (~2ms @ 1MHz). CPU reset leaves $a=0 so no pre-load
                # needed. (On a soft reset $a may be non-zero, giving a shorter
                # delay, but by that point the VIA is already powered-up and
                # the RC delay is unnecessary.)
                lbl = self.label('via_dly')
                self.emit(f'{lbl}:')
                self.emit('\tdec')
                self.emit(f'\tjnz {lbl}')
                # After delay loop: A=0 (redundant — the loop's exit condition
                # IS "A=0"). Kept as an INIT_MARKER so the init-extraction
                # scanner in steps 7/11 correctly continues past this point.
                # Removing this causes overlay_dashboard to stop extracting
                # init code at the wrong boundary (+63B kernel regression).
                self.emit("\tclr $a")
                self.emit('\texw 0 0')         # ORB = 0 (A=0)
                self.emit_ddrb(0x00)   # DDRB = 0 (both lines idle/HIGH)
                # Full 9-clock bus recovery + STOP. Replaces the older
                # 4-ddrb_imm STOP sequence (which only handled idle bus).
                # The 9-clock loop unsticks any I2C slave that got caught
                # mid-byte from a prior run (RTC, PCF8574, EEPROM).
                # Costs 2B more in i2c_init but lets __lcd_init drop its
                # own duplicate recovery (saves 10B) — net -8B per LCD
                # program. Programs that use i2c_init without lcd_init
                # pay +2B for safer init.
                self.emit('\tldi $a,9')
                lbl_br = self.label('i2c_init_br')
                self.emit(f'{lbl_br}:')
                self.emit_ddrb(0x02)   # SCL LOW, SDA released
                self.emit_ddrb(0x00)   # SCL HIGH, SDA released
                self.emit('\tdec')
                self.emit(f'\tjnz {lbl_br}')
                self.emit_ddrb(0x01)   # SDA LOW, SCL HIGH (STOP prep)
                self.emit_ddrb(0x00)   # SDA HIGH = STOP
                self.emit('\tpush $a ;!keep')  # stack warmup (STK pin settling)
                self.emit('\tpop $a ;!keep')
                return

            if name == 'i2c_bus_reset':
                # I2C bus recovery: N SCL pulses with SDA released, then STOP.
                # Clears any slave stuck mid-transaction. 9 is the standard
                # "worst case of 8 pending bits + 1 margin", but looped version
                # ~15 bytes inline — can be tuned down if space-constrained.
                self.emit('\tldi $a,9')
                lbl_br = self.label('i2cbr')
                self.emit(f'{lbl_br}:')
                self.emit_ddrb(0x02)   # SCL drive LOW, SDA INPUT
                self.emit_ddrb(0x00)   # SCL release (rises via pullup)
                self.emit('\tdec')
                self.emit(f'\tjnz {lbl_br}')
                self.emit_ddrb(0x03)   # drive both low
                self.emit_ddrb(0x01)   # SCL HIGH, SDA LOW
                self.emit_ddrb(0x00)   # SDA rises = STOP
                return

            if name == 'lcd_init':
                # Complete LCD init sequence as one jal call.
                # LCD init data is pre-populated in data page (no EEPROM read needed).
                #
                # Optional args: lcd_init() or lcd_init(r, g, b). When provided,
                # r/g/b must be compile-time constants (0..255); they replace
                # the default 0xFF/0xFF/0xFF backlight brightness values baked
                # into the init data table. No runtime cost — substitution
                # happens at emit time. Non-const args are rejected; callers
                # who need runtime RGB control should follow lcd_init() with
                # an explicit lcd_rgb(r, g, b) call.
                if args:
                    if len(args) != 3:
                        raise Exception(
                            f"lcd_init() takes 0 or 3 arguments (R, G, B), got {len(args)}")
                    rgb = []
                    for i, a in enumerate(args):
                        c = self._const_eval(a)
                        if c is None:
                            raise Exception(
                                f"lcd_init() argument {i+1} must be a "
                                f"compile-time constant (use lcd_rgb() for "
                                f"runtime RGB)")
                        c &= 0xFF
                        # Init-data walker uses 0xFB..0xFE as sentinels
                        # (END/DELAY/STOP/START). A brightness in that
                        # range would be misread as a control sentinel
                        # and terminate/derail init. Clamp into the safe
                        # range instead of silently miscompiling.
                        if 0xFB <= c <= 0xFE:
                            raise Exception(
                                f"lcd_init() RGB value 0x{c:02X} collides with "
                                f"an __lcd_init walker sentinel "
                                f"(0xFB..0xFE). Use 0..0xFA or 0xFF.")
                        rgb.append(c)
                    prev = getattr(self, '_lcd_init_rgb', None)
                    if prev is not None and prev != tuple(rgb):
                        raise Exception(
                            f"lcd_init() called with conflicting RGB "
                            f"{tuple(rgb)} vs {prev} — the init data table is "
                            f"emitted once; use lcd_rgb() for later changes")
                    self._lcd_init_rgb = tuple(rgb)
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_init')
                self._lcd_helpers.add('__i2c_sb')
                # __lcd_init's body (data-driven walker emitted at
                # mk1cc2.py:~3326) issues `jal __i2c_st_only` for START
                # sentinels, `jal __i2c_sp` for STOP sentinels, and
                # `jal __i2c_sb` for plain bytes. All three must be
                # registered as helpers so they get emitted; without these,
                # programs that call lcd_init() but no other LCD primitive
                # (e.g. test_i2c_ack_diag.c, which uses only lcd_init() +
                # raw i2c_send_byte) leave dangling jal targets and the
                # init-helper dependency resolver in _overlay_partition
                # loops forever trying to satisfy them.
                self._lcd_helpers.add('__i2c_st_only')
                self._lcd_helpers.add('__i2c_sp')
                # __i2c_rb NOT needed: LCD init data pre-populated in data page
                self.emit('\tjal __lcd_init')
                # The init data sequence we emit includes Clear Display
                # (0x01) as a command, so a program's `lcd_cmd(0x01)` /
                # `lcd_clear()` immediately after `lcd_init()` is redundant.
                # Mark the flag so the lcd_cmd builtin can elide such a call.
                self._lcd_init_just_emitted = True
                return

            if name == 'lcd_clear':
                # HD44780 Clear Display (0x01) via __lcd_cmd (shares
                # __lcd_send_raw with __lcd_chr so almost free). Inline
                # ~3ms delay avoids pulling in __delay_Nms / __delay_cal.
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_cmd')
                self._lcd_helpers.add('__lcd_send_raw')  # __lcd_cmd falls through to __lcd_send_raw — co-locate
                self._lcd_helpers.add('__i2c_sb')
                self.emit('\tldi $a,1')        # A = cmd byte (Clear) — __lcd_cmd reads caller's $a
                self.emit('\tjal __lcd_cmd')
                # ~3ms raw delay for HD44780 Clear (needs 1.52ms min).
                lbl_c = self.label('lcclr_d')
                self.emit('\tldi $a,0')
                self.emit(f'{lbl_c}:')
                self.emit('\tdec')
                self.emit(f'\tjnz {lbl_c}')
                return

            if name == 'i2c_stream':
                # i2c_stream(offset) — interpret sentinel-encoded I2C sequence
                # from page3 starting at offset. Sentinels: 0xFE=START, 0xFD=STOP, 0xFF=END.
                # Any other byte is sent via i2c_send_byte.
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_stream')
                self._lcd_helpers.add('__i2c_sb')
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        self.emit(f'\tldi $a,{c & 0xFF}')
                    else:
                        self.gen_expr(args[0])
                self.emit('\tjal __i2c_stream')
                return

            if name == 'i2c_stream_result':
                # Returns the last READ result (stored in C by __i2c_stream)
                self.emit('\tmov $c,$a')
                return

            if name == 'i2c_start':
                # Inline START
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)   # SDA LOW, SCL HIGH
                self.emit_ddrb(0x03)   # both LOW
                return

            if name == 'i2c_stop':
                # Inline STOP
                self.emit_ddrb(0x03)   # both LOW
                self.emit_ddrb(0x01)   # SDA LOW, SCL HIGH
                self.emit_ddrb(0x00)   # both HIGH (idle)
                return

            if name == 'i2c_repeated_start':
                # Repeated START via shared __i2c_rs helper. Lets user code
                # write `i2c_repeated_start()` between a register-pointer
                # write and a data read instead of `i2c_stop(); i2c_start();`
                # (saves ~5-9 B per call site by sharing the rs helper and
                # skipping the intermediate STOP).
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_rs')
                self.emit('\tjal __i2c_rs')
                return

            if name == 'i2c_send_byte':
                # Call shared __i2c_sb subroutine (byte in A)
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        self.emit(f'\tldi $a,{c & 0xFF}')
                    else:
                        self.gen_expr(args[0])
                # Ensure __i2c_sb is emitted
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self.emit('\tjal __i2c_sb')
                return

            if name == 'i2c_scan_results':
                vals = [self._const_eval(a) for a in (args or [])]
                if len(vals) != 4 or any(v is None for v in vals):
                    self.gen_error("i2c_scan_results requires constant base/start/end/capacity")
                    return
                base, start, end, cap = [v & 0xFF for v in vals]
                helper = f'__i2c_scan_{base}_{start}_{end}_{cap}'
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add(helper)
                if not hasattr(self, '_i2c_scan_helpers'):
                    self._i2c_scan_helpers = {}
                self._i2c_scan_helpers[helper] = (base, start, end, cap)
                self.emit(f'\tjal {helper}')
                return

            if name == 'lcd_hex_results':
                vals = [self._const_eval(a) for a in (args or [])]
                if len(vals) != 2 or any(v is None for v in vals):
                    self.gen_error("lcd_hex_results requires constant base/limit")
                    return
                base, limit = [v & 0xFF for v in vals]
                helper = f'__lcd_hex_results_{base}_{limit}'
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_chr')
                self._lcd_helpers.add('__lcd_send_raw')  # __lcd_chr tail-jumps to __lcd_send_raw — co-locate
                self._lcd_helpers.add('__print_u8_hex')
                self._lcd_helpers.add(helper)
                if not hasattr(self, '_lcd_hex_result_helpers'):
                    self._lcd_hex_result_helpers = {}
                self._lcd_hex_result_helpers[helper] = (base, limit)
                self.emit(f'\tjal {helper}')
                return

            if name == 'i2c_read_byte':
                # Read one I2C byte into A (8 bits MSB first, SDA released)
                # Caller must send ACK or NACK after this
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_rb')
                self.emit('\tjal __i2c_rb')
                self.emit('\tmov $d,$a')  # A = byte read (from D)
                return

            if name in ('eeprom_write_byte', 'eeprom_write'):
                # eeprom_write_byte(addr, data) — addr is 16-bit, split at compile time.
                # eeprom_write(addr_hi, addr_lo, data) is the legacy spelling.
                # Both forms also accept explicit hi/lo bytes for backwards compat.
                if not args or len(args) < 2:
                    self.gen_error("eeprom_write_byte(addr, data) or eeprom_write(addr_hi, addr_lo, data)")
                    return
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                if len(args) == 2 and name == 'eeprom_write_byte':
                    # 2-arg form: eeprom_write_byte(addr16, data)
                    addr_c = self._const_eval(args[0])
                    if addr_c is None:
                        self.gen_error("eeprom_write_byte: address must be a constant")
                        return
                    addr_hi = (addr_c >> 8) & 0xFF
                    addr_lo = addr_c & 0xFF
                    data_arg = args[1]
                else:
                    # 3-arg form: eeprom_write_byte(hi, lo, data)
                    addr_hi_c = self._const_eval(args[0])
                    addr_lo_c = self._const_eval(args[1])
                    addr_hi = addr_hi_c & 0xFF if addr_hi_c is not None else None
                    addr_lo = addr_lo_c & 0xFF if addr_lo_c is not None else None
                    data_arg = args[2]
                # START
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                # Device address (write)
                self.emit('\tldi $a,0xAE')
                self.emit('\tjal __i2c_sb')
                # Address high byte
                if addr_hi is not None:
                    self.emit(f'\tldi $a,{addr_hi}')
                else:
                    self.gen_expr(args[0])
                self.emit('\tjal __i2c_sb')
                # Address low byte
                if addr_lo is not None:
                    self.emit(f'\tldi $a,{addr_lo}')
                else:
                    self.gen_expr(args[1])
                self.emit('\tjal __i2c_sb')
                # Data byte
                c = self._const_eval(data_arg)
                if c is not None:
                    self.emit(f'\tldi $a,{c & 0xFF}')
                else:
                    self.gen_expr(data_arg)
                self.emit('\tjal __i2c_sb')
                # STOP
                self.emit_ddrb(0x03)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x00)
                # ACK poll (wait for write cycle)
                lbl = self.label('ewp')
                self.emit(f'{lbl}:')
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xAE')
                self.emit('\tjal __i2c_sb')
                self.emit('\tpush $a')
                self.emit_ddrb(0x03)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x00)
                self.emit('\tpop $a')
                self.emit('\ttst 0x01')
                self.emit(f'\tjnz {lbl}')
                return

            if name in ('eeprom_read_byte', 'eeprom_read'):
                # eeprom_read_byte(addr) — addr is 16-bit constant, returns byte in A.
                # eeprom_read(addr_hi, addr_lo) is the legacy spelling.
                if not args:
                    self.gen_error("eeprom_read_byte(addr) or eeprom_read(addr_hi, addr_lo)")
                    return
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                if len(args) == 1 and name == 'eeprom_read_byte':
                    addr_c = self._const_eval(args[0])
                    if addr_c is None:
                        self.gen_error("eeprom_read_byte: address must be a constant")
                        return
                    addr_hi = (addr_c >> 8) & 0xFF
                    addr_lo = addr_c & 0xFF
                else:
                    addr_hi_c = self._const_eval(args[0])
                    addr_lo_c = self._const_eval(args[1])
                    addr_hi = addr_hi_c & 0xFF if addr_hi_c is not None else None
                    addr_lo = addr_lo_c & 0xFF if addr_lo_c is not None else None
                # Register __i2c_rs — used for repeated START between write and read phases
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_rs')
                # Set address: START + 0xAE + addr_hi + addr_lo
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xAE')
                self.emit('\tjal __i2c_sb')
                if addr_hi is not None:
                    self.emit(f'\tldi $a,{addr_hi}')
                else:
                    self.gen_expr(args[0])
                self.emit('\tjal __i2c_sb')
                if addr_lo is not None:
                    self.emit(f'\tldi $a,{addr_lo}')
                else:
                    self.gen_expr(args[1])
                self.emit('\tjal __i2c_sb')
                # Repeated START (no STOP) + 0xAF + read byte + NACK + STOP
                self.emit('\tjal __i2c_rs')
                self.emit('\tldi $a,0xAF')
                self.emit('\tjal __i2c_sb')
                self.emit('\tjal __i2c_rb')
                self.emit('\tmov $d,$a')       # A = read byte
                # NACK + STOP (merged)
                self.emit_ddrb(0x02)
                self.emit_ddrb(0x00)
                self.emit_ddrb(0x03)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x00)
                return

            if name == 'rtc_read_seconds':
                # Read DS3231 seconds register (0x00), returns BCD in A.
                # Uses repeated START between register-pointer write and
                # data read — keeps bus owned across the two phases, more
                # robust than STOP+START and saves ~6B.
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self._lcd_helpers.add('__i2c_rs')
                # Set register pointer: START + 0xD0 + 0x00
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xD0')
                self.emit('\tjal __i2c_sb')
                self.emit('\tclr $a')
                self.emit('\tjal __i2c_sb')
                # Repeated START (no STOP) + 0xD1 + read byte + NACK + STOP
                self.emit('\tjal __i2c_rs')
                self.emit('\tldi $a,0xD1')
                self.emit('\tjal __i2c_sb')
                self.emit('\tjal __i2c_rb')
                self.emit('\tmov $d,$a')       # A=byte, D=byte backup
                self.emit_ddrb(0x02)
                self.emit_ddrb(0x00)
                self.emit_ddrb(0x02)
                self.emit_ddrb(0x03)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x00)
                return

            if name == 'rtc_read_temp':
                # Read DS3231 temperature MSB (register 0x11), returns signed °C in A.
                # Uses repeated START between register-pointer write and data read.
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self._lcd_helpers.add('__i2c_rs')
                # Set register pointer to 0x11
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xD0')
                self.emit('\tjal __i2c_sb')
                self.emit('\tldi $a,0x11')
                self.emit('\tjal __i2c_sb')
                # Repeated START (no STOP) + 0xD1 + read MSB + NACK + STOP
                self.emit('\tjal __i2c_rs')
                self.emit('\tldi $a,0xD1')
                self.emit('\tjal __i2c_sb')
                self.emit('\tjal __i2c_rb')
                self.emit('\tmov $d,$a')       # A=byte, D=byte backup
                self.emit_ddrb(0x02)
                self.emit_ddrb(0x00)
                self.emit_ddrb(0x02)
                self.emit_ddrb(0x03)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x00)
                return

            if name == 'i2c_ack':
                # Send ACK (pull SDA LOW, clock SCL once)
                self.emit_ddrb(0x03)   # SDA LOW, SCL LOW
                self.emit_ddrb(0x01)   # SDA LOW, SCL HIGH
                self.emit_ddrb(0x02)   # SDA released, SCL LOW
                return

            if name == 'i2c_nack':
                # Send NACK (SDA released/HIGH, clock SCL once)
                self.emit_ddrb(0x02)   # SDA released, SCL LOW
                self.emit_ddrb(0x00)   # SDA released, SCL HIGH
                self.emit_ddrb(0x02)   # SDA released, SCL LOW
                return

            if name == 'write_code':
                # write_code(byte, addr) — write byte to code page via istc
                # addr must be constant (reload after byte eval to avoid clobber)
                if len(args) < 2:
                    self.gen_error("write_code(byte, addr)")
                c_addr = self._const_eval(args[1])
                if c_addr is None:
                    self.gen_error("write_code: addr must be constant")
                self.gen_expr(args[0])           # byte → A (may clobber B)
                self.emit(f'\tldi $b,{c_addr & 0xFF}')  # B = addr
                self.emit('\tistc')              # code[B] = A
                return

            if name == 'eeprom_read_to_code':
                # eeprom_read_to_code(eeprom_addr, code_addr, count)
                # Bulk sequential I2C read from EEPROM → code page via istc.
                # All args must be compile-time constants.
                # Emits inline I2C setup + calls __eeprom_r2c_loop for the read loop.
                if len(args) < 3:
                    self.gen_error("eeprom_read_to_code(eeprom_addr, code_addr, count)")
                ee_addr = self._const_eval(args[0])
                code_addr = self._const_eval(args[1])
                count = self._const_eval(args[2])
                if ee_addr is None or code_addr is None or count is None:
                    self.gen_error("eeprom_read_to_code: all args must be constants")
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self._lcd_helpers.add('__eeprom_r2c_loop')
                # Inline I2C address setup (constants baked in, no data page)
                # START + device write addr + EEPROM address + repeated START + read addr
                self.emit_ddrb(0x01)       # START
                self.emit_ddrb(0x03)
                self.emit(f'\tldi $a,0xAE')        # EEPROM write
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $a,{(ee_addr >> 8) & 0xFF}')  # addr high
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $a,{ee_addr & 0xFF}')         # addr low
                self.emit('\tjal __i2c_sb')
                self.emit_ddrb(0x00)       # repeated START
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit(f'\tldi $a,0xAF')        # EEPROM read
                self.emit('\tjal __i2c_sb')
                # Set up loop: B = code_dest, count in data[7]
                self.emit(f'\tldi $b,{code_addr & 0xFF}')  # B = dest start
                self.emit(f'\tldi $a,{count & 0xFF}')
                self.emit('\tpush $a')             # count on stack
                self.emit('\tjal __eeprom_r2c_loop')
                self.emit('\tpop $a')              # clean stack
                return

            if name == 'call_code':
                # call_code(addr) — jal to a code page address
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        self.emit(f'\tjal {c}')
                    else:
                        self.gen_error("call_code requires constant address")
                return

            if name in ('ora_imm', 'orb_imm', 'ddra_imm'):
                # VIA register immediate writes — preserves all registers
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        self.emit(f'\t{name} {c & 0xFF}')
                    else:
                        self.gen_error(f"{name} requires a constant argument")
                return

            if name == 'i2c_wait_ack':
                # Poll until device ACKs (for EEPROM write cycle completion)
                # Arg: device address byte (e.g. 0xAE for EEPROM write)
                lbl = self.label('wpoll')
                if args:
                    c = self._const_eval(args[0])
                self.emit(f'{lbl}:')
                self.emit('\texrw 2')           # START
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                if c is not None:
                    self.emit(f'\tldi $a,{c & 0xFF}')
                else:
                    self.gen_expr(args[0])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self.emit('\tjal __i2c_sb')      # send addr, A = ACK bit
                self.emit('\tpush $a')
                self.emit_ddrb(0x03)     # STOP
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x00)
                self.emit('\tpop $a')
                self.emit('\ttst 0x01')          # check ACK (bit 0: 0=ACK, 1=NACK)
                self.emit(f'\tjnz {lbl}')        # retry if NACK
                return

            if name == 'delay_calibrate':
                # Run SQW-based calibration, store D/4 in data page addr 0.
                # Emits jal to __delay_cal helper (emitted once with other helpers).
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__delay_cal')
                self._explicit_delay_calibrate = True
                self.emit('\tjal __delay_cal')
                return

            if name == 'delay':
                # delay N ms using calibrated value from page3[240].
                # A = ms count. Calls __delay_Nms. Auto-inserts
                # __delay_cal into _main — otherwise page3[240] is
                # uninitialised and the delay runs an undefined duration.
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        self.emit(f'\tldi $a,{c & 0xFF}')
                    else:
                        self.gen_expr(args[0])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__delay_Nms')
                self._needs_delay_calibrate = True
                self.emit('\tmov $a,$b')    # B = ms count
                self.emit('\tjal __delay_Nms')
                return

            if name == 'tone':
                # tone(freq_hz, duration_ms) — play square wave on PA1.
                # Both args MUST be compile-time constants.
                # Stores [ratio, cyc_lo, cyc_hi] in data page note table.
                # Emits ldi $a, offset; jal __play_note. Runtime main configures
                # DDRA/PA1 after delay calibration; __tone only toggles ORA.
                if not hasattr(self, '_tone_ddra_emitted'):
                    self._tone_ddra_emitted = True
                    self._needs_tone_init = True
                if len(args) < 2:
                    raise Exception("tone() requires 2 arguments: freq_hz, duration_ms")
                freq = self._const_eval(args[0])
                dur = self._const_eval(args[1])
                if freq is None or dur is None:
                    raise Exception("tone() requires constant arguments")
                ratio = round(4000 / freq)
                if ratio > 255 or ratio < 1:
                    raise Exception(f"tone frequency {freq}Hz out of range (~16Hz-4kHz)")
                total_cycles = freq * dur // 1000
                if total_cycles < 1:
                    return  # too short to play
                cyc_hi = min((total_cycles >> 8) & 0xFF, 255)
                cyc_lo = total_cycles & 0xFF
                # Allocate 3 bytes in data page (page 1) note table
                # Deduplicate: reuse existing entry with same values
                if not hasattr(self, '_note_table'):
                    self._note_table = []
                p1_off = None
                for existing_off, er, ecl, ech in self._note_table:
                    if er == ratio and ecl == cyc_lo and ech == cyc_hi:
                        p1_off = existing_off
                        break
                if p1_off is None:
                    p1_off = self.data_alloc
                    self._note_table.append((p1_off, ratio, cyc_lo, cyc_hi))
                    self.data_alloc += 3
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__play_note')
                self._lcd_helpers.add('__tone_setup')
                self._lcd_helpers.add('__tone')
                self.emit(f'\tldi $a,{p1_off}')
                self.emit('\tjal __play_note')
                return

            if name == 'silence':
                # silence(duration_ms) — pause between notes.
                # Stores [0, ms, 0] in page 3 (ratio=0 = silence marker).
                # Same 4 bytes per call via __play_note.
                if not args:
                    return
                dur_ms = self._const_eval(args[0])
                if dur_ms is None:
                    raise Exception("silence() requires a constant argument")
                if not hasattr(self, '_note_table'):
                    self._note_table = []
                dur_byte = dur_ms & 0xFF
                p1_off = None
                for existing_off, er, ecl, ech in self._note_table:
                    if er == 0 and ecl == dur_byte and ech == 0:
                        p1_off = existing_off
                        break
                if p1_off is None:
                    p1_off = self.data_alloc
                    self._note_table.append((p1_off, 0, dur_byte, 0))
                    self.data_alloc += 3
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__play_note')
                self._lcd_helpers.add('__delay_Nms')
                # __play_note's body unconditionally issues `jal __tone`
                # (mk1cc2.py:~3892) regardless of which note-table entry
                # is being played. silence()-only programs (no tone() call)
                # would otherwise emit a dangling jal target. Same family
                # as the lcd_init / __i2c_st_only registration miss fixed
                # earlier this session.
                self._lcd_helpers.add('__tone')
                self.emit(f'\tldi $a,{p1_off}')
                self.emit('\tjal __play_note')
                return

            if name == 'lcd_rgb':
                # lcd_rgb(red, green, blue) — for DFRobot RGB LCD V2.0 (0x2D)
                if len(args) < 3:
                    raise Exception("lcd_rgb() requires 3 arguments: r, g, b")
                # Push blue (arg2), green in $b (arg1), red in $a (arg0)
                if args[2][0] == 'num':
                    self.emit(f'\tpush_imm {args[2][1] & 0xFF}')
                else:
                    self.gen_expr(args[2])
                    self.emit('\tpush $a')
                self.local_count += 1
                if args[1][0] == 'num':
                    self.emit(f'\tldi $b,{args[1][1] & 0xFF}')
                else:
                    self.gen_expr(args[1])
                    self.emit('\tmov $a,$b')
                if args[0][0] == 'num':
                    self.emit(f'\tldi $a,{args[0][1] & 0xFF}')
                else:
                    self.gen_expr(args[0])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_rgb')
                self._lcd_helpers.add('__i2c_st_only')
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_sp')
                self.emit('\tjal __lcd_rgb')
                self.emit('\tpop $d')  # clean blue
                self.local_count -= 1
                return

            if name == 'lcd_cmd' or name == 'lcd_char':
                # LCD command (RS=0) or character (RS=1) via I2C to AiP31068L (0x3E).
                # Emits a jal to a generated helper function.
                is_char = (name == 'lcd_char')
                # Peephole: lcd_cmd(0x01)/(0x02) immediately after lcd_init() is
                # redundant — the init data sequence already issues Clear Display.
                if args and not is_char and getattr(self, '_lcd_init_just_emitted', False):
                    c_peep = self._const_eval(args[0])
                    if c_peep is not None and c_peep in (0x01, 0x02):
                        self._lcd_init_just_emitted = False
                        return
                self._lcd_init_just_emitted = False
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        self.emit(f'\tldi $a,{c & 0xFF}')  # char in A
                    else:
                        self.gen_expr(args[0])              # result in A
                helper = f'__lcd_{"chr" if is_char else "cmd"}'
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add(helper)
                self._lcd_helpers.add('__lcd_send_raw')
                self._lcd_helpers.add('__i2c_st_only')
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_sp')
                self.emit(f'\tjal {helper}')
                # HD44780 execution delay after lcd_cmd (not needed for lcd_char).
                if not is_char:
                    if c is not None and c in (0x01, 0x02):
                        lbl_c = self.label('lcd_cmd_d')
                        self.emit('\tldi $a,0')
                        self.emit(f'{lbl_c}:')
                        self.emit('\tdec')
                        self.emit(f'\tjnz {lbl_c}')
                return

            if name == 'lcd_print':
                # lcd_print(string or page3-offset) — gen_expr's string case
                # allocates + dedupes; a bare integer offset just lands in $a.
                if args:
                    self.gen_expr(args[0])
                    if not hasattr(self, '_lcd_helpers'):
                        self._lcd_helpers = set()
                    self._lcd_helpers.add('__lcd_print')
                    self._lcd_helpers.add('__lcd_chr')
                    self._lcd_helpers.add('__lcd_send_raw')  # __lcd_chr tail-jumps to __lcd_send_raw — co-locate
                    self.emit('\tjal __lcd_print')
                return

            if name == 'lcd_temp_u8':
                if args:
                    self.gen_expr(args[0])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_temp_u8')
                self._lcd_helpers.add('__lcd_chr')
                self._lcd_helpers.add('__lcd_send_raw')  # __lcd_chr tail-jumps to __lcd_send_raw — co-locate
                self.emit('\tjal __lcd_temp_u8')
                return

            if name == 'lcd_hex_u8':
                if args:
                    self.gen_expr(args[0])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__print_u8_hex')
                self._lcd_helpers.add('__lcd_chr')
                self._lcd_helpers.add('__lcd_send_raw')  # __lcd_chr tail-jumps to __lcd_send_raw — co-locate
                self.emit('\tjal __print_u8_hex')
                return

            if name == 'lcd_bcd_u8':
                if args:
                    self.gen_expr(args[0])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_bcd_u8')
                self._lcd_helpers.add('__lcd_chr')
                self._lcd_helpers.add('__lcd_send_raw')  # __lcd_chr tail-jumps to __lcd_send_raw — co-locate
                self.emit('\tjal __lcd_bcd_u8')
                return

            if name == 'printf':
                # Compile-time format-string expansion. Targets the LCD via
                # existing __lcd_print / __lcd_chr / __print_u8_dec / __print_u8_hex.
                # Supported specifiers: %d (u8 decimal, 3 digits), %x (u8 hex,
                # 2 digits), %c (char), %s (string or page3 offset), %% (literal
                # '%'). Escape \xNN in the literal text passes through as-is to
                # the LCD — use 0xDF for the HD44780 degree symbol, etc.
                if not args or args[0][0] != 'string':
                    self.gen_error("printf requires a string literal as first argument")
                    return
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_chr')
                self._lcd_helpers.add('__lcd_send_raw')  # __lcd_chr tail-jumps to __lcd_send_raw — co-locate
                fmt = args[0][1]
                temp_marker = '%d\xDFC'
                if (len(args) >= 2 and fmt.count('%') == 1
                        and temp_marker in fmt):
                    prefix, suffix = fmt.split(temp_marker, 1)
                    def _emit_lit(s):
                        if not s:
                            return
                        if len(s) == 1:
                            self.emit(f'\tldi $a,{ord(s) & 0xFF}')
                            self.emit('\tjal __lcd_chr')
                        else:
                            self.gen_expr(('string', s))
                            self._lcd_helpers.add('__lcd_print')
                            self.emit('\tjal __lcd_print')
                    _emit_lit(prefix)
                    self.gen_expr(args[1])
                    self._lcd_helpers.add('__lcd_temp_u8')
                    self._lcd_helpers.add('__lcd_chr')
                    self._lcd_helpers.add('__lcd_send_raw')  # __lcd_chr tail-jumps to __lcd_send_raw — co-locate
                    self.emit('\tjal __lcd_temp_u8')
                    _emit_lit(suffix)
                    if len(args) > 2:
                        self.emit(f'; printf: {len(args) - 2} extra argument(s) ignored')
                    return
                arg_idx = 1
                buf = []       # accumulating literal chunk

                def _flush():
                    if not buf:
                        return
                    s = ''.join(buf)
                    buf.clear()
                    if len(s) == 1:
                        # one-char literal → lcd_char(ch) is 3+3B, cheaper than lcd_print
                        self.emit(f'\tldi $a,{ord(s) & 0xFF}')
                        self.emit('\tjal __lcd_chr')
                    else:
                        # stash the multi-char literal as a string expr → gen_expr
                        self.gen_expr(('string', s))
                        self._lcd_helpers.add('__lcd_print')
                        self.emit('\tjal __lcd_print')

                i = 0
                while i < len(fmt):
                    ch = fmt[i]
                    if ch != '%':
                        buf.append(ch)
                        i += 1
                        continue
                    i += 1
                    if i >= len(fmt):
                        self.gen_error("printf: trailing '%' in format string")
                        return
                    spec = fmt[i]
                    i += 1
                    if spec == '%':
                        buf.append('%')
                        continue
                    _flush()
                    if arg_idx >= len(args):
                        self.gen_error(f"printf: not enough arguments for '%{spec}'")
                        return
                    arg = args[arg_idx]
                    arg_idx += 1
                    if spec == 'd':
                        self.gen_expr(arg)
                        self._lcd_helpers.add('__print_u8_dec')
                        self.emit('\tjal __print_u8_dec')
                    elif spec == 'x':
                        self.gen_expr(arg)
                        self._lcd_helpers.add('__print_u8_hex')
                        self.emit('\tjal __print_u8_hex')
                    elif spec == 'c':
                        self.gen_expr(arg)
                        self.emit('\tjal __lcd_chr')
                    elif spec == 's':
                        self.gen_expr(arg)   # string literal → offset, or integer offset
                        self._lcd_helpers.add('__lcd_print')
                        self.emit('\tjal __lcd_print')
                    else:
                        self.gen_error(f"printf: unsupported specifier '%{spec}'")
                        return
                _flush()
                if arg_idx < len(args):
                    # Not fatal — just warn via emit comment. Keeps compilation going.
                    self.emit(f'; printf: {len(args) - arg_idx} extra argument(s) ignored')
                return

            # peek/poke for page 3 (convenience wrappers)
            if name == 'peek3':
                if args:
                    self.gen_expr(args[0])
                self.emit('\tderefp3')
                return
            if name == 'poke3':
                if len(args) >= 2:
                    self.gen_expr(args[0])       # value
                    self.emit('\tpush $a')
                    self.local_count += 1
                    self.gen_expr(args[1])       # address
                    self.emit('\tmov $a,$b')      # B = address
                    self.emit('\tpop $a')
                    self.local_count -= 1
                    self.emit('\tiderefp3')       # page3[B] = A
                return

            nparams = self.func_params.get(name, len(args))
            use_regcall = self._use_regcall(nparams) and len(args) == nparams

            if use_regcall:
                # Hybrid: push stack args (index 2+) right-to-left first
                stack_args = args[2:]
                for arg in reversed(stack_args):
                    if arg[0] == 'num':
                        self.emit(f'\tpush_imm {arg[1] & 0xFF}')
                    else:
                        self.gen_expr(arg)
                        self.emit('\tpush $a')
                    self.local_count += 1

                # Then load B (arg 1), then A (arg 0) — B first to preserve A
                if len(args) >= 2:
                    if args[1][0] == 'num':
                        self.emit(f'\tldi $b,{args[1][1] & 0xFF}')
                    else:
                        self.gen_expr(args[1])
                        self.emit('\tmov $a,$b')
                if len(args) >= 1:
                    if args[0][0] == 'num':
                        self.emit(f'\tldi $a,{args[0][1] & 0xFF}')
                    else:
                        self.gen_expr(args[0])

                self.emit(f'\tjal _{name}')
                # Pop only stack args
                for _ in stack_args:
                    self.emit('\tpop $d')
                    self.local_count -= 1
            else:
                # Stack calling: push all args right-to-left
                for arg in reversed(args):
                    if arg[0] == 'num':
                        self.emit(f'\tpush_imm {arg[1] & 0xFF}')
                    else:
                        self.gen_expr(arg)
                        self.emit('\tpush $a')
                    self.local_count += 1
                self.emit(f'\tjal _{name}')
                for _ in args:
                    self.emit('\tpop $d')
                    self.local_count -= 1

        elif kind == 'index':
            base_expr, idx_expr = expr[1], expr[2]
            # Fast path: constant index with non-EEPROM array. Fold base+index
            # at compile time → single `ldi $a,addr` (2B) vs `ldi $a,idx; addi
            # base,$a` (4B). Saves 2B per fold.
            const_idx = self._const_eval(idx_expr)
            if (const_idx is not None
                and base_expr[0] == 'var'
                and base_expr[1] not in self.eeprom_globals):
                if base_expr[1] in self.page3_globals:
                    base = self.page3_globals[base_expr[1]]
                    addr = (base + const_idx) & 0xFF
                    self.emit(f'\tldi $a,{addr}')
                    self.emit('\tderefp3')
                    return
                if base_expr[1] in self.globals:
                    base = self.globals[base_expr[1]]
                    addr = (base + const_idx) & 0xFF
                    self.emit(f'\tldi $a,{addr}')
                    self.emit('\tderef')
                    return
            self.gen_expr(idx_expr)
            if base_expr[0] == 'var' and base_expr[1] in self.eeprom_globals:
                # EEPROM array: A = index, read EEPROM[base + A]
                self._needs_runtime_eeprom_rd = True
                base = self.eeprom_globals[base_expr[1]]
                # __eeprom_rd takes 16-bit address in B:A (hi:lo)
                # base is 16-bit EEPROM address; index is 8-bit
                lo = base & 0xFF
                hi = (base >> 8) & 0xFF
                if lo:
                    self.emit(f'\taddi {lo},$a')  # A = lo + index
                self.emit(f'\tldi $b,{hi}')       # B = hi byte
                # Handle carry from lo + index
                self.emit('\tjnc .noc%d' % self.label_id)
                self.emit('\tmov $b,$a')
                self.emit('\tinc')
                self.emit('\tmov $a,$b')
                self.emit('.noc%d:' % self.label_id)
                self.label_id += 1
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__eeprom_rd')
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self._lcd_helpers.add('__i2c_rs')
                self.emit('\tjal __eeprom_rd')
            elif base_expr[0] == 'var' and base_expr[1] in self.page3_globals:
                base = self.page3_globals[base_expr[1]]
                if base:
                    self.emit(f'\taddi {base},$a')
                self.emit('\tderefp3')  # page 3 array read
            else:
                if base_expr[0] == 'var' and base_expr[1] in self.globals:
                    base = self.globals[base_expr[1]]
                    if base:
                        self.emit(f'\taddi {base},$a')
                self.emit('\tderef')  # page 1 array read

        elif kind == 'postinc':
            var = expr[1]
            self.gen_expr(var)
            self.emit('\tpush $a')
            self.local_count += 1
            self.emit('\tinc')
            self._store(var)
            self.emit('\tpop $a')
            self.local_count -= 1

        elif kind == 'postdec':
            var = expr[1]
            self.gen_expr(var)
            self.emit('\tpush $a')
            self.local_count += 1
            self.emit('\tdec')
            self._store(var)
            self.emit('\tpop $a')
            self.local_count -= 1

        elif kind == 'preinc':
            var = expr[1]
            self.gen_expr(var)
            self.emit('\tinc')
            self._store(var)

        elif kind == 'predec':
            var = expr[1]
            self.gen_expr(var)
            self.emit('\tdec')
            self._store(var)

        elif kind == 'log_and':
            # Short-circuit: if left is false, skip right
            _, left, right = expr
            false_l = self.label('and_f')
            end_l = self.label('and_end')
            self.gen_expr(left)
            self.emit('\tcmp 0')
            self.emit(f'\tjz {false_l}')
            self.gen_expr(right)
            self.emit('\tcmp 0')
            self.emit(f'\tjz {false_l}')
            self.emit('\tldi $a,1')
            self.emit(f'\tj {end_l}')
            self.emit(f'{false_l}:')
            self.emit('\tclr $a')
            self.emit(f'{end_l}:')

        elif kind == 'log_or':
            # Short-circuit: if left is true, skip right
            _, left, right = expr
            true_l = self.label('or_t')
            end_l = self.label('or_end')
            self.gen_expr(left)
            self.emit('\tcmp 0')
            self.emit(f'\tjnz {true_l}')
            self.gen_expr(right)
            self.emit('\tcmp 0')
            self.emit(f'\tjnz {true_l}')
            self.emit('\tclr $a')
            self.emit(f'\tj {end_l}')
            self.emit(f'{true_l}:')
            self.emit('\tldi $a,1')
            self.emit(f'{end_l}:')

        elif kind == 'ternary':
            _, cond, then, els = expr
            else_l = self.label('tern_e')
            end_l = self.label('tern_end')
            self.gen_branch_false(cond, else_l)
            self.gen_expr(then)
            self.emit(f'\tj {end_l}')
            self.emit(f'{else_l}:')
            self.gen_expr(els)
            self.emit(f'{end_l}:')

    def _emit_binop(self, op):
        if op == '+': self.emit('\tadd $b,$a')
        elif op == '-': self.emit('\tsub $b,$a')
        elif op == '&': self.emit('\tand $b,$a')
        elif op == '|': self.emit('\tor $b,$a')
        elif op == '^': self.emit('\txor')
        elif op == '*': self._emit_multiply()
        elif op == '/': self._emit_divide()
        elif op == '%': self._emit_modulo()

    def _emit_multiply(self):
        loop, done = self.label('mul'), self.label('muldone')
        self.emit('\tmov $b,$c')
        self.emit('\tmov $a,$b')
        self.emit('\tldi $a,0')
        self.emit(f'{loop}:')
        self.emit('\tpush $a')
        self.emit('\tmov $c,$a')
        self.emit('\tcmp 0')
        self.emit('\tpop $a')
        self.emit(f'\tjz {done}')
        self.emit('\tadd $b,$a')
        self.emit('\tpush $a')
        self.emit('\tmov $c,$a')
        self.emit('\tdec')
        self.emit('\tmov $a,$c')
        self.emit('\tpop $a')
        self.emit(f'\tj {loop}')
        self.emit(f'{done}:')

    def _emit_divide(self):
        """Software divide: A = A / B."""
        loop, done = self.label('div'), self.label('divdone')
        self.emit('\tmov $a,$c')    # C = dividend (working copy)
        self.emit('\tclr $a')
        self.emit('\tmov $a,$d')    # D = 0 (quotient)
        self.emit(f'{loop}:')
        self.emit('\tmov $c,$a')
        self.emit('\tcmp $b')       # C < B? done
        self.emit(f'\tjnc {done}')  # CF=0 means C < B
        self.emit('\tsub $b,$a')    # A = C - B
        self.emit('\tmov $a,$c')    # C = C - B
        self.emit('\tmov $d,$a')
        self.emit('\tinc')
        self.emit('\tmov $a,$d')    # D++
        self.emit(f'\tj {loop}')
        self.emit(f'{done}:')
        self.emit('\tmov $d,$a')    # A = quotient

    def _emit_modulo(self):
        """Software modulo: A = A % B."""
        loop, done = self.label('mod'), self.label('moddone')
        self.emit(f'{loop}:')
        self.emit('\tcmp $b')       # A < B? done (A is the remainder)
        self.emit(f'\tjnc {done}')  # CF=0 means A < B
        self.emit('\tsub $b,$a')    # A = A - B
        self.emit(f'\tj {loop}')
        self.emit(f'{done}:')

    def _store(self, target):
        """Store A to variable. Register vars use mov; stack vars use stsp.
        After stsp, D holds the original A value (microcode saves A→D first).
        Recovery via mov $d,$a (1 byte) instead of ldsp N (2 bytes)."""
        if target[0] == 'var':
            name = target[1]
            if name in self.locals:
                loc = self.locals[name]
                if loc[0] == 'reg':
                    self.emit(f'\tmov $a,${loc[1]}')
                    return  # no clobber, A preserved
                elif loc[0] == 'regparam':
                    if loc[1] != 'a':
                        self.emit(f'\tmov $a,${loc[1]}')
                    return  # A preserved (or is the target)
                elif loc[0] == 'dead':
                    return  # don't store to dead vars
                off = self._sp_offset(name)
                # stsp clobbers D (microcode: D = A before write).
                # If D holds a parameter or register variable, save/restore it.
                # Must preserve BOTH A (value to store) AND D (register var).
                d_in_use = any(v == ('reg', 'd') for v in self.locals.values())
                if d_in_use:
                    # Save A to B, save D via stack, restore A, do stsp, restore D
                    self.emit('\tpush_b')      # save B (might be in use)
                    self.emit('\tmov $a,$b')   # B = value to store (save A)
                    self.emit('\tmov $d,$a')   # A = D (register var)
                    self.emit('\tpush $a')     # push D value
                    self.local_count += 2      # two extra stack entries
                    off = self._sp_offset(name)
                    self.emit('\tmov $b,$a')   # A = value to store (from B)
                    self.emit(f'\tstsp {off}') # store value (D clobbered)
                    self.emit('\tpop $a')      # A = saved D
                    self.emit('\tmov $a,$d')   # D = restored
                    self.emit('\tpop_b')       # B = restored
                    self.local_count -= 2
                else:
                    self.emit(f'\tstsp {off}')
                    self.emit('\tmov $d,$a')   # D = original A after stsp
            elif name in self.globals:
                self.emit(f'\tst $a,_{name}')
            elif name in self.page3_globals:
                self.emit(f'\tstp3 _{name}')
            elif name in self.locals and self.locals[name][0] == 'local16':
                # 16-bit store: A = low byte, B = high byte (from _gen_expr16)
                # If the source expression was 8-bit, B might not be set — zero it
                off_lo = self._sp_offset_raw(name, 0)
                off_hi = self._sp_offset_raw(name, 1)
                self.emit(f'\tstsp {off_lo}')   # store low byte (A clobbered, D=lo)
                self.emit('\tmov $b,$a')         # A = high byte
                self.emit(f'\tstsp {off_hi}')   # store high byte
                self.emit('\tmov $d,$a')         # recover

    # ── Comparisons (branch and value) ───────────────────────────────

    def _normalize_cmp(self, op, left, right):
        """Normalize comparison to use single-jump forms.

        MK1 has jc (CF=1 → A>=B), jnc (CF=0 → A<B), jz (ZF=1), jnz (ZF=0).
        ==, !=, >=, < all need one jump. > and <= need two.

        Fix: swap operands to convert > into < and <= into >=.
        For immediate >: use >= (val+1) when val < 255.
        """
        if op == '>':
            if right[0] == 'num' and right[1] < 255:
                # a > N  ↔  a >= (N+1)
                return '>=', left, ('num', right[1] + 1)
            else:
                # a > b  ↔  b < a  (swap operands)
                return '<', right, left
        elif op == '<=':
            if right[0] == 'num' and right[1] < 255:
                # a <= N  ↔  a < (N+1)
                return '<', left, ('num', right[1] + 1)
            else:
                # a <= b  ↔  b >= a  (swap operands)
                return '>=', right, left
        return op, left, right

    def _b_has(self, expr):
        """Check if B currently holds the value of expr."""
        return self.b_expr is not None and self.b_expr == expr

    def _emit_cmp_operands(self, left, right):
        """Load operands and emit cmp. Left → A, right → B or imm."""
        if right[0] == 'num':
            self.gen_expr(left)
            self.emit(f'\tcmp {right[1] & 0xFF}')
        elif (right[0] == 'var' and right[1] in self.locals
              and self.locals[right[1]] == ('regparam', 'b')):
            # Right is regparam in B — don't move it, just load left
            self.gen_expr(left)
            self.emit('\tcmp $b')
        elif self._b_has(right):
            self.gen_expr(left)
            self.emit('\tcmp $b')
        elif (right[0] == 'var' and right[1] in self.locals
              and self.locals[right[1]][0] in ('param', 'local')):
            off = self._sp_offset(right[1])
            self.emit(f'\tldsp_b {off}')
            self.b_expr = right
            self.gen_expr(left)
            self.emit('\tcmp $b')
        else:
            self.gen_expr(right)
            self.emit('\tmov $a,$b')
            self.b_expr = right
            self.gen_expr(left)
            self.emit('\tcmp $b')

    def gen_branch_false(self, cond, target):
        """Branch to target if cond is false."""
        # Constant condition: eliminate branch entirely
        const = self._const_eval(cond)
        if const is not None:
            if not const:
                self.emit(f'\tj {target}')  # always false → always skip
            # else: always true → no branch needed, fall through
            return

        # Short-circuit &&: branch to target if EITHER is false
        if cond[0] == 'log_and':
            self.gen_branch_false(cond[1], target)
            self.gen_branch_false(cond[2], target)
            return

        # Short-circuit ||: branch to target only if BOTH are false
        if cond[0] == 'log_or':
            true_l = self.label('or_ok')
            self.gen_branch_true(cond[1], true_l)
            self.gen_branch_false(cond[2], target)
            self.emit(f'{true_l}:')
            return

        # IRQ check: if (irq0()) / if (irq1()) → je0/je1 directly (2 bytes)
        if (cond[0] == 'call' and cond[1] in ('irq0', 'irq1')):
            instr = 'je0' if cond[1] == 'irq0' else 'je1'
            skip = self.label('irq_skip')
            self.emit(f'\t{instr} {skip}')
            self.emit(f'\tj {target}')
            self.emit(f'{skip}:')
            return

        # Bit test: (x & CONST) → tst CONST + jz/jnz (preserves A)
        if (cond[0] == 'binop' and cond[1] == '&'
                and cond[3][0] == 'num'):
            # (expr & N) used as boolean: true if any bits set, false if zero
            self.gen_expr(cond[2])
            self.emit(f'\ttst {cond[3][1] & 0xFF}')
            self.emit(f'\tjz {target}')
            return
        if (cond[0] == 'binop' and cond[1] == '&'
                and cond[2][0] == 'num'):
            self.gen_expr(cond[3])
            self.emit(f'\ttst {cond[2][1] & 0xFF}')
            self.emit(f'\tjz {target}')
            return

        # Negation: flip branch direction
        if cond[0] == 'unop' and cond[1] == '!':
            self.gen_branch_true(cond[2], target)
            return

        if cond[0] == 'binop' and cond[1] in ('==', '!=', '<', '>', '<=', '>='):
            op, left, right = cond[1], cond[2], cond[3]

            # Optimization: x > 0 → tst 0xFF; jz (avoids cmp which clobbers B)
            # Also handles x != 0 and x == 0 similarly.
            if right[0] == 'num' and right[1] == 0 and op in ('>', '!='):
                self.gen_expr(left)
                self.emit(f'\ttst 0xFF')
                self.emit(f'\tjz {target}')
                return
            if right[0] == 'num' and right[1] == 0 and op == '==':
                self.gen_expr(left)
                self.emit(f'\ttst 0xFF')
                self.emit(f'\tjnz {target}')
                return
            if left[0] == 'num' and left[1] == 0 and op in ('<', '!='):
                self.gen_expr(right)
                self.emit(f'\ttst 0xFF')
                self.emit(f'\tjz {target}')
                return
            if left[0] == 'num' and left[1] == 0 and op == '==':
                self.gen_expr(right)
                self.emit(f'\ttst 0xFF')
                self.emit(f'\tjnz {target}')
                return

            if not self._b_has(right):
                op, left, right = self._normalize_cmp(op, left, right)
            self._emit_cmp_operands(left, right)

            if op == '==':   self.emit(f'\tjnz {target}')
            elif op == '!=': self.emit(f'\tjz {target}')
            elif op == '>=': self.emit(f'\tjnc {target}')
            elif op == '<':  self.emit(f'\tjc {target}')
            elif op == '>':
                self.emit(f'\tjz {target}')
                self.emit(f'\tjnc {target}')
            elif op == '<=':
                skip = self.label('skip')
                self.emit(f'\tjz {skip}')
                self.emit(f'\tjnc {target}')
                self.emit(f'{skip}:')
        # Predecrement truthiness: dec already sets ZF, no cmp needed
        elif cond[0] == 'predec' and cond[1][0] == 'var':
            name = cond[1][1]
            if name in self.locals:
                loc = self.locals[name]
                if loc[0] == 'reg':
                    self.emit(f'\tmov ${loc[1]},$a')
                    self.emit('\tdec')
                    self.emit(f'\tmov $a,${loc[1]}')
                    self.emit(f'\tjz {target}')
                    return
            # Fall through to general case for stack vars
            self.gen_expr(cond)
            self.emit('\tcmp 0')
            self.emit(f'\tjz {target}')
        else:
            self.gen_expr(cond)
            self.emit('\tcmp 0')
            self.emit(f'\tjz {target}')

    def gen_branch_true(self, cond, target):
        """Branch to target if cond is true (inverse of gen_branch_false)."""
        const = self._const_eval(cond)
        if const is not None:
            if const:
                self.emit(f'\tj {target}')
            return

        # IRQ check: if (irq0()) → je0 target (2 bytes, optimal)
        if cond[0] == 'call' and cond[1] in ('irq0', 'irq1'):
            instr = 'je0' if cond[1] == 'irq0' else 'je1'
            self.emit(f'\t{instr} {target}')
            return

        if cond[0] == 'log_and':
            skip = self.label('and_skip')
            self.gen_branch_false(cond[1], skip)
            self.gen_branch_true(cond[2], target)
            self.emit(f'{skip}:')
            return

        if cond[0] == 'log_or':
            self.gen_branch_true(cond[1], target)
            self.gen_branch_true(cond[2], target)
            return

        if cond[0] == 'unop' and cond[1] == '!':
            self.gen_branch_false(cond[2], target)
            return

        # Invert comparison for true-branch
        if cond[0] == 'binop' and cond[1] in ('==', '!=', '<', '>', '<=', '>='):
            inv = {'==':'!=', '!=':'==', '<':'>=', '>':'<=', '<=':'>', '>=':'<'}
            inv_cond = ('binop', inv[cond[1]], cond[2], cond[3])
            self.gen_branch_false(inv_cond, target)
            return

        # Predecrement truthiness: dec sets ZF, branch directly with jnz
        if cond[0] == 'predec' and cond[1][0] == 'var':
            name = cond[1][1]
            if name in self.locals:
                loc = self.locals[name]
                if loc[0] == 'reg':
                    self.emit(f'\tmov ${loc[1]},$a')
                    self.emit('\tdec')
                    self.emit(f'\tmov $a,${loc[1]}')
                    self.emit(f'\tjnz {target}')
                    return

        self.gen_expr(cond)
        self.emit('\tcmp 0')
        self.emit(f'\tjnz {target}')

    def _gen_compare_value(self, op, left, right):
        """Generate comparison that produces 0/1 in A."""
        op2, left2, right2 = self._normalize_cmp(op, left, right)
        self._emit_cmp_operands(left2, right2)
        if op2 == '==':   self.emit('\tsetz')
        elif op2 == '!=': self.emit('\tsetnz')
        elif op2 == '>=': self.emit('\tsetc')
        elif op2 == '<':  self.emit('\tsetnc')


# ── Peephole optimizer ───────────────────────────────────────────────

def peephole(lines):
    """Multi-pass peephole optimizer with register tracking."""

    def is_label(l): return l and not l.startswith('\t') and l.endswith(':')
    def is_section(l): return l and ('section ' in l or l.strip().startswith('org '))
    def is_instr(l): return l and l.startswith('\t') and not is_section(l)
    def mnemonic(l): return l.strip().split()[0] if is_instr(l) else None

    TERMINATORS = {'ret', 'hlt', 'j'}
    CLOBBERS_A = {'ldi', 'ldsp', 'ldsp_b', 'pop', 'push_imm', 'ld',
                  'add', 'sub', 'or', 'and', 'xor', 'not', 'neg',
                  'inc', 'dec', 'sll', 'slr', 'rll', 'rlr', 'swap',
                  'setz', 'setnz', 'setc', 'setnc', 'deref', 'derefp3',
                  'addi', 'subi', 'andi', 'ori', 'ldp3', 'stsp', 'clr',
                  'adc', 'sbc', 'exr', 'exrw',
                  'istc_inc', 'deref2'}

    # ── Pass 1: Dead code elimination ────────────────────────────────
    # IR-based: walk each chunk, drop items after a terminator until
    # the next label (or section marker boundary). Byte-identical to
    # the prior flat-list implementation, validated against corpus.
    import mk1ir as _ir_p1
    _prog = _ir_p1.parse_program(lines)
    _ir_p1.dead_code_elim(_prog, terminators=TERMINATORS)
    lines = _ir_p1.serialize_program(_prog)

    # ── Pass 2: Register-tracking optimization ───────────────────────
    # Track what's known to be in A, B, C, D. Eliminate redundant loads/movs.

    REG_NAMES = {'$a': 'a', '$b': 'b', '$c': 'c', '$d': 'd'}
    regs = {'a': None, 'b': None, 'c': None, 'd': None}
    _uid = [0]

    def fresh_unknown():
        """Unique unknown value — tracks register equality without knowing content."""
        _uid[0] += 1
        return ('unk', _uid[0])

    def shift_sp(v, delta):
        if v and v[0] == 'sp':
            return ('sp', v[1] + delta)
        return v

    def shift_all(delta):
        for r in regs:
            regs[r] = shift_sp(regs[r], delta)

    def invalidate_all():
        for r in regs:
            regs[r] = fresh_unknown()

    def find_reg_with(val):
        """Find any register (prefer B,C,D over A) that holds val."""
        if val is None or (isinstance(val, tuple) and val[0] == 'unk'):
            return None  # don't substitute unknown values
        for r in ('b', 'c', 'd', 'a'):
            if regs[r] == val:
                return r
        return None

    out = []
    for line in lines:
        if is_label(line):
            # Any label could be a jump target (loop back-edge, branch).
            # Must invalidate ALL register tracking — subroutine calls in
            # the loop body clobber registers that the peephole can't see.
            invalidate_all()
            out.append(line)
            continue
        if not is_instr(line):
            out.append(line)
            continue

        mn = mnemonic(line)
        parts = line.strip().split()

        # ldsp_b N: loads B from stack, clobbers A
        if mn == 'ldsp_b' and len(parts) == 2:
            try:
                off = int(parts[1])
            except ValueError:
                off = None
            out.append(line)
            regs['b'] = ('sp', off) if off is not None else None
            regs['a'] = fresh_unknown()  # ldsp_b clobbers A
            continue

        # ldsp N: skip if A already has it, or use mov $R,$a if another reg has it
        if mn == 'ldsp' and len(parts) == 2:
            try:
                off = int(parts[1])
            except ValueError:
                off = None
            target_val = ('sp', off) if off is not None else None
            if target_val and regs['a'] == target_val:
                continue  # A already has it
            if target_val:
                src = find_reg_with(target_val)
                if src and src != 'a':
                    out.append(f'\tmov ${src},$a')  # 1 byte vs 2
                    regs['a'] = target_val
                    continue
            out.append(line)
            regs['a'] = target_val
            continue

        # ldi $X,N: skip if register already holds N
        if mn == 'ldi' and ',' in line:
            parts_ldi = line.strip().split(',')
            if len(parts_ldi) == 2:
                reg_name = parts_ldi[0].split()[-1]  # e.g. "$c" from "ldi $c"
                r = REG_NAMES.get(reg_name)
                try:
                    val = int(parts_ldi[1])
                except ValueError:
                    val = None
                target_val = ('const', val) if val is not None else None
                if r and target_val and regs[r] == target_val:
                    continue  # register already has this value
                out.append(line)
                if r:
                    regs[r] = target_val
                continue

        # clr $a: A = 0 (only clr $a is safe — ALU always uses A)
        # clr $b/$c/$d actually compute X = A - X, NOT X = 0!
        if line.strip() == 'clr $a':
            if regs['a'] == ('const', 0):
                continue
            out.append(line)
            regs['a'] = ('const', 0)
            continue
        if mn == 'clr':
            # clr $b/$c/$d — result depends on A, treat as unknown
            out.append(line)
            r = REG_NAMES.get(parts[1] if len(parts) > 1 else '', None)
            if r:
                regs[r] = fresh_unknown()
            continue

        # mov $X,$Y: skip if Y already has X's value
        if mn == 'mov' and len(parts) >= 2:
            # Parse "mov $src,$dst" or "mov $src $dst"
            tok = line.strip()[3:].replace(',', ' ').split()
            if len(tok) == 2 and tok[0] in REG_NAMES and tok[1] in REG_NAMES:
                src_r = REG_NAMES[tok[0]]
                dst_r = REG_NAMES[tok[1]]
                if regs[dst_r] == regs[src_r]:
                    continue  # dst already has src's value
                out.append(line)
                regs[dst_r] = regs[src_r]
                continue
            # Fall through for non-register movs
            out.append(line)
            regs['a'] = fresh_unknown()  # conservative
            continue

        # cmp: does NOT modify any register
        if mn == 'cmp':
            out.append(line)
            continue

        # Conditional jumps: don't modify registers
        if mn in ('jc', 'jnc', 'jz', 'jnz', 'je0', 'je1'):
            out.append(line)
            continue

        # Instructions that don't modify any register
        # tst: doesn't physically modify A, but we invalidate A tracking
        # to prevent the optimizer from removing subsequent loads that
        # depend on A's value in post-branch code
        if mn == 'tst':
            out.append(line)
            regs['a'] = fresh_unknown()
            continue

        if mn in ('out', 'nop', 'hlt', 'ret', 'out_imm',
                  'istc', 'ideref', 'iderefp3', 'exw', 'stp3'):
            out.append(line)
            continue

        # push $X: SP shifts, pushed register unchanged
        if mn == 'push' and len(parts) == 2 and parts[1] in REG_NAMES:
            out.append(line)
            shift_all(1)
            continue

        # push_imm N: A = N, SP shifts
        if mn == 'push_imm' and len(parts) == 2:
            out.append(line)
            shift_all(1)
            try:
                regs['a'] = ('const', int(parts[1]))
            except ValueError:
                regs['a'] = fresh_unknown()
            continue

        # pop $X: SP shifts, target register gets unknown value
        if mn == 'pop' and len(parts) == 2 and parts[1] in REG_NAMES:
            out.append(line)
            r = REG_NAMES[parts[1]]
            shift_all(-1)
            regs[r] = None
            continue

        # stsp: clobbers A, but D = original A (microcode saves A→D first).
        # Also mark D as holding the value at stack[SP+off]: after `stsp off`,
        # stack[SP+off] = A and D = A (= stack[SP+off]). That lets the
        # follow-up `mov $d,$a; ldsp off` elide the ldsp, since A would match.
        if mn == 'stsp':
            out.append(line)
            try:
                off = int(parts[1]) if len(parts) == 2 else None
            except ValueError:
                off = None
            regs['d'] = ('sp', off) if off is not None else regs['a']
            regs['a'] = fresh_unknown()
            continue

        # jal/jal_r/ocall: function call, all bets off
        if mn in ('jal', 'jal_r', 'ocall'):
            out.append(line)
            invalidate_all()
            continue

        # swap: exchange A and B
        if mn == 'swap':
            out.append(line)
            regs['a'], regs['b'] = regs['b'], regs['a']
            continue

        # inc/dec: A modified, value unknown
        if mn in ('inc', 'dec'):
            out.append(line)
            regs['a'] = fresh_unknown()
            continue

        # j (unconditional jump): don't modify regs
        if mn == 'j':
            out.append(line)
            continue

        # xor: clobbers A AND D (D used as scratch in microcode)
        if mn == 'xor':
            out.append(line)
            regs['a'] = fresh_unknown()
            regs['d'] = fresh_unknown()
            continue

        # istc_inc: clobbers A (set to old B) and B (incremented)
        if mn == 'istc_inc':
            out.append(line)
            regs['a'] = fresh_unknown()
            regs['b'] = fresh_unknown()
            continue

        # push_b: doesn't modify registers but changes SP
        if mn == 'push_b':
            out.append(line)
            shift_all(1)
            continue

        # pop_b: loads B from stack, changes SP
        if mn == 'pop_b':
            out.append(line)
            regs['b'] = fresh_unknown()
            shift_all(-1)
            continue

        # Everything else: conservatively assume A clobbered
        out.append(line)
        if mn in CLOBBERS_A:
            regs['a'] = fresh_unknown()

    lines = out

    # ── Pass 3: Pattern-based peephole ───────────────────────────────
    # IR-based implementation. Each rule is an IR pass; the outer loop
    # iterates until a fixed point (no further rewrites). Rules:
    #   - stsp/ldsp elision when the ldsp's A is dead
    #   - ldi $a,X + push $a → push_imm X
    #   - push $a + pop $a → eliminate (unless ;!keep)
    #   - push $a + pop $X → mov $a,$X
    #   - Branch threading through one-instruction jump labels
    import mk1ir as _ir_p3

    # 2-instr window rewriter closure — all Pass 3's 2-instr rules live here.
    def _pass3_window(a, b, first_raw):
        indent = first_raw[:len(first_raw) - len(first_raw.lstrip())]
        # ldi $a,X ; push $a → push_imm X
        if a.startswith('ldi $a,') and b == 'push $a':
            val = a.split(',', 1)[1]
            return [indent + f'push_imm {val}']
        # push $a ; pop $a → eliminate (honor ;!keep)
        if a == 'push $a' and b == 'pop $a':
            if ';!keep' not in first_raw:
                return []
        # push $a ; pop $X (X != $a) → mov $a,$X
        if a == 'push $a' and b.startswith('pop ') and b != 'pop $a':
            dst = b.split(None, 1)[1]
            return [indent + f'mov $a,{dst}']
        return None

    changed = True
    while changed:
        changed = False
        _prog = _ir_p3.parse_program(lines)
        n1 = _ir_p3.stsp_ldsp_elide(_prog, CLOBBERS_A)
        n2 = _ir_p3.pass_2_window(_prog, _pass3_window)
        n3 = _ir_p3.branch_thread(_prog)
        lines = _ir_p3.serialize_program(_prog)
        if n1 + n2 + n3 > 0:
            changed = True

    # ── Pass: collapse `mov $X,$a ; inc|dec ; mov $a,$X` → incX/decX ──
    # New microcode opcodes decb/decc/decd/incb/incc/incd do this in 1 byte.
    # They clobber A with the new register value (same effect as the 3-byte
    # sequence they replace — the compiler already assumes A is discardable
    # at this point in the sequence, since the pattern intentionally routes
    # through A to increment/decrement the register).
    #
    # Flags: the collapsed opcode sets Z/C identically to the expanded form
    # (both use the CINV-based inc/dec microcode on A). Safe for `jnz`/`jz`/
    # `jc`/`jnc` tests that follow.
    COLLAPSE = {
        ('mov $d,$a', 'dec', 'mov $a,$d'): 'decd',
        ('mov $c,$a', 'dec', 'mov $a,$c'): 'decc',
        ('mov $b,$a', 'dec', 'mov $a,$b'): 'decb',
        ('mov $d,$a', 'inc', 'mov $a,$d'): 'incd',
        ('mov $c,$a', 'inc', 'mov $a,$c'): 'incc',
        ('mov $b,$a', 'inc', 'mov $a,$b'): 'incb',
    }
    # Superopt-discovered: `mov $b,$a; sll; mov $a,$b` (3B) → `sllb` (1B),
    # saves 2B per occurrence. sllb is opcode 0x22 (retires the unused
    # `move $sp, $c`). Verified safe on flashed hardware and present in
    # the canonical microcode.bin, so it is enabled unconditionally.
    COLLAPSE[('mov $b,$a', 'sll', 'mov $a,$b')] = 'sllb'
    # IR-based implementation — replaces the old flat-list window walk.
    # Behavior and byte output are identical (validated against every
    # corpus program); the IR version is the migration foundation that
    # subsequent passes will build on.
    import mk1ir as _ir
    _prog = _ir.parse_program(lines)
    collapsed_count = _ir.collapse_3_window(_prog, COLLAPSE)
    lines = _ir.serialize_program(_prog)
    if collapsed_count > 0:
        import sys
        print(f"  Peephole: collapsed {collapsed_count} reg-inc/dec patterns "
              f"(saved {collapsed_count * 2}B)", file=sys.stderr)

    # ── DDRB fusion DISABLED — ddrb2_imm/ddrb3_imm are hardware-unsafe ──
    # Fusing consecutive ddrb_imm writes into the multi-immediate forms
    # produces bus/timing glitches on real silicon that plain ddrb_imm
    # does not exhibit. Root cause appears to be insufficient settling
    # between the back-to-back RO|U1|E0 cycles. Until the microcode or
    # hardware is updated to guarantee settling, the compiler never
    # emits ddrb2_imm or ddrb3_imm. The sllb 1-byte shift remains
    # safe and is still gated behind MK1_NEW_OPCODES above.
    #
    # The size-counters for ddrb2_imm/ddrb3_imm elsewhere are kept so
    # any stray hand-written occurrence is still accounted for.

    # ── Superopt auto-discovered peephole rules ──────────────────────
    # Verified equivalences found by superopt.py against mk1sim, each
    # under 64 boundary + 136 random initial register states. Adding
    # them here is safe because equivalence includes SP/ZF/CF.
    #
    # Rule 1: addi 1, $a ; mov $a, $X → mov $a, $X ; incX
    #   Both leave A = (old A) + 1, $X = (old A) + 1. incX's carry/zero
    #   flags match the addi's (both from same CINV-less inc microcode).
    # Rule 2: andi N, $a ; tst 0xFF → andi N, $a
    #   andi already sets ZF from the result; tst 0xFF re-tests the same
    #   value against all-bits-set, reproducing the same ZF. Redundant.
    # Rule 3: ori N, $a ; tst 0xFF → ori N, $a
    #   Symmetric to rule 2.
    # Rule 4: ldi $X, N ; ldi $a, N → ldi $a, N ; mov $a, $X
    #   Old: $X = N (via ldi 2B), then $a = N (ldi 2B) = 4B total.
    #   New: $a = N (2B), then $X = $a (mov 1B) = 3B. Saves 1B for
    #   any constant N that both registers need.
    import re as _re

    # Auto-rule rewriter — shared closure over the rule set.
    # Returns None if no match (keep both lines), else a list of
    # replacement raw lines.
    def _t41_rewrite(a, b, first_raw):
        # Preserve indentation from the first line being replaced.
        indent = first_raw[:len(first_raw) - len(first_raw.lstrip())]
        # Rule 2/3: andi/ori N,$a ; tst 0xFF → drop the tst
        m = _re.match(r'^(andi|ori) (\S+),\$a$', a)
        if m and b == 'tst 0xFF':
            return [indent + a]
        # Rule 4: ldi $X,N ; ldi $a,N → ldi $a,N ; mov $a,$X
        m = _re.match(r'^ldi \$([bcd]),(\S+)$', a)
        m2 = _re.match(r'^ldi \$a,(\S+)$', b)
        if m and m2 and m.group(2) == m2.group(1):
            reg_letter = m.group(1)
            imm = m2.group(1)
            return [indent + f'ldi $a,{imm}', indent + f'mov $a,${reg_letter}']
        # Rule 1: addi 1,$a ; mov $a,$X → mov $a,$X ; incX
        for reg_letter in ('b', 'c', 'd'):
            if a == f'addi 1,${reg_letter}' and b == f'mov $a,${reg_letter}':
                return [indent + f'mov $a,${reg_letter}', indent + f'inc{reg_letter}']
        return None

    _prog = _ir.parse_program(lines)
    auto_count = _ir.pass_2_window(_prog, _t41_rewrite)
    lines = _ir.serialize_program(_prog)
    if auto_count > 0:
        import sys
        print(f"  Peephole: auto-rules applied {auto_count}× "
              f"(saved ≥{auto_count}B)", file=sys.stderr)

    # Save/restore through stack around pure DDRB bus toggles:
    #   mov $d,$a; push $a; ddrb_imm...; ldsp 1; pop $d
    # → push $d; ddrb_imm...; pop $a
    # This appears in I2C read wrappers preserving the read byte across NACK/STOP.
    _stack_save_saved = 0
    new_lines = []
    i = 0
    while i < len(lines):
        if (i + 4 < len(lines)
                and lines[i].strip() == 'mov $d,$a'
                and lines[i + 1].strip() == 'push $a'):
            j = i + 2
            while j < len(lines) and lines[j].strip().startswith('ddrb_imm '):
                j += 1
            if (j > i + 2 and j + 1 < len(lines)
                    and lines[j].strip() == 'ldsp 1'
                    and lines[j + 1].strip() == 'pop $d'):
                indent = lines[i][:len(lines[i]) - len(lines[i].lstrip())]
                new_lines.append(indent + 'push $d')
                new_lines.extend(lines[i + 2:j])
                new_lines.append(indent + 'pop $a')
                _stack_save_saved += 2
                i = j + 2
                continue
        new_lines.append(lines[i])
        i += 1
    lines = new_lines
    if _stack_save_saved:
        import sys
        print(f"  Peephole: shortened DDRB-only stack save/restore "
              f"(saved {_stack_save_saved}B)", file=sys.stderr)

    # Loop-local inc/dec stores often preserve A just before jumping back to a
    # loop header that reloads the same stack slot:
    #   ldsp N; push $a; inc; stsp N+1; mov $d,$a; pop $a; j .loop
    #   .loop: ldsp N
    # The saved A is dead on that edge, and stsp already leaves D equal to the
    # updated value, so store directly to the unshifted stack slot.
    def _next_instr_after_label(label):
        try:
            pos = next(i for i, line in enumerate(lines)
                       if line.strip() == f'{label}:')
        except StopIteration:
            return None
        j = pos + 1
        while j < len(lines):
            s = lines[j].strip()
            if not s or s.startswith(';') or s.startswith('section ') or s.startswith('org '):
                j += 1
                continue
            return s
        return None

    _loop_store_saved = 0
    new_lines = []
    i = 0
    while i < len(lines):
        if i + 6 < len(lines):
            s0 = lines[i].strip()
            s1 = lines[i + 1].strip()
            s2 = lines[i + 2].strip()
            s3 = lines[i + 3].strip()
            s4 = lines[i + 4].strip()
            s5 = lines[i + 5].strip()
            s6 = lines[i + 6].strip()
            m0 = _re.match(r'^ldsp (\d+)$', s0)
            m3 = _re.match(r'^stsp (\d+)$', s3)
            m6 = _re.match(r'^j (\.\w+)$', s6)
            if (m0 and m3 and m6
                    and s1 == 'push $a'
                    and s2 in ('inc', 'dec')
                    and s4 == 'mov $d,$a'
                    and s5 == 'pop $a'
                    and int(m3.group(1)) == int(m0.group(1)) + 1
                    and _next_instr_after_label(m6.group(1)) == s0):
                indent = lines[i][:len(lines[i]) - len(lines[i].lstrip())]
                new_lines.append(lines[i])
                new_lines.append(indent + s2)
                new_lines.append(indent + f'stsp {m0.group(1)}')
                new_lines.append(lines[i + 6])
                _loop_store_saved += 3
                i += 7
                continue
        new_lines.append(lines[i])
        i += 1
    lines = new_lines
    if _loop_store_saved:
        import sys
        print(f"  Peephole: shortened loop-local inc/dec stores "
              f"(saved {_loop_store_saved}B)", file=sys.stderr)

    return lines


def _validate_section_jumps(lines):
    """Catch jal/j from runtime sections to stage-1-only labels.

    MK1 layout at boot:
      - `section code` (stage 1) runs from address 0 at power-on. Its bytes
        get OVERWRITTEN when self-copy blits the kernel from page3 to code[0..].
      - `section page3_code` holds the kernel image in page 3, copied to code
        page 0 by self-copy. This is what runs at runtime.
      - `section data_code` holds overlay bodies in the data page, copied to
        the overlay region at runtime via _overlay_load.

    If a runtime caller (in page3_code or data_code) does `jal FOO` where FOO
    is defined only in section `code`, the call resolves to FOO's address —
    but code[FOO_addr] has been overwritten by self-copy. The CPU executes
    garbage.

    This validator scans the emitted assembly, builds a label → section map,
    and flags any runtime→stage-1 jal that would hit that trap.
    """
    import sys
    # Runtime sections (code that runs AFTER self-copy):
    #   page3_code  = overlay-mode kernel (separate from stage-1 `code`)
    #   data_code   = overlay bodies stored in page 1, copied at runtime
    #   stack_code  = overlay bodies stored in page 2, copied at runtime
    # Sections EXCLUDED from validation:
    #   page3_kernel = init-extraction kernel, which INTENTIONALLY calls
    #     helpers placed in stage-1 `code` at addresses past kernel_size
    #     (so they survive self-copy). Validator can't distinguish those
    #     from stage-1-only helpers without address tracking.
    RUNTIME_SECTIONS = {'page3_code', 'data_code', 'stack_code'}
    STAGE1_ONLY = 'code'

    # IR-based validation: parse once, use the labels index directly.
    import mk1ir as _ir_v
    prog = _ir_v.parse_program(lines)
    errors = _ir_v.validate_section_jumps(prog, RUNTIME_SECTIONS, STAGE1_ONLY)

    if errors:
        print("  !! SECTION-MISMATCH ERROR: runtime code references stage-1-only labels:",
              file=sys.stderr)
        for e in errors:
            print(f"     - {e}", file=sys.stderr)
        print("     The compiler extracted a helper to stage 1 but its caller ",
              file=sys.stderr)
        print("     remained in runtime main. Check init_extraction logic or ",
              file=sys.stderr)
        print("     force the helper resident. Aborting to prevent broken output.",
              file=sys.stderr)
        sys.exit(1)


def _emit_why_not_smaller(gen, args, metrics, code_bytes, MAX_CODE):
    """Detailed per-component byte breakdown. First tool to reach for when
    a program overflows. No-op when the --why-not-smaller flag isn't set."""
    if not getattr(args, 'why_not_smaller', False):
        return
    import sys
    _why = getattr(gen, '_why_breakdown', None)
    print("\n── Byte Sink Report ──", file=sys.stderr)
    if _why is None:
        print("  (flat mode — no overlay partition to break down)", file=sys.stderr)
        print(f"  code_bytes: {code_bytes}B / {MAX_CODE}B", file=sys.stderr)
        print("───────────────────────", file=sys.stderr)
        return
    _why['stage1_init'] = code_bytes
    print(f"  Mode: {_why['mode']}", file=sys.stderr)
    print(f"  Stage-1 init: {_why['stage1_init']}B / {MAX_CODE}B",
          file=sys.stderr)
    print(f"  Kernel total: {_why['kernel_total']}B "
          f"(loader={_why['loader']}B + main={_why['main']}B "
          f"+ helpers={_why['helpers_total']}B"
          + (f" + shared={_why['shared_helpers_total']}B"
             if _why['shared_helpers_total'] else "")
          + ")", file=sys.stderr)
    _named = _why['resident_helpers']
    if _named:
        print("  Resident helpers (largest first):", file=sys.stderr)
        for _n, _sz in sorted(_named.items(), key=lambda kv: -kv[1])[:10]:
            _pct = 100.0 * _sz / max(1, _why['kernel_total'])
            print(f"    {_n:<28} {_sz:>4}B  ({_pct:>4.1f}% of kernel)",
                  file=sys.stderr)
    if _why['overlay_slots']:
        print("  Overlay slots (largest first):", file=sys.stderr)
        for _slot in sorted(_why['overlay_slots'], key=lambda s: -s['size'])[:10]:
            print(f"    [{_slot['idx']}] {_slot['name']:<24} {_slot['size']:>4}B",
                  file=sys.stderr)
    _tight = 'stage1' if _why['stage1_init'] > MAX_CODE - 10 else 'kernel'
    print(f"  Top 5 sinks (tight budget: {_tight}):", file=sys.stderr)
    _sinks = [('loader', _why['loader']),
              ('main', _why['main'])] + list(_named.items())
    for _n, _sz in sorted(_sinks, key=lambda kv: -kv[1])[:5]:
        print(f"    {_n:<28} {_sz:>4}B", file=sys.stderr)
    print("───────────────────────", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description='MK1 C Compiler v2')
    ap.add_argument('input', help='C source file')
    ap.add_argument('-o', '--output', help='Output .asm file')
    ap.add_argument('-O', '--optimize', action='store_true', help='Enable optimization')
    ap.add_argument('--eeprom', action='store_true', help='EEPROM overlay mode: partition code for EEPROM-backed execution')
    ap.add_argument('--eeprom-base', type=lambda x: int(x, 0), default=0x0200,
                    help='EEPROM base address for overlay storage (default: 0x0200)')
    ap.add_argument('--metrics-out', help='Write size metrics as JSON to this path (for size-regression harness)')
    ap.add_argument('--why-not-smaller', action='store_true',
                    help='Emit a per-section byte breakdown and top-5 byte sinks to stderr. '
                         'Useful when a program overflows or is near-limit; shows exactly '
                         'which resident helpers and overlay bodies are eating budget.')
    ap.add_argument('--timeout', type=int, default=30,
                    help='Wall-clock timeout in seconds (default: 30). MK1 is a small ISA '
                         'targeting 256-byte pages; a healthy compile is <2s. A run >30s '
                         'indicates a pathological optimizer path or infinite loop. Set to '
                         '0 to disable.')
    args = ap.parse_args()

    if args.timeout > 0:
        import signal as _signal, os as _os
        _to = args.timeout
        _input = args.input
        def _timeout_handler(signum, frame):
            # Write to fd 2 directly — sys.stderr may be redirected by
            # _prepare_compile's contextlib.redirect_stderr at the moment
            # the alarm fires, which would swallow this message.
            _os.write(2, (f"mk1cc2: aborting — compile exceeded {_to}s wall-clock "
                          f"timeout on {_input}. Likely a pathological optimizer "
                          f"path or infinite loop. Use --timeout 0 to disable; "
                          f"--timeout N to raise.\n").encode())
            _os._exit(2)
        _signal.signal(_signal.SIGALRM, _timeout_handler)
        _signal.alarm(_to)

    with open(args.input) as f: source = f.read()

    def _prepare_compile(prefer_delay_cal_init=False, capture_log=False):
        gen = MK1CodeGen(optimize=args.optimize)
        # Stash the in-flight gen so the exception handler can recover
        # `_matcher_fires` even on compile failure (populated during
        # _simplify_function_bodies, before _overlay_partition can throw).
        _prepare_compile._last_gen = gen
        gen._prefer_delay_cal_init = prefer_delay_cal_init
        if hasattr(args, 'eeprom') and args.eeprom:
            gen.eeprom_mode = True
            gen.eeprom_base = args.eeprom_base
        if capture_log:
            import contextlib as _ctx
            import io as _io
            _buf = _io.StringIO()
            with _ctx.redirect_stderr(_buf):
                asm = gen.compile(source)
            compile_log = _buf.getvalue()
        else:
            asm = gen.compile(source)
            compile_log = ''

        # ── Section-mismatch validator ──
        # Catch the class of bug where runtime code (page3_code or data_code,
        # i.e. anything that runs AFTER self-copy) calls a label defined only
        # in section `code` (stage-1-only helpers whose bytes get overwritten
        # by self-copy's kernel blit). These bugs produce "jal to nonsense"
        # and are nearly impossible to debug from hardware symptoms alone.
        _validate_section_jumps(asm.split('\n'))

        # Run peephole optimizer — but NOT on kernel sections for overlay
        # programs where manifest offsets were computed from pre-peepholed
        # component sizes. A final peephole of the KERNEL would shrink it and
        # make offsets wrong; peepholing STAGE-1 is fine because it's a
        # separate section and doesn't affect kernel addresses.
        lines = asm.split('\n')
        if not getattr(gen, '_overlay_kernel_size', None):
            # Flat or init-extraction mode: peephole everything
            lines = peephole(lines)
            asm = '\n'.join(lines)
        else:
            # Overlay mode: peephole stage-1 (`code` section) only, leave
            # `page3_code`, `page3_kernel`, `data_code`, `stack_code`,
            # `page3`, `data`, `eeprom` unchanged.
            SAFE_SECTIONS = {'code'}  # stage-1 code; no manifest-offset concern
            groups = []   # list of (section_name, [lines])
            cur_sec = 'code'
            cur_lines = []
            for line in lines:
                s = line.strip()
                if s.startswith('section '):
                    if cur_lines:
                        groups.append((cur_sec, cur_lines))
                    cur_sec = s.split()[1]
                    cur_lines = [line]
                else:
                    cur_lines.append(line)
            if cur_lines:
                groups.append((cur_sec, cur_lines))
            out_lines = []
            for sec, grp in groups:
                if sec in SAFE_SECTIONS:
                    # Split `section` directive from body so peephole doesn't
                    # interpret it oddly — peephole already handles these lines
                    # but keep the directive in place.
                    out_lines.extend(peephole(grp))
                else:
                    out_lines.extend(grp)
            lines = out_lines
            asm = '\n'.join(lines)

        two_byte_set_local = {'ldsp','stsp','push_imm','jal','jc','jz','jnc','jnz','j','ldi',
                              'cmp','addi','subi','andi','ori','ld','st','ldsp_b','ldp3','stp3',
                              'ocall','tst','out_imm','cmpi','ddrb_imm','ddra_imm',
                              'ora_imm','orb_imm'}
        code_bytes_local = 0
        section = 'code'
        for line in lines:
            s = line.strip()
            if 'section page3_kernel' in s or 'section page3_code' in s:
                section = 'page3'; continue
            elif 'section page3' in s and 'kernel' not in s and 'code' not in s:
                section = 'page3_data'; continue
            elif 'section p3patch' in s: section = 'p3patch'; continue
            elif 'section eeprom' in s: section = 'eeprom'; continue
            elif 'section data_code' in s: section = 'data'; continue
            elif 'section stack_code' in s: section = 'stack'; continue
            elif 'section data' in s: section = 'data'; continue
            elif 'section code' in s: section = 'code'; continue
            if section != 'code':
                continue
            if not s or s.endswith(':') or s.startswith(';'): continue
            mn = s.split()[0]
            if mn == 'byte':
                b = 1
            elif mn == 'cmp':
                parts = s.split()
                b = 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
            elif mn == 'ddrb2_imm':
                b = 3
            elif mn == 'ddrb3_imm':
                b = 4
            elif mn in two_byte_set_local:
                b = 2
            else:
                b = 1
            code_bytes_local += b

        init_extraction_local = getattr(gen, '_init_extraction_done', False)
        overlay_mode_local = hasattr(gen, '_overlay_kernel_size')
        init_limit_local = 254 if (init_extraction_local or overlay_mode_local) else 250
        return gen, asm, lines, code_bytes_local, init_limit_local, compile_log

    def _emit_partial_metrics(g, exc_msg):
        """Emit metrics with `compile_ok=False` + matcher_fires when compile
        fails. Keeps the corpus audit / regression baseline complete: programs
        that wrap-fail at partition still record which matchers fired so
        future regressions stay detectable."""
        if not args.metrics_out:
            return
        import json as _json
        partial = {'compile_ok': False, 'compile_error': exc_msg}
        _fires = getattr(g, '_matcher_fires', {}) if g else {}
        if _fires:
            partial['matcher_fires'] = {fn: sorted(s) for fn, s in sorted(_fires.items())}
        try:
            with open(args.metrics_out, 'w') as _mf:
                _json.dump(partial, _mf, indent=2)
        except Exception:
            pass

    try:
        gen, asm, lines, _code_bytes_try, _init_limit_try, _compile_log = _prepare_compile(
            prefer_delay_cal_init=True, capture_log=True)
    except Exception as _e:
        _emit_partial_metrics(getattr(_prepare_compile, '_last_gen', None), str(_e))
        raise
    if (_code_bytes_try > _init_limit_try and
            getattr(gen, '_prefer_delay_cal_init', False)):
        import sys as _retry_sys
        print(f"  Placement retry: init-only __delay_cal overflowed stage 1 "
              f"({_code_bytes_try}B > {_init_limit_try}B); using runtime/overlay placement",
              file=_retry_sys.stderr)
        gen, asm, lines, _code_bytes_try, _init_limit_try, _compile_log = _prepare_compile(
            prefer_delay_cal_init=False)
    elif _compile_log:
        import sys as _compile_log_sys
        print(_compile_log, end='', file=_compile_log_sys.stderr)

    if args.output:
        with open(args.output, 'w') as f: f.write(asm + '\n')
    else:
        print(asm)

    # Page & EEPROM utilisation report
    import sys
    MAX_CODE = 250  # safe code page limit
    init_extraction = getattr(gen, '_init_extraction_done', False)
    overlay_mode = hasattr(gen, '_overlay_kernel_size')  # full overlay path

    # Count assembled bytes per section from the output
    code_bytes = 0
    data_bytes = 0
    p3_bytes = 0
    stack_bytes = 0   # page 2 (stack page) — Phase 5 manifest + stack_code overlays
    eeprom_bytes = 0
    section = 'code'
    two_byte_set = {'ldsp','stsp','push_imm','jal','jc','jz','jnc','jnz','j','ldi',
                    'cmp','addi','subi','andi','ori','ld','st','ldsp_b','ldp3','stp3',
                    'ocall','tst','out_imm','cmpi','ddrb_imm','ddra_imm',
                    'ora_imm','orb_imm'}
    for line in lines:
        s = line.strip()
        if 'section page3_kernel' in s or 'section page3_code' in s:
            section = 'page3'; continue
        elif 'section page3' in s and 'kernel' not in s and 'code' not in s:
            section = 'page3_data'; continue
        elif 'section p3patch' in s: section = 'p3patch'; continue
        elif 'section eeprom' in s: section = 'eeprom'; continue
        elif 'section data_code' in s: section = 'data'; continue
        elif 'section stack_code' in s: section = 'stack'; continue
        elif 'section stack' in s: section = 'stack'; continue
        elif 'section data' in s: section = 'data'; continue
        elif 'section code' in s: section = 'code'; continue
        if not s or s.endswith(':') or s.startswith(';'): continue
        mn = s.split()[0]
        if mn == 'byte':
            b = 1
        elif mn == 'cmp':
            parts = s.split()
            b = 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
        elif mn == 'ddrb2_imm':
            b = 3
        elif mn == 'ddrb3_imm':
            b = 4
        elif mn in two_byte_set:
            b = 2
        else:
            b = 1
        if section == 'code': code_bytes += b
        elif section == 'data': data_bytes += b
        elif section == 'stack': stack_bytes += b
        elif section in ('page3', 'page3_data'): p3_bytes += b
        elif section == 'eeprom': eeprom_bytes += b

    # Stack depth estimate from locals
    stack_depth = 0
    if hasattr(gen, '_max_stack_depth'):
        stack_depth = gen._max_stack_depth
    else:
        # Rough estimate from locals count
        for fn_name in dir(gen):
            pass  # can't easily get this without more tracking

    # Compute kernel size for init extraction
    kernel_bytes = 0
    if init_extraction:
        sec = 'skip'
        for line in lines:
            s = line.strip()
            if 'section page3_kernel' in s: sec = 'kernel'; continue
            if sec == 'kernel' and ('section ' in s): sec = 'done'; continue
            if sec != 'kernel': continue
            if not s or s.endswith(':') or s.startswith(';'): continue
            mn = s.split()[0]
            if mn == 'byte': kernel_bytes += 1
            elif mn == 'cmp':
                parts = s.split()
                kernel_bytes += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
            elif mn in two_byte_set: kernel_bytes += 2
            else: kernel_bytes += 1

    copy_loop = 14
    eeprom_header = 16
    eeprom_payload = max(0, eeprom_bytes - eeprom_header)

    # Collect structured metrics for the size-regression harness.
    # Populated alongside the human-readable report; written to --metrics-out
    # at the end. Fields are null when not applicable to the current mode.
    metrics = {
        'mode': None,
        'compile_ok': True,
        'code_overflow': False,
        'stage1_init': None,
        'stage1_copy': None,
        'stage1_total': None,
        'kernel': None,
        'page0_used': None,
        'page0_free': None,
        'page1_allocated': gen.data_alloc,
        'page1_free': 256 - gen.data_alloc,
        'page3_used': None,
        'page3_free': None,
        'eeprom_used': 0,
        'eeprom_free': 4096,
        'overlay_slots': None,
        'overlay_largest': None,
        'overlay_shared': None,
        'overlay_region': None,
    }

    print("\n── MK1 Memory Report ──", file=sys.stderr)
    if overlay_mode:
        ov_kernel = gen._overlay_kernel_size
        ov_region = gen._overlay_region
        ov_shared = gen._overlay_shared_size
        init_only = code_bytes - copy_loop
        metrics.update({
            'mode': 'overlay',
            'stage1_init': init_only,
            'stage1_copy': copy_loop,
            'stage1_total': code_bytes,
            'kernel': ov_kernel,
            'page0_used': ov_region,
            'page0_free': MAX_CODE - ov_region,
            'overlay_shared': ov_shared,
            'overlay_region': ov_region,
            'overlay_slots': getattr(gen, '_overlay_count', None),
            'overlay_largest': getattr(gen, '_overlay_largest', None),
            'page3_used': p3_bytes,
            'page3_free': 256 - p3_bytes,
        })
        print(f"  Mode:       two-stage boot (overlay system)", file=sys.stderr)
        print(f"  Stage 1:    {init_only}B init + {copy_loop}B copy = {code_bytes}B", file=sys.stderr)
        print(f"  Stage 2:    {ov_kernel}B kernel + {ov_shared}B shared = {ov_region}B", file=sys.stderr)
        runtime_free = MAX_CODE - ov_region
        print(f"  Page 0:     {ov_region}B used, {runtime_free}B overlay region", file=sys.stderr)
    elif init_extraction:
        init_only = code_bytes - copy_loop
        total = max(init_only, kernel_bytes) + copy_loop
        metrics.update({
            'mode': 'init_extraction',
            'stage1_init': init_only,
            'stage1_copy': copy_loop,
            'stage1_total': code_bytes,
            'kernel': kernel_bytes,
            'page0_used': kernel_bytes,
            'page0_free': MAX_CODE - kernel_bytes,
            'page3_used': kernel_bytes,
            'page3_free': 256 - kernel_bytes,
        })
        print(f"  Mode:       two-stage boot (init extraction)", file=sys.stderr)
        print(f"  Stage 1:    {init_only}B init + {copy_loop}B copy = {code_bytes}B / {MAX_CODE}B", file=sys.stderr)
        print(f"  Stage 2:    {kernel_bytes}B kernel  (runtime code page)", file=sys.stderr)
        runtime_free = MAX_CODE - kernel_bytes
        print(f"  Page 0:     {kernel_bytes}B used, {runtime_free}B free at runtime", file=sys.stderr)
    else:
        metrics.update({
            'mode': 'flat',
            'kernel': code_bytes,
            'page0_used': code_bytes,
            'page0_free': MAX_CODE - code_bytes,
            'page3_used': p3_bytes,
            'page3_free': 256 - p3_bytes,
        })
        print(f"  Mode:       single-stage (flat)", file=sys.stderr)
        print(f"  Page 0:     {code_bytes}B / {MAX_CODE}B code", file=sys.stderr)

    data_runtime = gen.data_alloc
    n_globals = len([n for n in gen.globals if not n.startswith('_')])
    data_temp = data_bytes - gen.data_alloc if data_bytes > gen.data_alloc else 0
    data_desc = f"{n_globals} globals" if n_globals else "allocated"
    print(f"  Page 1:     {data_runtime}B {data_desc}, {256 - data_runtime}B free  (data page)", file=sys.stderr)

    # Page 2 layout per the kernel-state allocator + page-2 partition
    # refactor: bottom = page-2 overlay storage (data_size bytes for any
    # `section stack_code` overlays placed there; 0 if all overlays
    # fit in pages 1/3); kstate region 0x80..0xAF holds the kernel-
    # state slots from KERNEL_STATE plus the overlay manifest+pages
    # array; guard band 0xB0..0xBF; reserved stack 0xC0..0xFF (SP grows
    # down from 0xFF; static analysis enforces depth ≤ 64 B). The
    # `stack_size` field reported by the assembler covers everything
    # written, so for a typical program it is `manifest_end` (= about
    # 0x90 + meta_size bytes).
    if overlay_mode:
        # Distinguish whether anything actually lives at the page-2
        # overlay slot vs only the manifest/kstate.
        p2_overlay_present = stack_bytes > 0xB0  # anything past guard band would be unusual
        p2_desc = "manifest + kstate" if stack_bytes else "stack only"
        if p2_overlay_present and stack_bytes >= P2_OVERLAY_BYTES:
            p2_desc = "overlays + manifest + kstate"
        print(f"  Page 2:     {stack_bytes}B used ({p2_desc}); stack reserved at 0xC0..0xFF",
              file=sys.stderr)
    else:
        print(f"  Page 2:     stack (grows down from 0xFF)", file=sys.stderr)

    if overlay_mode:
        # Phase 5 moved manifest+pages off page 3 into the page-2
        # kstate region, so post-init page 3 is just the freed kernel
        # image area + shared helpers + page-3 overlay storage.
        print(f"  Page 3:     {p3_bytes}B / 256B  "
              f"(kernel image transient; shared helpers + page-3 overlays persist)",
              file=sys.stderr)
    elif init_extraction:
        p3_free = 256 - kernel_bytes
        print(f"  Page 3:     {kernel_bytes}B kernel image, {p3_free}B free", file=sys.stderr)
        print(f"              (fully reusable at runtime via derefp3/iderefp3)", file=sys.stderr)
    elif p3_bytes > 0:
        print(f"  Page 3:     {p3_bytes}B / 256B", file=sys.stderr)
    else:
        print(f"  Page 3:     unused (256B available)", file=sys.stderr)

    if eeprom_bytes > 0:
        metrics['eeprom_used'] = eeprom_payload
        metrics['eeprom_free'] = 4096 - eeprom_payload
        print(f"  EEPROM:     {eeprom_payload}B used, {4096 - eeprom_payload}B free  (AT24C32)", file=sys.stderr)
    else:
        print(f"  EEPROM:     unused  (4096B available)", file=sys.stderr)

    if init_extraction:
        total_free = (MAX_CODE - kernel_bytes) + (256 - data_runtime) + 256 + (4096 - eeprom_payload)
        print(f"  Total free: {total_free}B  (code {MAX_CODE-kernel_bytes} + data {256-data_runtime} + page3 256 + eeprom {4096-eeprom_payload})", file=sys.stderr)

    # Warnings — init stage can use up to 254B (runs once, self-copies kernel)
    init_limit = 254 if (init_extraction or overlay_mode) else MAX_CODE
    if code_bytes > init_limit:
        metrics['code_overflow'] = True
        metrics['compile_ok'] = False
        print(f"  !! CODE OVERFLOW: {code_bytes}B > {init_limit}B limit "
              f"(over by {code_bytes - init_limit}B)", file=sys.stderr)
        # Suggest concrete remediation
        if overlay_mode:
            ov_region_needed = getattr(gen, '_overlay_region', 0) + 14
            if ov_region_needed > 250:
                print(f"     → kernel+copy needs {ov_region_needed}B but page is 250B."
                      f" Move more user functions into overlays or split large helpers.",
                      file=sys.stderr)
        elif init_extraction:
            print(f"     → stage 1 exceeds page limit. Causes: "
                  f"large init helpers (e.g. __lcd_init), many resident runtime helpers, "
                  f"or an oversized main.",
                  file=sys.stderr)
        else:
            print(f"     → program doesn't fit in flat mode. Add user functions to "
                  f"enable overlay partitioning, or reduce code size.",
                  file=sys.stderr)
        # Hard fail: uploading oversized code silently truncates at the ESP32
        # assembler's 256B code buffer, leaving the MK1 running garbage and
        # corrupting state for subsequent programs. Refuse to emit.
        print("───────────────────────", file=sys.stderr)
        # Emit the byte-sink report BEFORE exiting — overflow is exactly
        # when the user needs it most.
        _emit_why_not_smaller(gen, args, metrics, code_bytes, MAX_CODE)
        if args.metrics_out:
            import json as _json
            with open(args.metrics_out, 'w') as _mf:
                _json.dump(metrics, _mf, indent=2)
        sys.exit(1)
    elif code_bytes > init_limit - 3:
        print(f"  !! Code page tight: only {init_limit - code_bytes}B free", file=sys.stderr)
    if gen.data_alloc > 256:
        print(f"  !! DATA OVERFLOW: {gen.data_alloc}B > 256B", file=sys.stderr)

    print("───────────────────────", file=sys.stderr)

    _emit_why_not_smaller(gen, args, metrics, code_bytes, MAX_CODE)

    if args.metrics_out:
        import json as _json
        # Merge the size breakdown into metrics when available.
        _why = getattr(gen, '_why_breakdown', None)
        if _why is not None:
            metrics['breakdown'] = _why
        # Matcher-fire telemetry: which compiler-injected calls each user
        # function picked up. Auto-derived from pre/post call-name diff
        # in _simplify_function_bodies. Output as sorted lists for
        # deterministic JSON. Future matchers that synthesize new call
        # nodes show up here automatically.
        _fires = getattr(gen, '_matcher_fires', {})
        if _fires:
            metrics['matcher_fires'] = {
                fn: sorted(s) for fn, s in sorted(_fires.items())
            }
        with open(args.metrics_out, 'w') as _mf:
            _json.dump(metrics, _mf, indent=2)

if __name__ == '__main__':
    main()
