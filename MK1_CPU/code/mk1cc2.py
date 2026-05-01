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
    'oled_font_loaded': KSlot(
        page=2, offset=0xB3, size=1, owner='__oled_init',
        description='one-shot guard: non-zero means the SSD1327 font '
                    'has been copied from EEPROM to page 3 so a second '
                    'oled_init() call skips the multi-byte sequential '
                    'EEPROM read.'),
    'cold_current_id': KSlot(
        page=2, offset=0xB4, size=1, owner='__cold_call',
        description='id of the cold helper currently loaded into the '
                    'cold slot. 0xFF = no cold helper loaded (running '
                    'from main/kernel). Used by __cold_call to track '
                    'which helper to RELOAD after a cold→cold call '
                    "returns — a callee's bytes overwrote the caller's "
                    "in the slot, so the caller's bytes need to be "
                    'reloaded before its `ret` lands on slot bytes.'),
    'cold_call_arg': KSlot(
        page=2, offset=0xB5, size=1, owner='__cold_call',
        description="caller's $a saved by __cold_call's prologue and "
                    'restored by the cold helper body prologue. Lets '
                    'cold helpers receive their parameter in $a even '
                    "though the dispatcher's setup and the final "
                    '`jal_r {slot_base}` overwrite $a. Each cold helper '
                    '(user `ee64` or auto-cold) gets a 4-byte synthetic '
                    'prologue (`ldi $a,0xB5; deref2`) prepended at '
                    'compile time when its body reads $a as input.'),
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

P2_OVERLAY_BYTES     = P2_OVERLAY_HI - P2_OVERLAY_LO + 1   # 176
P2_KSTATE_BYTES      = P2_KSTATE_HI  - P2_KSTATE_LO  + 1   #  48
P2_GUARD_BYTES       = P2_GUARD_HI   - P2_GUARD_LO   + 1   #  16
P2_STACK_BUDGET      = P2_STACK_HI   - P2_STACK_LO   + 1   #  16

# Manifest + pages array base within the kstate region. Phase 5 moved
# them out of `section page3_code` (where they squatted at the tail of
# the kernel image, occupying 15-30 B of page-3 post-init) into the
# structured page-2 kstate area. KERNEL_STATE fixed slots fill from
# P2_KSTATE_LO upward; the manifest+pages variable region fills from
# P2_MANIFEST_BASE upward. The gap between fixed-slot top and manifest
# base is reserved for future fixed-slot growth.
P2_MANIFEST_BASE     = 0xC0

# ── Cold tier (AT24C512 @ 0x50) ──────────────────────────────────────
# Layout for runtime cold-call dispatch (Phase 4 of cold-tier rollout).
# Helpers tagged COLD live in the 64KB EEPROM and are loaded into a
# fixed code-page slot on demand. The dispatcher (`__cold_call`) reads
# (ee64_addr_hi, ee64_addr_lo, byte_count) from a 3-byte-per-entry
# table in page 3 starting at COLD_TABLE_BASE.
#
# COLD_TABLE_BASE = 0x10: skip the first 16 B of page 3 to leave room
#   for OLED font cache or other front-of-page-3 callers if they ever
#   coexist with cold-tier on the same program. 32 B (page3[0x10..0x2F])
#   gives 10 cold helpers — generous for now.
# COLD_SLOT_BASE = 0xF0: 16 B at the high end of the code page. Page
#   0 is capped at 250 B by the partitioner; the slot at 0xF0..0xFF
#   sits in the unused tail. Helpers that need a bigger slot can grow
#   this region downward when the time comes.
# Cold-tier slot lives at the high end of code page 0. The runtime
# dispatcher loads each cold helper into addresses
# [COLD_SLOT_BASE .. COLD_SLOT_BASE + max_cold_size - 1]. Slot size
# is right-sized at compile time to fit the largest cold helper —
# a program with only 30 B helpers reserves a 30 B slot rather than
# the worst-case 64 B. `COLD_SLOT_BYTES` is the upper cap; the
# partitioner picks `min(max_cold_size, COLD_SLOT_BYTES)` and
# positions the slot accordingly.
COLD_TABLE_BASE      = 0x10
COLD_SLOT_BYTES      = 128

# Mnemonics that assemble to >1 byte. Mirrors the per-mnemonic byte
# accounting in `measure_lines` (closure inside `_overlay_partition`)
# and the final size report. Promoted to module scope so cold-function
# compilation can size its output before the partitioner runs.
_TWO_BYTE_MNEMONICS = {
    'ldsp', 'stsp', 'push_imm', 'jal', 'jc', 'jz', 'jnc', 'jnz', 'j',
    'ldi', 'cmp', 'addi', 'subi', 'andi', 'ori', 'ld', 'st', 'ldsp_b',
    'ldp3', 'stp3', 'ocall', 'tst', 'out_imm', 'cmpi', 'ddrb_imm',
    'ddra_imm', 'ora_imm', 'orb_imm',
}

def _helper_clobbers_a_first(body_lines):
    """True if the helper's first real instruction writes `$a` as its
    first action — meaning the caller's `$a` is immediately discarded
    and the auto-cold prologue (`ldi $a,0xB5; deref2`) would just
    reload a value that's about to be overwritten.

    The mk1 mnemonic `move SRC, DST` always writes DST. Recognised
    forms in the assembler dialect:
      - `ldi $a, X`   (= `move imm, $a`) → writes $a
      - `clr $a`      (= `move 0, $a` typically)
      - `mov $X, $a`  → writes $a
      - `pop $a`      (= `load $a, [$sp]`) → writes $a

    Used by `_extract_force_cold_helpers` to decide whether to prepend
    the 3-byte arg-recovery prologue. Conservative: any unrecognised
    first instruction returns False (prologue gets prepended)."""
    for raw in body_lines:
        s = raw.strip()
        if not s or s.endswith(':') or s.startswith(';'):
            continue
        # First real instruction. Parse mnemonic + operands.
        toks = s.split(None, 1)
        mn = toks[0]
        rest = toks[1] if len(toks) > 1 else ''
        # `ldi $a` / `clr $a` / `pop $a` / explicit `mov X, $a`.
        if mn == 'ldi' and rest.startswith('$a'):
            return True
        if mn == 'clr' and rest.startswith('$a'):
            return True
        if mn == 'pop' and rest.startswith('$a'):
            return True
        if mn == 'mov':
            # `mov SRC, DST` — DST is the second arg.
            parts = [p.strip() for p in rest.split(',')]
            if len(parts) == 2 and parts[1] == '$a':
                return True
        # Unrecognised first instruction: assume $a may be live.
        return False
    return False


def _measure_asm_bytes(lines):
    """Estimate assembled byte size of asm-text lines.
    Mirrors the size logic the partitioner uses; matches the firmware
    assembler within rounding (`byte` directives count 1; `cmp $reg`
    is 1 B vs `cmp <imm>` is 2 B; `ddrb2_imm`/`ddrb3_imm` count their
    actual macro expansion length)."""
    size = 0
    for line in lines:
        s = line.strip()
        if not s or s.endswith(':') or s.startswith(';'):
            continue
        mn = s.split()[0]
        if mn == 'byte':
            size += 1
        elif mn == 'cmp':
            parts = s.split()
            size += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
        elif mn == 'ddrb2_imm':
            size += 3
        elif mn == 'ddrb3_imm':
            size += 4
        elif mn in _TWO_BYTE_MNEMONICS:
            size += 2
        else:
            size += 1
    return size

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
            'u8', 'u16', 'eeprom', 'ee64', 'sizeof', 'typedef', 'static'}

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
        self.cold_function_names = set()  # functions tagged with `ee64`

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
            # Check for storage qualifiers before type. `ee64` overrides
            # `eeprom` if both somehow appear; in practice they're
            # mutually exclusive.
            if self.match('ee64'):
                storage = 'ee64'
            elif self.match('eeprom'):
                storage = 'eeprom'
            else:
                storage = 'ram'
            typ = self.parse_type()
            name = self.expect('IDENT').value
            if self.match('('):
                fn = self.parse_function(typ, name)
                if fn:
                    if storage == 'ee64':
                        # Cold function — gets routed to AT24C512, called
                        # via __cold_call. Tracked by name so later
                        # codegen / call-rewrite knows which functions
                        # are cold.
                        self.cold_function_names.add(name)
                    functions.append(fn)
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
        # Empty statement (`;` alone). Permits idioms like `while (1);`
        # for spin-forever loops without forcing a useless block.
        if t.value == ';':
            self.advance()
            return ('block', [])
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
        # User functions auto-promoted to cold tier by the outer retry
        # in _prepare_compile when overlay placement would wrap into kernel.
        # Treated identically to source-level `ee64` tags from this point on.
        self._auto_cold_user_fns = set()
        # Compiler helpers (names starting with __) auto-promoted to cold
        # tier when overlay placement can't fit the kernel. Bodies are
        # extracted from self.code after _emit_i2c_helpers runs, then
        # routed through __cold_call. Phase 2 of skinny-kernel auto-paging.
        self._force_cold_helpers = set()

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

    def _port_shadow_mask_excluding(self, reg, device):
        """OR of all bits claimed on `reg` EXCEPT those by `device`.

        Used by emitters that drive their own claimed bits explicitly
        (e.g. the keypad scanner pulls one row LOW per row-slot — it
        owns bits 4-7 of ORA but its scan code can't OR those bits in
        from its own claim, otherwise it could never drive any of its
        rows LOW). Other devices' bits still need preserving."""
        total = 0
        for d, m in self._port_claims[reg].items():
            if d != device:
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

    def _i2c_ddrb_str(self, low_bits):
        """Build a `\\tddrb_imm 0xNN` string for an I2C bit-bang DDRB
        write. low_bits in {0,1,2,3} sets PB0/PB1; bits 2-7 follow the
        port shadow so a device that has claimed any DDRB[2-7] bit
        keeps it set across I2C operations. With no DDRB[2-7] claimers
        (the only state of the codebase as of 2026-04-26), this is
        byte-identical to the old raw `\\tddrb_imm 0x0X`.

        Use this for helper-emit functions that build asm as a list of
        strings (loader = []; loader.append(...)) where `emit_ddrb` is
        the wrong shape because they don't write directly to self.code."""
        shadow_high = self._port_shadow_mask('DDRB') & 0xFC
        return f'\tddrb_imm 0x{(low_bits & 0x03) | shadow_high:02X}'

    def emit_unrolled_i2c_send_const(self, val):
        """Emit the unrolled __i2c_sb body for a compile-time-constant
        byte: 24 ddrb_imm bus pulses (3 per bit, MSB-first) plus a
        4-pulse ACK clock with `exrw 0` to sample/discard ACK. ~3.4×
        faster than `jal __i2c_sb` (127 vs 437 cyc/byte) because every
        per-bit test/shift/branch is folded at emit time. Costs ~52B
        of inline code; only worth the trade in hot loops. Shared
        between the `i2c_send_const(K)` builtin and `__oled_fill`'s
        inner loop body — keep the two call sites in sync via this
        method."""
        val &= 0xFF
        for shift in range(7, -1, -1):
            if (val >> shift) & 1:
                self.emit_ddrb(0x02)
                self.emit_ddrb(0x00)
                self.emit_ddrb(0x02)
            else:
                self.emit_ddrb(0x03)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
        # ACK clock — same shape as __i2c_sb's tail
        self.emit_ddrb(0x02)             # SCL LOW, SDA released
        self.emit_ddrb(0x00)             # SCL HIGH (read ACK)
        self.emit('\texrw 0')            # sample ACK (discarded)
        self.emit_ddrb(0x02)             # SCL LOW

    # 5x5 font for SSD1327 oled_putc(). Each glyph is 5 row bytes; bits
    # 4..0 = leftmost..rightmost pixel. Add chars as needed; the
    # builtin errors out on unsupported chars rather than guessing.
    _OLED_FONT_5X5 = {
        ' ': [0x00, 0x00, 0x00, 0x00, 0x00],
        '.': [0x00, 0x00, 0x00, 0x00, 0x04],
        '-': [0x00, 0x00, 0x1F, 0x00, 0x00],
        '!': [0x04, 0x04, 0x04, 0x00, 0x04],
        '?': [0x0E, 0x11, 0x02, 0x00, 0x04],
        ':': [0x00, 0x04, 0x00, 0x04, 0x00],
        'A': [0x0E, 0x11, 0x1F, 0x11, 0x11],
        'B': [0x1E, 0x11, 0x1E, 0x11, 0x1E],
        'C': [0x0F, 0x10, 0x10, 0x10, 0x0F],
        'D': [0x1E, 0x11, 0x11, 0x11, 0x1E],
        'E': [0x1F, 0x10, 0x1E, 0x10, 0x1F],
        'F': [0x1F, 0x10, 0x1E, 0x10, 0x10],
        'G': [0x0F, 0x10, 0x13, 0x11, 0x0F],
        'H': [0x11, 0x11, 0x1F, 0x11, 0x11],
        'I': [0x1F, 0x04, 0x04, 0x04, 0x1F],
        'J': [0x1F, 0x04, 0x04, 0x14, 0x08],
        'K': [0x11, 0x12, 0x1C, 0x12, 0x11],
        'L': [0x10, 0x10, 0x10, 0x10, 0x1F],
        'M': [0x11, 0x1B, 0x15, 0x11, 0x11],
        'N': [0x11, 0x19, 0x15, 0x13, 0x11],
        'O': [0x0E, 0x11, 0x11, 0x11, 0x0E],
        'P': [0x1E, 0x11, 0x1E, 0x10, 0x10],
        'Q': [0x0E, 0x11, 0x15, 0x12, 0x0D],
        'R': [0x1E, 0x11, 0x1E, 0x12, 0x11],
        'S': [0x0F, 0x10, 0x0E, 0x01, 0x1E],
        'T': [0x1F, 0x04, 0x04, 0x04, 0x04],
        'U': [0x11, 0x11, 0x11, 0x11, 0x0E],
        'V': [0x11, 0x11, 0x11, 0x0A, 0x04],
        'W': [0x11, 0x11, 0x15, 0x15, 0x0A],
        'X': [0x11, 0x0A, 0x04, 0x0A, 0x11],
        'Y': [0x11, 0x0A, 0x04, 0x04, 0x04],
        'Z': [0x1F, 0x02, 0x04, 0x08, 0x1F],
        '0': [0x0E, 0x13, 0x15, 0x19, 0x0E],
        '1': [0x04, 0x0C, 0x04, 0x04, 0x0E],
        '2': [0x1E, 0x01, 0x0E, 0x10, 0x1F],
        '3': [0x1E, 0x01, 0x06, 0x01, 0x1E],
        '4': [0x12, 0x12, 0x1F, 0x02, 0x02],
        '5': [0x1F, 0x10, 0x1E, 0x01, 0x1E],
        '6': [0x0E, 0x10, 0x1E, 0x11, 0x0E],
        '7': [0x1F, 0x01, 0x02, 0x04, 0x04],
        '8': [0x0E, 0x11, 0x0E, 0x11, 0x0E],
        '9': [0x0E, 0x11, 0x0F, 0x01, 0x0E],
    }

    def _oled_render_5x5_to_4bpp(self, glyph):
        """Convert a 5-byte 5x5 glyph to the 21 4bpp bytes streamed to
        SSD1327 GDDRAM for one 6x7 cell. Foreground is bright (nibble
        0xF), background dark (0x0). 7 rows = 5 glyph + 1 descender +
        1 line gap; the right margin (col 5) is always dark.

        Layout per row: 3 bytes covering 6 pixels, high nibble = even
        col (left), low nibble = odd col (right)."""
        rows = list(glyph) + [0x00, 0x00]   # 2 blank rows after the glyph
        out = []
        for r in rows:
            b4 = (r >> 4) & 1
            b3 = (r >> 3) & 1
            b2 = (r >> 2) & 1
            b1 = (r >> 1) & 1
            b0 = r & 1
            out.append((0xF0 if b4 else 0) | (0x0F if b3 else 0))
            out.append((0xF0 if b2 else 0) | (0x0F if b1 else 0))
            out.append(0xF0 if b0 else 0)   # right margin always dark
        return out

    def _oled_ensure_font_p3(self):
        """Idempotent: allocate the compact 5x5 OLED font directly in
        page 3 (43 glyphs × 5 bytes = 215 B). The compiler emits the
        font data inline into page 3 at compile time; the bytes are
        uploaded with the program and stay resident in page 3 RAM
        with no runtime EEPROM read.

        This is simpler than the EEPROM + sequential-read-to-page-3
        approach: zero runtime cost (the font is just there), no
        sequential-read helper, no kstate guard, no init-time copy
        latency. The cost is that the font occupies 215 B of page 3
        permanently, but page 3 is otherwise unused in most current
        programs.

        Sets:
          self._oled_font_p3_base   — page 3 base address (0)
          self._oled_font_size      — count = N glyphs × 5
          self._oled_font_index_map — char → index (0..N-1)
        """
        if hasattr(self, '_oled_font_p3_base'):
            return
        chars = sorted(self._OLED_FONT_5X5.keys(), key=ord)
        self._oled_font_index_map = {ch: i for i, ch in enumerate(chars)}
        font_bytes = []
        for ch in chars:
            font_bytes.extend(self._OLED_FONT_5X5[ch])
        self._oled_font_p3_base = self.page3_alloc
        self._oled_font_size = len(font_bytes)
        self.page3_globals['__oled_font_p3'] = self._oled_font_p3_base
        self._oled_font_p3_data = font_bytes
        self.page3_alloc += self._oled_font_size

    def _oled_ensure_lut(self):
        """Idempotent: allocate three 32-byte 4bpp expansion LUTs in
        the data page (page 1), keyed by the FULL 5-bit glyph row
        byte. Each LUT_n[row] = the n-th output byte for that row
        pattern, with no runtime shift/mask needed:

            byte 0 = (bit4 ? 0xF0 : 0) | (bit3 ? 0x0F : 0)   ← LUT0
            byte 1 = (bit2 ? 0xF0 : 0) | (bit1 ? 0x0F : 0)   ← LUT1
            byte 2 = (bit0 ? 0xF0 : 0)                       ← LUT2

        Total page-1 cost: 96 B (was 4 B for the 2-bit-keyed LUT).
        Per byte send in __oled_stream_chars drops from:
            mov $c,$a; derefp3; srl×N; andi 0x03,$a; jal helper
        to:
            mov $c,$a; derefp3; addi LUT_n,$a; deref; jal __i2c_sb
        Saves ~3 instr per row in the streaming helper and eliminates
        the __oled_send_lut_byte sub-helper (-5 B label).
        """
        if hasattr(self, '_oled_lut_base'):
            return
        self._oled_lut_base = self.data_alloc
        self.data_alloc += 96
        # Build the 3 × 32-entry LUTs.
        lut0 = []
        lut1 = []
        lut2 = []
        for row in range(32):
            b4 = (row >> 4) & 1
            b3 = (row >> 3) & 1
            b2 = (row >> 2) & 1
            b1 = (row >> 1) & 1
            b0 = row & 1
            lut0.append((0xF0 if b4 else 0) | (0x0F if b3 else 0))
            lut1.append((0xF0 if b2 else 0) | (0x0F if b1 else 0))
            lut2.append(0xF0 if b0 else 0)
        self._oled_lut_data = lut0 + lut1 + lut2

    def _emit_oled_putc(self, ch, xv, yv):
        """Emit one Phase-3-style oled_putc call for char `ch` at cell
        (xv, yv). Allocates a 27-byte glyph table in the data page,
        registers the per-K helper if first time, and emits the
        `ldi $d, addr; jal __oled_putc_xfer` call sequence.

        Used by both the `oled_putc(c, x, y)` builtin (compile-time
        const args) and the `oled_text(s)` builtin (compiler-tracked
        cursor walking the string at emit time).
        """
        if not (0 <= xv <= 20):
            raise Exception(
                f"oled_putc emission: x must be in [0, 20], got {xv}")
        if not (0 <= yv <= 17):
            raise Exception(
                f"oled_putc emission: y must be in [0, 17], got {yv}")
        glyph = self._OLED_FONT_5X5.get(ch)
        if glyph is None:
            raise Exception(
                f"oled_putc emission: char {ch!r} (0x{ord(ch):02X}) is not "
                f"in the embedded 5x5 font. Supported: "
                f"{''.join(sorted(self._OLED_FONT_5X5.keys()))}")
        self._oled_ensure_init()
        col_s = xv * 3
        col_e = col_s + 2
        row_s = yv * 7
        row_e = row_s + 6
        # Per-call data table: 6 window bytes + 21 pixel bytes.
        table = [0x15, col_s, col_e, 0x75, row_s, row_e] + \
            self._oled_render_5x5_to_4bpp(glyph)
        tbl_base = self.data_alloc
        self.data_alloc += len(table)
        if not hasattr(self, '_oled_putc_tables'):
            self._oled_putc_tables = []
        self._oled_putc_tables.append((tbl_base, table, ch, xv, yv))
        self._lcd_helpers.add('__oled_putc_xfer')
        self.emit(f'\tldi $d,{tbl_base}')
        self.emit('\tjal __oled_putc_xfer')

    def _oled_ensure_init(self):
        """Idempotent: allocate the SSD1327 init data in the data
        section (once), register the `__oled_init` walker plus its
        I²C dependencies, and emit `jal __oled_init` (once). Any of
        the oled_init/oled_fill/oled_clear builtins can call this on
        every invocation — subsequent calls are no-ops.

        The init transaction is one I²C burst with DISPLAYOFF, the
        full chip config, CMDLOCK unlock and DISPLAYON in that order
        — matches the Adafruit driver. Followed by SETCONTRAST=0x2F
        and a full-screen address window (0,0..63,127 in 2-pixel
        column units / pixel rows). Total init payload is ~75 bytes
        in the data section.
        """
        if not hasattr(self, '_oled_init_data'):
            OSTART, OSTOP, OEND, ODELAY = 0xF1, 0xF2, 0xF0, 0xF3
            OLED = 0x7A           # SSD1327 7-bit addr 0x3D shifted for write
            CMD_STREAM = 0x00
            init = (
                [ODELAY] * 5 +    # ~25 ms power-on settle
                [OSTART, OLED, CMD_STREAM,
                 0xAE,             # DISPLAYOFF
                 0x81, 0x80,        # SETCONTRAST (overridden below)
                 0xA0, 0x51,        # SEGREMAP
                 0xA1, 0x00,        # SETSTARTLINE
                 0xA2, 0x00,        # SETDISPLAYOFFSET
                 0xA4,             # NORMALDISPLAY (resume from RAM)
                 0xA8, 0x7F,        # SETMULTIPLEX = 1/128
                 0xB1, 0x11,        # PHASELEN
                 0xB3, 0x00,        # DCLK
                 0xAB, 0x01,        # REGULATOR on
                 0xB6, 0x04,        # PRECHARGE2
                 0xBE, 0x0F,        # SETVCOM
                 0xBC, 0x08,        # PRECHARGE1
                 0xD5, 0x62,        # FUNCSELB
                 0xFD, 0x12,        # CMDLOCK unlock
                 0xAF,             # DISPLAYON (in same transaction)
                 OSTOP] +
                [ODELAY] * 20 +   # ~100 ms after init
                [OSTART, OLED, CMD_STREAM, 0x81, 0x2F, OSTOP] +
                # Full-screen address window:
                #   0x15 col_start col_end (cols are 2-px units, 0..63)
                #   0x75 row_start row_end (rows are pixels, 0..127)
                [OSTART, OLED, CMD_STREAM,
                 0x15, 0x00, 0x3F,
                 0x75, 0x00, 0x7F,
                 OSTOP] +
                [OEND]
            )
            data_base = self.data_alloc
            self.data_alloc += len(init)
            self._oled_init_data = (data_base, init)
        if not hasattr(self, '_lcd_helpers'):
            self._lcd_helpers = set()
        self._lcd_helpers.add('__oled_init')
        self._lcd_helpers.add('__i2c_sb')
        self._lcd_helpers.add('__i2c_st_only')
        self._lcd_helpers.add('__i2c_sp')
        if not getattr(self, '_oled_init_emitted', False):
            self.emit('\tjal __oled_init')
            self._oled_init_emitted = True

    def emit_unrolled_i2c_fill_const(self, val, count):
        """Emit a tight asm loop that sends `val` over I²C `count`
        times, using the unrolled byte-send body. Uses $c as the
        counter (preserved across the unrolled body since none of its
        ddrb_imm/exrw instructions touch $c).

        Supported counts: 1 ≤ count ≤ 65536. count > 256 must be a
        multiple of 256 (uses an outer/inner pair with $c saved on
        the stack). For non-multiple counts > 256 the user should
        split the work themselves — keeps the builtin simple and the
        emitted code predictable.

        Code-size: ~52B for the unrolled body + ~6B loop bookkeeping
        (single loop) or ~14B (nested). Fast: ~127 cyc/byte at the
        bus, dominated by the 28 ddrb_imm pulses themselves."""
        if not (1 <= count <= 65536):
            raise Exception(
                f"i2c_fill_const count must be in [1, 65536], got {count}")
        val &= 0xFF
        if count <= 256:
            lbl = self.label('ifc_lp')
            # ldi $c, N where N=256 wraps to 0 and decc decrements 256
            # iterations (0→255→...→1, jnz exits when $c hits 0).
            self.emit(f'\tldi $c,{count & 0xFF}')
            self.emit(f'{lbl}:')
            self.emit_unrolled_i2c_send_const(val)
            self.emit('\tdecc')
            self.emit(f'\tjnz {lbl}')
        else:
            if count % 256 != 0:
                raise Exception(
                    f"i2c_fill_const count > 256 must be a multiple of 256, "
                    f"got {count}. Split into a {count // 256 * 256}-byte "
                    f"fill plus a {count % 256}-byte fill.")
            outer = count // 256
            outer_lbl = self.label('ifc_o')
            inner_lbl = self.label('ifc_i')
            self.emit(f'\tldi $c,{outer & 0xFF}')   # outer count
            self.emit(f'{outer_lbl}:')
            self.emit('\tpush $c')                  # save outer
            self.emit('\tldi $c,0')                 # inner = 256 via wrap
            self.emit(f'{inner_lbl}:')
            self.emit_unrolled_i2c_send_const(val)
            self.emit('\tdecc')
            self.emit(f'\tjnz {inner_lbl}')
            self.emit('\tpop $c')                   # restore outer
            self.emit('\tdecc')
            self.emit(f'\tjnz {outer_lbl}')

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

    def _cold_arg_prologue(self):
        """Lines a cold helper body prepends to recover its $a
        parameter from kstate cold_call_arg. The dispatcher saves
        caller's $a there before its own setup clobbers $a; the
        helper restores it as its first instructions. 3 bytes
        (`ldi $a,0xB5; deref2`)."""
        slot = KERNEL_STATE['cold_call_arg']
        return [f'\tldi $a,{slot.offset}', '\tderef2']

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
                'ee64_read_to_code', 'cold_call',
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
        'rtc_read_reg',    # uses __i2c_rb / __i2c_sb (both clobber $d)
        'i2c_read_byte',   # __i2c_rb directly
        'eeprom_read_byte',
        'eeprom_read',
        'eeprom_write_byte',
        'eeprom_write',
        'ee64_write_byte',
        'cold_call',
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
                 'rtc_read_temp', 'rtc_read_seconds',
                 'eeprom_read_byte',
                 'eeprom_read', 'eeprom_write_byte', 'eeprom_write'}
    # NOTE: rtc_read_reg is intentionally NOT in I2C_CALLS. It lowers to a
    # callable shared helper rather than inlining the I2C transaction, so
    # call sites are tiny (4 B) and don't justify a domain split. Putting
    # it in I2C_CALLS triggered phase-splitter behaviour on
    # overlay_dashboard.c that created a 74 B _main_p1 wrap instead of
    # fitting the (smaller) post-alias main into a single overlay.
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

    def _wrap_main_in_synthetic_ee64(self, functions, n_chunks=1):
        """Phase 4: wrap `main`'s body in `n_chunks` synthetic ee64
        functions, each tagged for cold-tier promotion. Main becomes a
        sequence of `_synth_main_body_K()` calls plus a defensive halt.

        n_chunks=1 (the default): one synthetic with the entire body —
        what the outer auto-cold loop tries first.

        n_chunks>1: lift all top-level `('local', ...)` declarations
        in main's body to globals (so values flow across chunk
        boundaries), then split the remaining statements into N
        contiguous chunks of similar count. Each chunk becomes
        `_synth_main_body_K`. Locals declared inside nested blocks
        (if/while/etc.) are unaffected — they live entirely within
        whichever chunk owns the surrounding block.

        Doesn't handle:
          - Local arrays (`('local_arr', ...)`) — left in place; if
            referenced across chunks, splitting will fail at compile
            time and the outer loop will fall back.
          - Splits inside nested control flow — only top-level stmt
            boundaries are considered.

        If a chunk's compiled body still exceeds COLD_SLOT_BYTES, the
        existing slot-size check raises and the outer loop bumps
        n_chunks until it fits or hits a cap.

        No-op if main isn't found or any `_synth_main_body*` already
        present (avoids double-wrapping if the pass re-runs).
        """
        SYNTH_BASE = '_synth_main_body'
        # Find main and check we haven't already wrapped.
        main_idx = None
        for i, fn in enumerate(functions):
            if fn[0] == 'main':
                main_idx = i
            if fn[0].startswith(SYNTH_BASE):
                return functions   # already wrapped
        if main_idx is None:
            return functions       # no main, nothing to wrap

        main_name, main_params, main_body, main_ret = functions[main_idx]
        if main_body[0] != 'block':
            return functions
        stmts = list(main_body[1])

        if n_chunks <= 1:
            # Single-chunk path: body untouched, one synthetic.
            synth_name = SYNTH_BASE
            synth_fn = (synth_name, [], main_body, 'void')
            self.cold_function_names.add(synth_name)
            new_main_body = ('block', [
                ('expr_stmt', ('call', synth_name, [])),
                ('expr_stmt', ('call', 'halt', [])),
            ])
            new_main_fn = (main_name, main_params, new_main_body, main_ret)
            out = []
            for i, fn in enumerate(functions):
                out.append(new_main_fn if i == main_idx else fn)
            out.append(synth_fn)
            return out

        # Multi-chunk path: promote top-level locals to globals.
        # Pulled-out declarations turn into:
        #   (a) An entry in `_synth_promoted_globals` (consumed by
        #       compile() right after this transform — see hook).
        #   (b) An `('expr_stmt', assign-init)` left in the chunk if
        #       the decl had an initializer (so the global gets
        #       initialized at runtime in the same place).
        promoted = []
        body_stmts = []
        for s in stmts:
            if s[0] == 'local':
                _, name, init, typ = s
                if name not in {p[0] for p in promoted}:
                    promoted.append((name, 0, 'ram'))
                if init is not None:
                    # Keep the initializer as an assignment statement
                    # so codegen still runs it at the original spot.
                    body_stmts.append(
                        ('expr_stmt', ('assign', '=', ('var', name), init)))
            else:
                body_stmts.append(s)

        # Stash promoted globals on self so compile() can consume them
        # before any per-function compilation runs. (We can't mutate
        # the parser's `globals_` list from here; this is the
        # documented hand-off.)
        self._synth_promoted_globals = promoted

        # Even split by statement count.
        total = len(body_stmts)
        chunk_sz = max(1, (total + n_chunks - 1) // n_chunks)
        chunks = []
        for ci in range(n_chunks):
            start = ci * chunk_sz
            end = min(start + chunk_sz, total)
            if start >= end:
                continue
            chunks.append(body_stmts[start:end])
        if len(chunks) < 2:
            # Degenerate split — fall back to single-chunk path.
            return self._wrap_main_in_synthetic_ee64(functions, n_chunks=1)

        # Build the synthetic functions and the new main.
        synth_fns = []
        new_main_stmts = []
        for ci, chunk_stmts in enumerate(chunks):
            sname = f'{SYNTH_BASE}_{ci}'
            self.cold_function_names.add(sname)
            synth_fns.append((sname, [], ('block', chunk_stmts), 'void'))
            new_main_stmts.append(('expr_stmt', ('call', sname, [])))
        # Defensive halt — usually one of the chunks ends in halt() but
        # if the body lacked a terminating halt, keep main from running
        # off into garbage.
        new_main_stmts.append(('expr_stmt', ('call', 'halt', [])))

        new_main_body = ('block', new_main_stmts)
        new_main_fn = (main_name, main_params, new_main_body, main_ret)

        out = []
        for i, fn in enumerate(functions):
            out.append(new_main_fn if i == main_idx else fn)
        out.extend(synth_fns)
        return out

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

    def _is_rtc_read_reg_alias(self, name, params, body, ret_type):
        """Recognize a user-defined DS3231 register-read function whose body
        is structurally identical to the compiler's `rtc_read_reg(reg)`
        builtin.

        Pattern (`overlay_dashboard.c`'s rtc_reg, others may vary):
          unsigned char NAME(unsigned char reg) {
              i2c_start();
              i2c_send_byte(0xD0);
              i2c_send_byte(reg);
              [i2c_stop(); i2c_start(); | i2c_repeated_start();]
              i2c_send_byte(0xD1);
              unsigned char v;
              v = i2c_read_byte();
              i2c_nack();
              i2c_stop();
              return v;
          }

        When matched, the function is dropped from the program and every
        `('call', name, [arg])` in the rest of the corpus is rewritten to
        `('call', 'rtc_read_reg', [arg])`. The compiler's rtc_read_reg
        emitter then takes over (callable shared helper, smaller per-call
        cost than user's bespoke function-overlay).
        """
        if len(params) != 1 or body[0] != 'block':
            return False
        p_name = params[0][0] if isinstance(params[0], tuple) else params[0]
        body_s = str(body)
        # Required tokens: device address writes, reg-byte send via param,
        # read of D1 + i2c_read_byte + nack + stop + return v.
        required = [
            "('call', 'i2c_start',",
            "('call', 'i2c_send_byte', [('num', 208)])",     # 0xD0 device write addr
            f"('call', 'i2c_send_byte', [('var', '{p_name}')])",  # reg
            "('call', 'i2c_send_byte', [('num', 209)])",     # 0xD1 device read addr
            "('call', 'i2c_read_byte', [])",
            "('call', 'i2c_nack', [])",
            "('call', 'i2c_stop',",
            "('return',",
        ]
        return all(tok in body_s for tok in required)

    def _alias_builtin_compatible_functions(self, functions):
        aliases = set()
        # name -> builtin name to redirect calls to (when builtin name differs
        # from user name, e.g. user's rtc_reg → rtc_read_reg builtin).
        rename_map = {}
        kept = []
        for name, params, body, ret_type in functions:
            if name in self.cold_function_names:
                # ee64-tagged: user wants this body in cold tier, not
                # replaced with a kernel-resident builtin. Skip aliasing.
                kept.append((name, params, body, ret_type))
                continue
            if self._is_builtin_i2c_send_byte_alias(name, params, body, ret_type):
                aliases.add(name)
                continue
            if self._is_rtc_read_reg_alias(name, params, body, ret_type):
                aliases.add(name)
                rename_map[name] = 'rtc_read_reg'
                continue
            kept.append((name, params, body, ret_type))
        if rename_map:
            # Rename calls in all kept function bodies. This is the critical
            # step — without it, calls to the dropped function dangle.
            def _rename_calls(node):
                if isinstance(node, tuple):
                    if (len(node) >= 2 and node[0] == 'call'
                            and node[1] in rename_map):
                        new_name = rename_map[node[1]]
                        return ('call', new_name,
                                [_rename_calls(a) for a in node[2]])
                    return tuple(_rename_calls(c) for c in node)
                if isinstance(node, list):
                    return [_rename_calls(c) for c in node]
                return node
            kept = [(n, p, _rename_calls(b), r) for (n, p, b, r) in kept]
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
        # Cold C-function names (declared `ee64 void f(...) {...}`).
        # Codegen captures their bodies into the AT24C512 image and
        # rewrites direct `jal _f` to `cold_call(id)` via __cold_call.
        self.cold_function_names = set(parser.cold_function_names)
        # Auto-cold promotions from the outer retry: user functions the
        # partitioner couldn't place because they exceed the overlay
        # slot. Treated identically to source-level `ee64` tags.
        self.cold_function_names |= getattr(self, '_auto_cold_user_fns', set())

        # Phase 4: Auto main-body extraction. When the outer auto-cold
        # loop has exhausted its other promotions and the program still
        # overflows, set `_extract_main_body=True` and re-compile with
        # main's body wrapped in a synthetic `_synth_main_body` cold
        # function. The synthetic gets auto-promoted to ee64 by the
        # standard cold-function path (same code that handles
        # source-level `ee64 void f(...)` and Phase 1 auto-cold).
        # Effect: main shrinks to a single `cold_call(synth_id)`,
        # freeing kernel-page bytes for dispatcher infrastructure
        # plus the cold slot.
        if getattr(self, '_extract_main_body', False):
            n_chunks = getattr(self, '_extract_main_body_chunks', 1)
            functions = self._wrap_main_in_synthetic_ee64(
                functions, n_chunks=n_chunks)
            # Multi-chunk path lifts main's locals to globals so values
            # flow across chunk boundaries. Merge those into the
            # parser's `globals_` list before allocation runs below.
            promoted = getattr(self, '_synth_promoted_globals', [])
            if promoted:
                # Avoid duplicate names if user code already has a same-
                # named global.
                existing = {g[0] for g in globals_}
                for name, init, storage in promoted:
                    if name in existing:
                        continue
                    globals_.append((name, init, storage))
                    existing.add(name)

        self.page3_globals = {}  # name → page 3 address (for overflow arrays)
        self.page3_alloc = 0
        self.eeprom_globals = {}  # name → EEPROM address
        self.eeprom_alloc = 0x0010  # skip 16-byte header (checksum + reserved)
        self.eeprom_data = []  # list of (addr, bytes) for upload
        # Cold tier (AT24C512). Each `ee64` array becomes a cold helper
        # with an auto-assigned id (declaration order). The compiler
        # emits its bytes into the ee64 image (uploaded by ESP32 to
        # AT24C512) and stage-1 init code that populates the cold
        # dispatcher's metadata table at page3[COLD_TABLE_BASE].
        self.ee64_globals = {}            # name → ee64 address
        self.ee64_alloc = 0
        self.ee64_data = []               # (addr, bytes) for upload
        self.cold_helpers = []            # (id, ee64_addr, length, name)
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
            elif storage == 'ee64':
                # Cold-tier helper. Bytes get uploaded to AT24C512;
                # implicit cold-helper id assigned by declaration order.
                if not isinstance(init, list):
                    raise Exception(f"ee64 variable '{name}' must be an initialized array")
                if size > COLD_SLOT_BYTES:
                    raise Exception(
                        f"ee64 helper '{name}' is {size} B, exceeds slot "
                        f"size {COLD_SLOT_BYTES} B (COLD_SLOT_BYTES). "
                        "Either shrink the helper or grow the slot.")
                cold_id = len(self.cold_helpers)
                self.ee64_globals[name] = self.ee64_alloc
                self.ee64_data.append((self.ee64_alloc, init))
                self.cold_helpers.append((cold_id, self.ee64_alloc, size, name))
                self.ee64_alloc += size
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

        # Cold functions go to a separate buffer — their bytes get written
        # into the AT24C512 image rather than the main code stream, and
        # internal labels need to resolve to the cold-slot address (set
        # via `org COLD_SLOT_BASE` when emitted later). Non-cold helpers
        # compile inline as normal.
        cold_fns_to_compile = [fn for fn in other_fns
                               if fn[0] in self.cold_function_names]
        warm_fns = [fn for fn in other_fns
                    if fn[0] not in self.cold_function_names]
        for fn in warm_fns:
            self.compile_function(*fn)

        # Compile cold functions one at a time, capturing each function's
        # asm lines into `cold_function_bodies`. Each one gets registered
        # as a cold helper (auto-id by current cold_helpers length, ee64
        # address from the bump allocator). Body byte size is enforced
        # against COLD_SLOT_BYTES so a too-large helper fails compile
        # rather than silently corrupting kernel state at runtime. The
        # actual slot size and base address are right-sized after
        # compilation: `cold_slot_size = max(byte_size for cold funcs)`
        # (or 0 if no cold tier active), `cold_slot_base = 256 -
        # cold_slot_size`.
        self.cold_function_bodies = []
        self.cold_slot_size = 0
        self.cold_slot_base = 256  # sentinel: no cold tier yet
        if cold_fns_to_compile:
            saved_code = self.code
            for fn in cold_fns_to_compile:
                self.code = []
                self.compile_function(*fn)
                # Apply peephole here, BEFORE measuring + recording
                # ee64 offsets. The main-stream peephole pass runs later
                # over self.code (which by then contains the cold body
                # in section ee64); without doing it here too, the
                # metadata-table `len` reflects pre-peephole size but
                # the actual ee64-image bytes are post-peephole, so the
                # dispatcher reads N bytes when only M < N exist and
                # loads garbage past byte M into the slot.
                cold_lines = peephole(list(self.code))
                # Caller-arg prologue: if this cold function takes a
                # parameter (first arg in $a), the dispatcher already
                # clobbered $a by the time the slot runs. Insert a
                # 3-byte load from the kstate `cold_call_arg` slot
                # AFTER the function-entry label so the body sees
                # its parameter at entry. (Insertion-after-label
                # rather than prepend: dead-code elim drops items
                # between the previous helper's `ret` and the next
                # top-level label; placing the prologue past the
                # label makes it survive that pass.)
                if self.func_params.get(fn[0], 0) > 0:
                    # Find the function's entry label inside cold_lines
                    # and insert prologue right after it. If no label
                    # is found (shouldn't happen — compile_function
                    # emits one), fall back to prepending.
                    _label_idx = None
                    for _i, _l in enumerate(cold_lines):
                        _ls = _l.strip()
                        if (_ls.endswith(':') and not _ls.startswith('.')
                                and not _ls.startswith(';')):
                            _label_idx = _i
                            break
                    if _label_idx is None:
                        cold_lines = self._cold_arg_prologue() + cold_lines
                    else:
                        cold_lines = (cold_lines[:_label_idx + 1]
                                      + self._cold_arg_prologue()
                                      + cold_lines[_label_idx + 1:])
                byte_size = _measure_asm_bytes(cold_lines)
                if byte_size > COLD_SLOT_BYTES:
                    raise Exception(
                        f"cold function '{fn[0]}' is {byte_size} B, "
                        f"exceeds slot size {COLD_SLOT_BYTES} B "
                        f"(COLD_SLOT_BYTES). Either shrink the function "
                        f"or grow the slot.")
                cold_id = len(self.cold_helpers)
                ee64_addr = self.ee64_alloc
                self.cold_helpers.append(
                    (cold_id, ee64_addr, byte_size, fn[0]))
                self.cold_function_bodies.append(
                    (cold_id, fn[0], cold_lines, byte_size, ee64_addr))
                self.ee64_alloc += byte_size
            self.code = saved_code
        # Right-size the slot to the largest cold body. Programs with
        # cold byte-array helpers (`ee64 unsigned char[]`) also count
        # — the dispatcher loads them into the same slot.
        for _cid, _addr, sz, _name in self.cold_helpers:
            if sz > self.cold_slot_size:
                self.cold_slot_size = sz
        if self.cold_slot_size > 0:
            self.cold_slot_base = 256 - self.cold_slot_size

        # Rewrite direct calls to cold functions: `jal _foo` becomes
        # `ldi $a,id; jal __cold_call`. Applies to BOTH the main code
        # stream AND cold function bodies — the latter enables cold→cold
        # chaining (a cold function calling another cold function
        # re-enters the dispatcher, which save/restores the
        # currently-loaded helper around the callee's run).
        # Must run after cold functions are registered (so the id map
        # exists) and before _overlay_partition (which treats self.code
        # as definitive).
        # If Phase 2 helper auto-cold is active (force-cold helpers
        # populated by the outer retry in _prepare_compile), the cold
        # dispatcher infrastructure must be emitted even when no source
        # `ee64` user function exists. Pre-register dispatcher helpers
        # in _lcd_helpers so `_emit_i2c_helpers` emits the bodies.
        if self._force_cold_helpers:
            self._lcd_helpers = getattr(self, '_lcd_helpers', set())
            self._lcd_helpers |= {'__i2c_sb', '__i2c_rb',
                                  '__cold_call', '__cold_load_helper'}

        if self.cold_function_names:
            cold_call_map = {fname: cid
                             for cid, _addr, _size, fname in self.cold_helpers
                             if fname in self.cold_function_names}
            self._lcd_helpers = getattr(self, '_lcd_helpers', set())
            # __cold_call calls __cold_load_helper for both the
            # initial-load and the post-callee reload path. The bulk-
            # read loop is inlined inside __cold_load_helper so
            # __eeprom_r2c_loop doesn't need to be kernel-resident.
            self._lcd_helpers |= {'__i2c_sb', '__i2c_rb',
                                  '__cold_call', '__cold_load_helper'}

            def _rewrite_cold_calls(lines):
                rewritten = []
                for line in lines:
                    s = line.strip()
                    if s.startswith('jal _'):
                        target = s[len('jal _'):]
                        if target in cold_call_map:
                            cid = cold_call_map[target]
                            # Pass id in $b so caller's $a (the helper's
                            # parameter) is intact at dispatcher entry.
                            # `push_imm` would set $a := imm as a side
                            # effect (its microcode loads imm→A before
                            # pushing), which destroys caller's $a;
                            # `ldi $b, X` only writes $b. The `;!keep`
                            # marker tells peephole not to elide this
                            # even when register-tracking thinks $b
                            # already holds the right value at this
                            # point — the dispatcher's contract requires
                            # the load to actually execute.
                            rewritten.append(f'\tldi $b,{cid} ;!keep')
                            rewritten.append('\tjal __cold_call')
                            continue
                    rewritten.append(line)
                return rewritten

            self.code = _rewrite_cold_calls(self.code)
            # Also rewrite within cold function bodies. Each body is
            # (id, name, lines, byte_size, ee64_addr); re-measure size
            # post-rewrite and reallocate ee64 addresses sequentially
            # (rewriting can change body length: 2 B `jal _b` → 4 B
            # `ldi $a,N; jal __cold_call`).
            if self.cold_function_bodies:
                new_bodies = []
                self.ee64_alloc = 0  # reset and re-pack
                self.cold_helpers = []  # rebuild
                for cold_id, fname, body, _old_size, _old_addr in self.cold_function_bodies:
                    rewritten_body = _rewrite_cold_calls(body)
                    new_size = _measure_asm_bytes(rewritten_body)
                    if new_size > COLD_SLOT_BYTES:
                        raise Exception(
                            f"cold function '{fname}' is {new_size} B "
                            f"after cold→cold rewriting; exceeds slot "
                            f"size {COLD_SLOT_BYTES} B.")
                    new_addr = self.ee64_alloc
                    self.cold_helpers.append((cold_id, new_addr, new_size, fname))
                    new_bodies.append((cold_id, fname, rewritten_body, new_size, new_addr))
                    self.ee64_alloc += new_size
                self.cold_function_bodies = new_bodies
                # Recompute slot size from new sizes.
                max_cold = max((sz for _id, _a, sz, _n in self.cold_helpers),
                               default=0)
                self.cold_slot_size = max_cold
                if max_cold > 0:
                    self.cold_slot_base = 256 - max_cold

        # (C) Transitive deps: every compiler helper called from a cold
        # function body has to be kernel-resident. A cold function runs
        # from the COLD_SLOT_BASE region; if it does `jal __lcd_chr` and
        # __lcd_chr lives in an overlay at some other address, the call
        # lands wherever the overlay LRU happens to have placed it — i.e.
        # garbage. Collecting the set here so _classify_helpers can force
        # them into the no_ov set.
        self._cold_function_callee_helpers = set()
        for cold_id, fname, cold_lines, _bsz, _addr in self.cold_function_bodies:
            for line in cold_lines:
                s = line.strip()
                # Match `jal __helper` (compiler helpers — leading __).
                # User-function calls (`jal _foo`) are an error from a
                # cold body and would have been caught above if they
                # were cold; warm user calls aren't legal here.
                if s.startswith('jal __'):
                    target = s[len('jal '):].split()[0]
                    self._cold_function_callee_helpers.add(target)

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

        # Phase 2: extract force-cold helpers from self.code into the
        # cold tier. Runs after _emit_i2c_helpers so helper bodies are
        # available to extract. The dispatcher (__cold_call /
        # __cold_load_helper) was emitted by _emit_i2c_helpers because
        # we pre-registered it in _lcd_helpers above.
        self._extract_force_cold_helpers()

        # Option (3): bundle consecutive cold-call sites. Runs AFTER
        # all cold-tier registration so it sees the final helper id
        # space and can append __cold_run_list at the end.
        self._bundle_cold_calls()

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
            # In flat mode there's no kernel self-copy: page 3 is written
            # from `result.page3` at upload and persists. Emit cold-helper
            # metadata as STATIC page-3 bytes via `section page3; org
            # COLD_TABLE_BASE; byte ...` — saves ~9–15 B of code-page
            # preamble per helper compared to the equivalent runtime
            # `ldi $a,V; ldi $b,P; iderefp3` triple. (Init-extraction /
            # overlay mode below still emits runtime init because page 3
            # gets clobbered by the kernel image during self-copy.)
            cold_helpers = getattr(self, 'cold_helpers', [])
            cold_active = bool(cold_helpers)
            for mi, line in enumerate(self.code):
                if line.strip() == '_main:':
                    self.code.insert(mi + 1, '\tldi $b,0xFF')
                    self.code.insert(mi + 2, '\tmov $b,$sp')
                    if cold_active:
                        # cold_current_id = 0xFF (sentinel: nothing
                        # loaded). Required so the first cold_call's
                        # post-callee reload path correctly skips the
                        # caller-reload branch instead of trying to
                        # reload a non-existent caller.
                        self.code.insert(mi + 3, '\tldi $a,0xFF')
                        # Inline the kstate store rather than emit_kstate_store
                        # — that helper writes to self.code which is what
                        # we're currently mutating. The cold_current_id
                        # slot lives at page2[0xB4]; page-2 store is
                        # `ldi $b,0xB4; iderefp3` … wait, page 2 needs
                        # ideref2 not iderefp3. Use the helper output.
                        self.code.insert(mi + 4, '\tldi $b,0xB4')
                        self.code.insert(mi + 5, '\tideref2')
                    break
            if cold_helpers:
                self.emit('\tsection page3')
                self.emit(f'\torg {COLD_TABLE_BASE}')
                self.emit('; cold-helper metadata table (static; page 3 persists in flat mode)')
                for cold_id, ee_addr, length, _name in cold_helpers:
                    self.emit(f'\tbyte {(ee_addr >> 8) & 0xFF}    ; helper {cold_id} addr_hi')
                    self.emit(f'\tbyte {ee_addr & 0xFF}    ; helper {cold_id} addr_lo')
                    self.emit(f'\tbyte {length & 0xFF}    ; helper {cold_id} len')
                # Cold-call bundle lists (option 3): emit (id, arg)
                # byte pairs terminated by 0xFF, immediately after the
                # metadata table. Read at runtime by __cold_run_list.
                _list_data = getattr(self, '_cold_list_data', None)
                if _list_data:
                    self.emit(f'; cold-call bundle lists ({len(_list_data)} bytes)')
                    cur_addr = COLD_TABLE_BASE + 3 * len(cold_helpers)
                    for p3_addr, byte_val in _list_data:
                        if p3_addr != cur_addr:
                            self.emit(f'\torg {p3_addr}')
                            cur_addr = p3_addr
                        self.emit(f'\tbyte {byte_val}')
                        cur_addr += 1
                self.emit('\tsection code')

        # Emit page 1 globals UNLESS the overlay partitioner already inlined
        # them (overlay mode emits globals inside the data section before
        # overlay bodies — that path sets _overlay_kernel_size). Init-
        # extraction mode does NOT emit globals separately, so we still
        # need to emit them here even when _init_extraction_done is True;
        # otherwise data[0..globals_size-1] is left empty and any compile-
        # time data_base computed AFTER globals (e.g. lcd_init) lands at
        # the wrong runtime offset (lcd_init expects keymap to occupy
        # data[0..15], so its data sits at data[16..]). Without this
        # emission, the data section opens with lcd_init bytes at offset 0
        # and `keymap[k]` deref reads garbage.
        page1_vars = self._page1_globals_cache
        overlays_handled_globals = getattr(self, '_overlay_kernel_size', None) is not None
        if page1_vars and not overlays_handled_globals:
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

        # Compiler-internal page-3 globals (not C-declared). Currently
        # only the SSD1327 OLED 5x5 font cache (215 B at the start of
        # page 3, populated at compile time so no runtime EEPROM read
        # is needed).
        if hasattr(self, '_oled_font_p3_data'):
            self.emit('\tsection page3')
            self.emit(f'; OLED 5x5 font cache at page3[{self._oled_font_p3_base}..{self._oled_font_p3_base+self._oled_font_size-1}] '
                      f'(43 glyphs × 5 bytes)')
            for b in self._oled_font_p3_data:
                self.emit(f'\tbyte {b}')
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

        # OLED init data (same shape as LCD init data, different sentinels).
        if hasattr(self, '_oled_init_data') and not getattr(self, '_oled_init_already_emitted', False):
            data_base, oled_bytes = self._oled_init_data
            self.emit('\tsection data')
            self.emit(f'; OLED (SSD1327) init data at data[{data_base}..{data_base+len(oled_bytes)-1}]')
            for b in oled_bytes:
                self.emit(f'\tbyte {b}')
            self.emit('\tsection code')

        # OLED putc per-call data tables (6 window bytes + 21 pixel bytes
        # each). The runtime helper `__oled_putc_xfer` walks one of these
        # tables to do a single character render.
        for tbl_base, table, ch, xv, yv in getattr(self, '_oled_putc_tables', []):
            self.emit('\tsection data')
            self.emit(f'; oled_putc {ch!r} at ({xv},{yv}) table @ data[{tbl_base}..{tbl_base+len(table)-1}]')
            for b in table:
                self.emit(f'\tbyte {b}')
            self.emit('\tsection code')

        # OLED text_at index buffers (one byte per char in the literal,
        # page-1 storage). The runtime stream helper walks each buffer.
        for buf_base, buf, s, xv, yv in getattr(self, '_oled_text_at_bufs', []):
            self.emit('\tsection data')
            self.emit(f'; oled_text_at({xv},{yv},{s!r}) idx buf @ data[{buf_base}..{buf_base+len(buf)-1}]')
            for b in buf:
                self.emit(f'\tbyte {b}')
            self.emit('\tsection code')

        # OLED 4bpp expansion LUTs (3 × 32 bytes in page 1) — keyed by
        # the full 5-bit glyph row byte. LUT_n[row] is the n-th 4bpp
        # output byte for that row pattern. See _oled_ensure_lut().
        if hasattr(self, '_oled_lut_base'):
            self.emit('\tsection data')
            self.emit(f'; OLED 4bpp expansion LUTs at data[{self._oled_lut_base}..{self._oled_lut_base+95}] '
                      f'(3 × 32 entries: byte0/byte1/byte2)')
            for b in self._oled_lut_data:
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

        # Emit ee64 data section (for ESP32 upload to AT24C512).
        # Mixes raw byte arrays (from `ee64 unsigned char[]` decls) and
        # compiled cold-function bodies (from `ee64 void f() {…}` decls).
        # Both share the AT24C512 image; both are loaded into the cold
        # slot at runtime via __cold_call.
        cold_bodies = getattr(self, 'cold_function_bodies', [])
        if self.ee64_data or cold_bodies:
            self.emit('\tsection ee64')
            # Raw byte arrays first — sequential, no `org` needed.
            for addr, data in self.ee64_data:
                self.emit(f'; ee64 array at 0x{addr:04X} ({len(data)} bytes)')
                for b in data:
                    self.emit(f'\tbyte {b}')
            # Compiled cold function bodies. Each gets `org COLD_SLOT_BASE`
            # before emission so its internal labels resolve to slot
            # addresses (not ee64-buffer addresses). The firmware
            # assembler's `org` only affects the code-PC counter (used
            # for label resolution); ee64-buffer write position is
            # independent and continues from wherever it was, so byte
            # addresses in the AT24C512 image remain contiguous.
            for cold_id, fname, cold_lines, byte_size, ee64_addr in cold_bodies:
                self.emit(f'; ee64 cold function _{fname} (id {cold_id}, '
                          f'{byte_size}B at ee64[0x{ee64_addr:04X}])')
                self.emit(f'\torg {self.cold_slot_base}')
                for line in cold_lines:
                    self.emit(line)
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
        # Keep __i2c_sb alive when i2c_init() was called — needed for
        # EEPROM preload (generated later in _overlay_partition) and used
        # by virtually every program that calls i2c_init(). __i2c_rb
        # used to also be force-kept here but the EEPROM preload is
        # inline (no __i2c_rb dependency) and runtime read users
        # (RTC, eeprom_read_byte, i2c_read_byte) add it themselves;
        # keeping it unconditionally cost ~38 B resident on write-only
        # programs (LCD/OLED/tone) for nothing.
        if getattr(self, '_i2c_init_called', False):
            refs.add('__i2c_sb')
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
        # OLED runtime helpers are runtime callers of __i2c_st_only /
        # __i2c_sp. __oled_init is NOT in this set: it's init-only
        # (one-shot, called from stage-1 oled_init() and never reached
        # at runtime). Including it here added __oled_init: to
        # _NO_OVERLAY upstream, which made it kernel-resident and
        # bypassed the INIT_ONLY_NAMES classification (~51 B kernel
        # bloat per program with OLED).
        #
        # __oled_fill_* helpers: call-site-aware. Each K is added to
        # the resident set ONLY if it's used at runtime (any call
        # inside a loop or in a non-main user function). K values used
        # exclusively from main's init prefix are init-only; the
        # oled_fill builtin tags runtime-context callers in
        # `_oled_fill_runtime_use`.
        oled_runtime = {'__oled_putc_xfer', '__oled_stream_chars',
                        '__oled_set_window'}
        # Add only the K helpers whose call sites include a runtime use.
        _ofu = getattr(self, '_oled_fill_runtime_use', set())
        for _fk in _ofu:
            _hn = f'__oled_fill_{_fk:02X}'
            if _hn in helpers:
                oled_runtime = oled_runtime | {_hn}
        if helpers & oled_runtime:
            for h in oled_runtime:
                if h in helpers:
                    no_ov.add(f'{h}:')
        if '__i2c_st_only' in helpers and (self._needs_runtime_i2c or (helpers & oled_runtime)):
            no_ov.add('__i2c_st_only:')
        if '__i2c_sp' in helpers and (self._needs_runtime_i2c or (helpers & oled_runtime)):
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

        # __cold_call: dispatcher for runtime AT24C512 helper loads.
        # Always kernel-resident — it must be reachable from any caller
        # site (including overlays). The bulk-read loop is inlined
        # directly into __cold_call's body, so __eeprom_r2c_loop is
        # NOT forced resident here (only `eeprom_read_to_code` callers
        # pull it in, and that path may put it in an overlay).
        if '__cold_call' in helpers:
            no_ov.add('__cold_call:')
            no_ov.add('__cold_load_helper:')

        # (C) Cold-function callees: every compiler helper invoked from a
        # cold-function body has to be kernel-resident — a cold function
        # runs from a fixed slot address and a `jal __X` to an overlayed
        # __X would land wherever the LRU happened to place it. The set
        # is collected during cold-function compilation in compile().
        for h in getattr(self, '_cold_function_callee_helpers', set()):
            if h in helpers:
                no_ov.add(f'{h}:')

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

    def _extract_force_cold_helpers(self):
        """Phase 2 of skinny-kernel auto-paging.

        For each helper name in `self._force_cold_helpers`, locate its
        body in `self.code` and migrate it into the AT24C512 cold tier.
        Call sites in `self.code` (and any pre-existing cold function
        bodies) are rewritten from `jal __helper` to the dispatcher
        sequence `ldi $a,id; jal __cold_call`. The dispatcher itself
        (`__cold_call`, `__cold_load_helper`) was emitted by
        `_emit_i2c_helpers` because we pre-registered it before that
        ran.

        First-cut constraints (intentionally narrow):
          - Helper body must end in `ret` (leaf-style); helpers that
            tail-jump into another helper are rejected.
          - Helper body must fit COLD_SLOT_BYTES.
          - Helpers whose body has any inbound `j __helper` (tail
            branches from elsewhere) are not detected here — relying
            on the helper-resolution check at compile() to catch the
            case where a `j __helper` would dangle after extraction.

        Rejected helpers stay kernel-resident; the outer retry's next
        iteration will see the same wrap and pick a different
        candidate.
        """
        targets = getattr(self, '_force_cold_helpers', set())
        if not targets:
            return

        # Skip helpers already in the cold tier (e.g. from a previous
        # compile pass that left state behind).
        already = {('_' + n.lstrip('_'))[1:]
                   for _id, _addr, _sz, n in self.cold_helpers}
        # `fname` convention used by the call-site rewriter: target name
        # after stripping `jal _` prefix. For `__rtc_read_reg` that is
        # `_rtc_read_reg` (one leading underscore).
        targets = {n for n in targets if (n[1:] not in already)}
        if not targets:
            return

        # Single forward pass: extract bodies of force-cold helpers,
        # leave everything else untouched.
        new_code = []
        extracted_bodies = {}  # name → list of body lines (incl. label)
        i = 0
        while i < len(self.code):
            line = self.code[i]
            s = line.strip()
            if (s.endswith(':') and not s.startswith('.') and
                    not s.startswith(';')):
                name = s[:-1]
                if name in targets:
                    body = [line]
                    j = i + 1
                    while j < len(self.code):
                        ns = self.code[j].strip()
                        if (ns.endswith(':') and not ns.startswith('.') and
                                not ns.startswith(';')):
                            break
                        body.append(self.code[j])
                        j += 1
                    extracted_bodies[name] = body
                    i = j
                    continue
            new_code.append(line)
            i += 1

        accepted = []  # (name, body_without_label_lines)
        rejected_back = []  # full body lines to restore to self.code
        import sys as _ac_sys
        for name, body_lines in extracted_bodies.items():
            label_line = body_lines[0]
            body_only = list(body_lines[1:])
            body_only = peephole(body_only)
            # Kernel helpers that read their first arg from $a
            # (rtc_read_reg, eeprom_rd, i2c_sb, lcd_chr, …) need a
            # prologue to recover $a from kstate — the dispatcher's
            # setup clobbers it. But many kernel helpers don't take
            # any arg (delay_cal, i2c_sp, i2c_rb, …) and can skip
            # the 3-byte prologue, saving slot space (which sets
            # the runtime kernel budget at 256 - cold_slot_size).
            #
            # Detection rule: if the first real instruction writes
            # $a as its first action (`ldi $a,...`, `clr $a`,
            # `mov $X,$a`, `pop $a`), then $a's caller value is
            # immediately discarded — prologue would only restore a
            # value that gets thrown away. Conservative: when in
            # doubt, prepend the prologue.
            _clobbers = _helper_clobbers_a_first(body_only)
            # Body layout: label line first (so dead_code_elim sees a
            # Label to clear in_dead before the prologue), THEN
            # prologue, THEN original body. Without the label-first
            # layout, the prologue lines that follow the previous
            # helper's `ret` get dropped by Pass-1 dead-code elim
            # before reaching the next internal label.
            if _clobbers:
                body = [label_line] + body_only
            else:
                body = [label_line] + self._cold_arg_prologue() + body_only
            size = _measure_asm_bytes(body)
            if size == 0:
                rejected_back.extend(body_lines)
                continue
            if size > COLD_SLOT_BYTES:
                print(f"  Auto-cold (helper): {name} body is {size}B, "
                      f"exceeds slot {COLD_SLOT_BYTES}B; rejected",
                      file=_ac_sys.stderr)
                rejected_back.extend(body_lines)
                continue
            # Find last real instruction.
            last_inst = None
            for b in reversed(body):
                bs = b.strip()
                if (not bs or bs.endswith(':') or bs.startswith(';')):
                    continue
                last_inst = bs.split()[0]
                break
            if last_inst != 'ret':
                print(f"  Auto-cold (helper): {name} doesn't end in `ret` "
                      f"(last={last_inst!r}); rejected",
                      file=_ac_sys.stderr)
                rejected_back.extend(body_lines)
                continue
            accepted.append((name, body, size))

        if rejected_back:
            new_code.extend(rejected_back)
        self.code = new_code

        if not accepted:
            return

        # Register accepted helpers in the cold tier.
        for name, body, size in accepted:
            cold_id = len(self.cold_helpers)
            ee64_addr = self.ee64_alloc
            fname = name[1:]  # __rtc_read_reg → _rtc_read_reg
            self.cold_helpers.append((cold_id, ee64_addr, size, fname))
            self.cold_function_bodies.append(
                (cold_id, fname, body, size, ee64_addr))
            self.ee64_alloc += size
            print(f"  Auto-cold (helper): {name} → cold tier id={cold_id}, "
                  f"{size}B at ee64[0x{ee64_addr:04X}]",
                  file=_ac_sys.stderr)

        # Right-size cold slot to the largest body.
        max_cold = max((sz for _id, _a, sz, _n in self.cold_helpers),
                       default=0)
        if max_cold > self.cold_slot_size:
            self.cold_slot_size = max_cold
        if self.cold_slot_size > 0:
            self.cold_slot_base = 256 - self.cold_slot_size

        # Rewrite `jal __helper` (and `j __helper`) call sites to the
        # cold-call dispatcher sequence. Same convention as the user-fn
        # rewriter at line ~2892: target = line.lstrip 'jal _' / 'j _'.
        cold_call_map = {fname: cid
                         for cid, _addr, _size, fname in self.cold_helpers}

        def _rewrite(lines):
            out = []
            for line in lines:
                s = line.strip()
                for prefix in ('jal _', 'j _'):
                    if s.startswith(prefix):
                        target = s[len(prefix):].split()[0].split(';')[0].strip()
                        if target in cold_call_map:
                            cid = cold_call_map[target]
                            # Pass id in $b (preserves caller's $a).
                            out.append(f'\tldi $b,{cid} ;!keep')
                            out.append('\tjal __cold_call')
                            if prefix == 'j _':
                                out.append('\tret')
                            break
                else:
                    out.append(line)
                    continue
                # break-from-prefix-loop fallthrough = handled
            return out

        self.code = _rewrite(self.code)
        # Rewrite within already-extracted cold function bodies (cold→
        # cold). Each call inflates from 2B to 4B (and tail-jumps to 5B),
        # so re-measure and re-pack ee64 addresses sequentially.
        if self.cold_function_bodies:
            new_bodies = []
            self.ee64_alloc = 0
            self.cold_helpers = []
            for cold_id, fname, body, _old_size, _old_addr in self.cold_function_bodies:
                rewritten = _rewrite(body)
                new_size = _measure_asm_bytes(rewritten)
                if new_size > COLD_SLOT_BYTES:
                    raise Exception(
                        f"cold helper '{fname}' is {new_size}B after "
                        f"cold→cold rewriting; exceeds slot size "
                        f"{COLD_SLOT_BYTES}B.")
                new_addr = self.ee64_alloc
                self.cold_helpers.append((cold_id, new_addr, new_size, fname))
                new_bodies.append((cold_id, fname, rewritten, new_size, new_addr))
                self.ee64_alloc += new_size
            self.cold_function_bodies = new_bodies
            self.cold_slot_size = max(
                (sz for _id, _a, sz, _n in self.cold_helpers), default=0)
            if self.cold_slot_size > 0:
                self.cold_slot_base = 256 - self.cold_slot_size

    def _cold_run_list_body(self):
        """Returns asm lines for `__cold_run_list` cold helper.

        Entry: $a = page-3 address of list (sequence of (id, arg) byte pairs
        terminated by 0xFF in id position).
        Iterates the list, calling __cold_call for each entry. Final
        retval (last call's $a) preserved across `ret`.
        """
        return [
            '__cold_run_list:',
            # Cold-helper arg-recovery prologue (caller's $a saved at
            # page2[0xB5] by __cold_call's setup). 3 B.
            '\tldi $a,0xB5',
            '\tderef2',
            '.crl_lp:',
            '\tpush $a',          # save current ptr
            '\tderefp3',          # A = page3[ptr] = id
            '\tcmpi 0xFF',        # 2 B vs 3 B for ldi+cmp
            '\tjz .crl_dn',
            '\tmov $a,$b',        # B = id (for __cold_call)
            '\tpop $a',           # A = ptr
            '\tinc',              # A = ptr + 1
            '\tpush $a',          # save ptr+1
            '\tderefp3',          # A = page3[ptr+1] = arg
            '\tjal __cold_call',  # callee runs; $a = retval after
            '\tmov $a,$d',        # D = retval (save before pop)
            '\tpop $a',           # A = ptr+1
            '\tinc',              # A = ptr+2 (next id offset)
            '\tj .crl_lp',
            '.crl_dn:',
            '\tpop $a',           # balance stack
            '\tmov $d,$a',        # A = saved retval
            '\tret',
        ]

    def _bundle_cold_calls(self):
        """Option (3): bundle consecutive cold-call sites into a single
        `__cold_run_list` dispatch.

        Detects runs of N≥2 consecutive cold-call call-sites
        (`[ldi $a,LIT;] ldi $b,ID ;!keep; jal __cold_call`) with no
        intervening instructions or labels. Replaces each run with:
            ldi $a, list_addr
            ldi $b, runlist_id ;!keep
            jal __cold_call
        and emits the (id, arg) pairs as page-3 bytes terminated by 0xFF.

        Saves up to 7 B per call beyond the first in each run. Closes
        small kernel overflows (overlay_dashboard 8B) and shrinks
        synthetic main bodies that contain call clusters.

        Skipped in overlay mode (each list byte costs ~5 B of runtime
        kernel preamble for `iderefp3` writes — net loss). Lists go
        in page 3 immediately after the cold-helper metadata table.
        """
        import re as _re_b
        if not getattr(self, 'cold_helpers', None):
            return
        if getattr(self, '_overlay_kernel_size', None) is not None:
            return  # overlay mode

        # Parse a cold-call call-site at lines[i]. Returns
        # (n_consumed, helper_id, arg_value, arg_present) or
        # (0, None, None, False) if not a call site.
        def _parse_block(lines, i):
            j = i
            # Skip blanks/comments. Stop on labels (control-flow targets
            # inside a run break the run).
            while j < len(lines):
                s = lines[j].strip()
                if not s or s.startswith(';'):
                    j += 1
                    continue
                if s.endswith(':'):
                    return (0, None, None, False)
                break
            if j >= len(lines):
                return (0, None, None, False)
            # Optional `ldi $a, LITERAL` (no ;!keep — that's a marker
            # used elsewhere; literal arg-set never carries it).
            arg_value = 0
            arg_present = False
            s = lines[j].strip()
            m_a = _re_b.match(
                r'ldi\s+\$a\s*,\s*(\S+?)\s*(?:;.*)?$', s)
            if m_a and ';!keep' not in lines[j]:
                tok = m_a.group(1)
                try:
                    arg_value = int(tok, 0) & 0xFF
                except ValueError:
                    return (0, None, None, False)
                arg_present = True
                j += 1
                while j < len(lines):
                    s2 = lines[j].strip()
                    if not s2 or s2.startswith(';'):
                        j += 1
                        continue
                    if s2.endswith(':'):
                        return (0, None, None, False)
                    break
                if j >= len(lines):
                    return (0, None, None, False)
            # Required `ldi $b, ID ;!keep`.
            s = lines[j].strip()
            m_b = _re_b.match(
                r'ldi\s+\$b\s*,\s*(\d+)\s*;!keep\s*$', s)
            if not m_b:
                return (0, None, None, False)
            helper_id = int(m_b.group(1)) & 0xFF
            j += 1
            while j < len(lines):
                s2 = lines[j].strip()
                if not s2 or s2.startswith(';'):
                    j += 1
                    continue
                if s2.endswith(':'):
                    return (0, None, None, False)
                break
            if j >= len(lines):
                return (0, None, None, False)
            # Required `jal __cold_call`.
            s = lines[j].strip()
            if s != 'jal __cold_call':
                return (0, None, None, False)
            j += 1
            return (j - i, helper_id, arg_value, arg_present)

        # Two-pass: (a) scan all code, plan bundles; (b) if any, emit
        # __cold_run_list and rewrite.
        def _plan(lines):
            """Returns list of (start_idx, n_consumed, run_entries)."""
            plans = []
            i = 0
            while i < len(lines):
                run = []
                cur = i
                consumed = 0
                while True:
                    n, hid, arg, _ = _parse_block(lines, cur)
                    if n == 0:
                        break
                    run.append((hid, arg))
                    consumed += n
                    cur += n
                if len(run) >= 2:
                    plans.append((i, consumed, run))
                    i += consumed
                elif len(run) == 1:
                    # Single block — skip past it
                    n, _h, _a, _p = _parse_block(lines, i)
                    i += n
                else:
                    i += 1
            return plans

        main_plans = _plan(self.code)
        body_plans = []
        for entry in self.cold_function_bodies:
            cold_id, fname, body, sz, addr = entry
            body_plans.append((cold_id, fname, body, _plan(body), sz, addr))

        total_runs = len(main_plans) + sum(len(p[3]) for p in body_plans)
        if total_runs == 0:
            return

        # Gate: only fire if total savings ≥ 4 B. Each bundle in main
        # saves (per-call bytes × N) − 7 B (the runlist dispatch site).
        # Bundles inside cold bodies save bytes within ee64 (slot
        # impact only); only count main-side savings here for the gate.
        # Bundling adds ~28 B of fixed ee64 cost (__cold_run_list + 3 B
        # metadata + per-bundle list bytes) and grows ee64 size enough
        # to expose AT24C512 page-write flakiness on small programs
        # (Phase 1 with 6 B → 30 B ee64 corrupts a byte mid-stream).
        # 4 B is the minimum savings worth that risk.
        def _bundle_savings(plans, in_main):
            total = 0
            for _start, _n, run in plans:
                # Estimate per-call cost: 4 B (no arg, `ldi $b,ID;!keep`
                # + `jal __cold_call`) or 7 B (with arg, additional
                # `ldi $a,LIT`). We don't know precisely without
                # re-scanning the original lines; conservatively use
                # 4 B per call (lower bound). Bundle replaces with
                # 7 B (`ldi $a,addr; ldi $b,id;!keep; jal __cold_call`).
                per_call = 4
                cost = 7
                total += per_call * len(run) - cost
            return total

        main_savings = _bundle_savings(main_plans, True)
        body_savings = sum(_bundle_savings(p[3], False) for p in body_plans)
        if main_savings + body_savings < 4:
            return

        # Register __cold_run_list as a cold helper. Its id is the next
        # available; ee64 placement happens after re-pack below.
        runlist_body = self._cold_run_list_body()
        runlist_size = _measure_asm_bytes(runlist_body)
        runlist_id = len(self.cold_helpers)

        # If __cold_run_list grows the slot, the dispatcher (__cold_call /
        # __cold_load_helper) — already emitted by _emit_i2c_helpers with
        # the OLD slot_base — has stale `ldi $a/$b, OLD_BASE` references.
        # We patch them post-emit. Capture the old value here.
        _old_slot_base = self.cold_slot_base

        # List bytes go in page 3 immediately after the metadata table.
        # Table size = 3 * (n_helpers_after_runlist_added).
        list_base = COLD_TABLE_BASE + 3 * (runlist_id + 1)
        list_offset = 0
        list_data = []  # (page3_addr, byte_value)

        def _alloc_list(entries):
            nonlocal list_offset
            addr = list_base + list_offset
            for k, (hid, arg) in enumerate(entries):
                list_data.append((addr + 2 * k, hid))
                list_data.append((addr + 2 * k + 1, arg))
            list_data.append((addr + 2 * len(entries), 0xFF))
            list_offset += 2 * len(entries) + 1
            return addr

        def _apply_plans(lines, plans):
            if not plans:
                return list(lines)
            out = []
            i = 0
            plan_idx = 0
            while i < len(lines):
                if (plan_idx < len(plans)
                        and i == plans[plan_idx][0]):
                    _start, n, run = plans[plan_idx]
                    addr = _alloc_list(run)
                    out.append(f'\tldi $a,{addr}')
                    out.append(f'\tldi $b,{runlist_id} ;!keep')
                    out.append('\tjal __cold_call')
                    i += n
                    plan_idx += 1
                else:
                    out.append(lines[i])
                    i += 1
            return out

        # Apply plans to main and to each cold body.
        self.code = _apply_plans(self.code, main_plans)

        new_bodies = []
        for cold_id, fname, body, plans, _osz, _oa in body_plans:
            new_body = _apply_plans(body, plans)
            new_bodies.append((cold_id, fname, new_body))

        # Re-pack ee64: bundling shrinks bodies, addresses shift.
        # Append __cold_run_list at the end (its id was set above so
        # all `ldi $b, runlist_id` references match).
        self.ee64_alloc = 0
        self.cold_helpers = []
        new_function_bodies = []
        for cold_id, fname, new_body in new_bodies:
            new_size = _measure_asm_bytes(new_body)
            if new_size > COLD_SLOT_BYTES:
                raise Exception(
                    f"cold helper '{fname}' is {new_size}B after "
                    f"call-list bundling; exceeds slot size "
                    f"{COLD_SLOT_BYTES}B.")
            new_addr = self.ee64_alloc
            self.cold_helpers.append(
                (cold_id, new_addr, new_size, fname))
            new_function_bodies.append(
                (cold_id, fname, new_body, new_size, new_addr))
            self.ee64_alloc += new_size
        # Append __cold_run_list at runlist_id.
        runlist_addr = self.ee64_alloc
        self.cold_helpers.append(
            (runlist_id, runlist_addr, runlist_size, '_cold_run_list'))
        new_function_bodies.append(
            (runlist_id, '_cold_run_list', runlist_body,
             runlist_size, runlist_addr))
        self.ee64_alloc += runlist_size
        self.cold_function_bodies = new_function_bodies

        # Right-size slot to new max (may have shrunk or grown).
        self.cold_slot_size = max(
            (sz for _id, _a, sz, _n in self.cold_helpers), default=0)
        if self.cold_slot_size > 0:
            self.cold_slot_base = 256 - self.cold_slot_size

        # Patch the dispatcher's hardcoded slot_base if it changed.
        # __cold_call has `ldi $a, OLD; jal_r` (the slot-jump). The
        # bulk-read loop in __cold_load_helper has `ldi $b, OLD` (the
        # destination address for the bulk write). Both must match
        # the new slot_base.
        if self.cold_slot_base != _old_slot_base:
            old_a = f'\tldi $a,{_old_slot_base}'
            old_b = f'\tldi $b,{_old_slot_base}'
            new_a = f'\tldi $a,{self.cold_slot_base}'
            new_b = f'\tldi $b,{self.cold_slot_base}'
            for i in range(len(self.code) - 1):
                cur = self.code[i].rstrip('\n')
                nxt = self.code[i + 1].strip()
                # __cold_call: `ldi $a, OLD` immediately followed by jal_r.
                if cur == old_a and nxt == 'jal_r':
                    self.code[i] = new_a
                # __cold_load_helper: `ldi $b, OLD` followed by a label
                # like `.clh_r2c…:`. The pattern is unique within the
                # dispatcher; main code never emits this combination
                # (the slot is private to the cold tier).
                elif (cur == old_b
                      and nxt.startswith('.clh_r2c')):
                    self.code[i] = new_b

        # Stash list data for emission alongside the metadata table.
        self._cold_list_data = list_data
        self._cold_list_base = list_base

        import sys as _bcl_sys
        n_main = len(main_plans)
        n_body = sum(len(p[3]) for p in body_plans)
        print(f"  Cold-call bundle: {n_main} run(s) in main, "
              f"{n_body} run(s) in cold bodies; emitted "
              f"__cold_run_list ({runlist_size}B) at id={runlist_id}",
              file=_bcl_sys.stderr)

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
            self.emit('\tsll')             # A <<= 1; bit 0 = 0 after shift
            self.emit('\tmov $a,$b')       # B = shifted (bit 0 = 0)
            self.emit_ddrb(0x00)   # SCL HIGH
            self.emit('\texrw 0')          # A = port B
            self.emit('\texrw 0')          # A = port B (bus settle)
            self.emit('\ttst 0x01')        # test SDA
            self.emit(f'\tjz {lbl_rz}')
            # B[0] = 0 (just shifted in), so OR-1 == +1 == incb (1B vs 4B
            # for the mov/ori/mov sequence). Saves 3 B per __i2c_rb.
            self.emit('\tincb')            # set bit 0
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

        if '__eeprom_seq_rd_to_p3' in helpers:
            # Sequential AT24C32 EEPROM read into page 3.
            # One I²C transaction: START + addr-write + RS + addr-read +
            # N×(read byte; ACK-clock; store to page 3) + NACK-clock + STOP.
            # ~10× faster than calling __eeprom_rd N times because we
            # skip N-1 (START + 4-byte address-setup + STOP) sequences.
            #
            # Inputs:  $a = eeprom addr lo
            #          $b = eeprom addr hi
            #          $c = byte count (must be ≥ 1)
            #          $d = page 3 destination offset
            # Output:  page3[dest..dest+count-1] populated from EEPROM
            # Init-only: this helper is called once per program at the
            # OLED font-cache load and never again, so it lives in
            # stage 1 (transient init code).
            lbl_loop = self.label('esrp_lp')
            lbl_last = self.label('esrp_last')
            self.emit('__eeprom_seq_rd_to_p3:')
            # Save inputs (push order: dest, count, eep_lo, eep_hi)
            self.emit('\tpush $d')                 # save dest [.., dest]
            self.emit('\tpush $c')                 # save count [.., dest, count]
            self.emit('\tpush $a')                 # eep_lo [.., dest, count, lo]
            self.emit('\tpush $b')                 # eep_hi [.., dest, count, lo, hi]
            # I²C: START, dev addr write, addr_hi, addr_lo, RS, dev addr read
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0xAE'); self.emit('\tjal __i2c_sb')
            self.emit('\tpop $a'); self.emit('\tjal __i2c_sb')   # eep_hi [.., dest, count, lo]
            self.emit('\tpop $a'); self.emit('\tjal __i2c_sb')   # eep_lo [.., dest, count]
            self.emit('\tjal __i2c_rs')
            self.emit('\tldi $a,0xAF'); self.emit('\tjal __i2c_sb')
            # Pull count + dest back into registers ($c = count, $d = dest)
            self.emit('\tpop $c')                  # count [.., dest]
            self.emit('\tpop $a'); self.emit('\tmov $a,$d')   # dest [.., empty]
            # Read N bytes
            self.emit(f'{lbl_loop}:')
            self.emit('\tpush $c')                 # save count
            self.emit('\tpush $d')                 # save dest
            self.emit('\tjal __i2c_rb')            # byte → $d (clobbers $a/$b/$d)
            self.emit('\tmov $d,$a')               # $a = byte
            self.emit('\tpop $d')                  # restore dest
            self.emit('\tpush $d')                 # keep on stack
            self.emit('\tmov $d,$b')               # iderefp3 dest is $b
            self.emit('\tiderefp3')                # page3[$b] = $a
            self.emit('\tpop $d')                  # dest restored
            self.emit('\tpop $c')                  # count restored
            self.emit('\tincd')                    # dest++
            self.emit('\tdecc')                    # count--
            self.emit(f'\tjz {lbl_last}')
            # Not last: send ACK clock pulse (drive SDA low through SCL pulse)
            self.emit_ddrb(0x03)                   # SCL=0 SDA=0 (drive)
            self.emit_ddrb(0x01)                   # SCL=1 SDA=0 (ACK clock)
            self.emit_ddrb(0x03)                   # SCL=0 SDA=0
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_last}:')
            # NACK clock + STOP
            self.emit_ddrb(0x02)                   # SCL=0 SDA=Hi-Z (release)
            self.emit_ddrb(0x00)                   # SCL=1 SDA=Hi-Z (NACK clock)
            self.emit_ddrb(0x02)                   # SCL=0
            self.emit_ddrb(0x03)                   # STOP setup: SCL=0 SDA=0
            self.emit_ddrb(0x01)                   # SCL=1 SDA=0
            self.emit_ddrb(0x00)                   # SDA=1 (STOP edge)
            self.emit('\tret')

        # __rtc_read_reg: read DS3231 register byte specified in $a.
        # Generic parameterized RTC read with repeated START. Caller passes
        # the register address in $a; returns the byte read in $a.
        # Saves bytes vs inlining for programs that read multiple registers
        # (e.g. overlay_dashboard.c reads 4 different registers in main()).
        # Same I2C sequence as rtc_read_seconds/rtc_read_temp but with the
        # register byte parameterized — shared body costs ~32 B once instead
        # of ~31 B per inline call site.
        if '__rtc_read_reg' in helpers:
            self.emit('__rtc_read_reg:')
            self.emit('\tmov $a,$d')         # save reg byte to D
            # START + 0xD0 (DS3231 write address)
            self.emit('\texrw 2')
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit('\tldi $a,0xD0')
            self.emit('\tjal __i2c_sb')
            # Send register byte (recover from D)
            self.emit('\tmov $d,$a')
            self.emit('\tjal __i2c_sb')
            # STOP + START (no RS) + 0xD1 (read address) + read byte + NACK + STOP
            self.emit('\tjal __i2c_sp')
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0xD1')
            self.emit('\tjal __i2c_sb')
            self.emit('\tjal __i2c_rb')
            self.emit('\tmov $d,$a')         # A = read byte (returned)
            # NACK + STOP (merged via the trailing 6 ddrb writes)
            self.emit_ddrb(0x02)
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
            # `cmpi X` (2 B) replaces `ldi $b,X; cmp $b` (3 B). $b is
            # not read between sentinel checks here, so the load was
            # redundant. Saves 1 B per check × 5 checks = 5 B body.
            self.emit('\tcmpi 0xFF')
            self.emit(f'\tjz {lbl_done}')     # END
            self.emit('\tcmpi 0xFE')
            self.emit(f'\tjnz {lbl_ns}')
            # START
            self.emit('\texrw 2')
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_ns}:')
            self.emit('\tcmpi 0xFD')
            self.emit(f'\tjnz {lbl_nr}')
            # STOP
            self.emit_ddrb(0x03)
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x00)
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_nr}:')
            self.emit('\tcmpi 0xFB')
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
            self.emit('\tsll')                # bit 0 = 0 after shift
            self.emit('\tmov $a,$b')
            self.emit_ddrb(0x00)    # SCL HIGH
            self.emit('\tnop')
            self.emit('\texrw 0')           # read port
            self.emit('\ttst 0x01')
            self.emit(f'\tjz {lbl_rz}')
            # B[0] = 0 (just shifted in), so OR-1 == +1 == incb.
            self.emit('\tincb')
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
            self.emit('\tcmpi 0xFC')
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

        if '__oled_init' in helpers:
            # Walker mirroring __lcd_init's shape, with non-conflicting
            # sentinels (SSD1327 uses 0xFD/0xFE/0xFC as real commands).
            # Reads init data from the data section byte-by-byte and
            # dispatches START/STOP/DELAY/byte to the right I²C helper.
            # Returns when OEND is seen.
            data_base, init_bytes = self._oled_init_data
            OSTART, OSTOP, OEND, ODELAY = 0xF1, 0xF2, 0xF0, 0xF3
            lp = self.label('oi_lp')
            ns = self.label('oi_ns')
            nd = self.label('oi_nd')
            sd = self.label('oi_sd')
            av = self.label('oi_av')
            dn = self.label('oi_dn')

            self.emit('__oled_init:')
            self.emit('\tout_imm 0x10')        # DEBUG: entered init walker
            self.emit(f'\tldi $d,{data_base}')
            self.emit(f'{lp}:')
            self.emit('\tmov $d,$a')
            self.emit('\tderef')
            self.emit(f'\tcmp {OEND}')
            self.emit(f'\tjz {dn}')
            self.emit('\tpush $d')
            self.emit(f'\tcmp {OSTART}')
            self.emit(f'\tjnz {ns}')
            self.emit('\tjal __i2c_st_only')
            self.emit(f'\tj {av}')
            self.emit(f'{ns}:')
            self.emit(f'\tcmp {OSTOP}')
            self.emit(f'\tjnz {nd}')
            self.emit('\tjal __i2c_sp')
            self.emit(f'\tj {av}')
            self.emit(f'{nd}:')
            self.emit(f'\tcmp {ODELAY}')
            self.emit(f'\tjnz {sd}')
            # ~5 ms delay (uncalibrated). Outer counter via decc
            # directly instead of round-tripping through $a (was 3 ops:
            # mov $c,$a; dec; mov $a,$c; jnz).
            do = self.label('oi_do')
            di = self.label('oi_di')
            self.emit('\tldi $c,5')
            self.emit(f'{do}:')
            self.emit('\tclr $a')                   # inner counter = 0 (-> 256 iters via dec/jnz)
            self.emit(f'{di}:')
            self.emit('\tdec')
            self.emit(f'\tjnz {di}')
            self.emit('\tdecc')                     # outer count-- (was mov/dec/mov)
            self.emit(f'\tjnz {do}')
            self.emit(f'\tj {av}')
            self.emit(f'{sd}:')
            self.emit('\tjal __i2c_sb')
            self.emit(f'{av}:')
            self.emit('\tpop $d')
            self.emit('\tincd')                     # advance walker ptr (was mov/inc/mov)
            self.emit(f'\tj {lp}')
            self.emit(f'{dn}:')
            self.emit('\tout_imm 0x20')        # DEBUG: init walker done
            self.emit('\tret')

        # Per-K specialized fill helpers. Each emits the
        # 8192-byte fill transaction with the unrolled byte-send
        # body for that specific K. Multiple distinct K values in
        # one program produce multiple helpers (one per K) but each
        # is reused if `oled_fill(K)` is called multiple times with
        # the same K. ~80B per helper.
        for k in sorted(getattr(self, '_oled_fill_ks', set())):
            helper_name = f'__oled_fill_{k:02X}'
            if helper_name not in helpers:
                continue
            # Body shape was the unrolled-byte-send fast path (~80 B per
            # helper, ~127 cyc/byte) which optimised for animation-style
            # frequent fills. In practice oled_fill / oled_clear are
            # almost always init-time one-shots where boot delay matters
            # less than code-page footprint. Loop via the kernel-resident
            # __i2c_sb instead — drops each helper from ~80 B to ~25 B
            # (~55 B saved) at the cost of fill time roughly tripling
            # (~127 cyc/byte → ~440 cyc/byte = ~22 s for the full 8192-
            # byte clear at 165 kHz). The byte saving is what unblocks
            # programs that need oled_clear() but currently bust the
            # stage-1/kernel budget; if a program ever needs the fast
            # path back it can be re-introduced under a separate builtin.
            self.emit(f'{helper_name}:')
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x7A')         # OLED write addr
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x40')         # control: data stream
            self.emit('\tjal __i2c_sb')
            # 8192-byte fill: outer × inner = 32 × 256 via __i2c_sb.
            outer_lbl = self.label('ofl_o')
            inner_lbl = self.label('ofl_i')
            self.emit('\tldi $b,32')           # outer = 32 (32 × 256 = 8192)
            self.emit(f'{outer_lbl}:')
            self.emit('\tpush $b')             # save outer counter
            self.emit('\tldi $c,0')            # inner = 256 (decc wraps 0→255)
            self.emit(f'{inner_lbl}:')
            self.emit(f'\tldi $a,{k}')         # constant fill byte
            self.emit('\tjal __i2c_sb')
            self.emit('\tdecc')
            self.emit(f'\tjnz {inner_lbl}')
            self.emit('\tpop $b')              # restore outer
            self.emit('\tdecb')
            self.emit(f'\tjnz {outer_lbl}')
            self.emit('\tjal __i2c_sp')
            self.emit('\tret')

        if '__oled_putc_xfer' in helpers:
            # Streams a 27-byte putc table across two I²C transactions.
            # Caller passes table base in $d. Table layout:
            #   bytes 0..5: 0x15, col_s, col_e, 0x75, row_s, row_e
            #              (6-byte window-set payload, sent after
            #              addr+CMD_STREAM)
            #   bytes 6..26: 21 4bpp pixel bytes
            #              (sent after addr+DATA_STREAM)
            # `$c` holds the per-section byte count, `$d` walks the
            # table. `__i2c_sb` clobbers $a/$b/$d — we save $d and
            # the loop counter on the stack across each send.
            wlp = self.label('puxw')
            dlp = self.label('puxd')
            self.emit('__oled_putc_xfer:')
            # The `out_imm` markers below were the diff between
            # "MK1 stair-steps diagonally" and "MK1 renders side-by-
            # side correctly" on hardware. Adding them inadvertently
            # fixed the rendering bug; removing them re-introduced
            # it (visual confirmation pending on next hardware run).
            # The byte→STOP hold-time hypothesis was checked against
            # the SSD1327 datasheet (Table 13-6) and ruled out: tHD
            # = 300 ns, tSSTOP = 0.6 µs, tIDLE = 1.3 µs are all
            # easily met (we have ~50 µs gaps everywhere). True
            # cause is still unknown; markers are kept as a
            # known-good workaround until root-caused. See WORKLOG
            # 2026-04-27 for the open TODO.
            self.emit('\tout_imm 0x40')             # entry pad
            self.emit('\tpush $d')                  # save table base
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x7A')              # OLED write addr
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x00')              # CMD_STREAM
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $d')                   # restore table base
            self.emit('\tldi $c,6')                 # 6 window bytes
            self.emit(f'{wlp}:')
            self.emit('\tpush $c')
            self.emit('\tpush $d')
            self.emit('\tmov $d,$a')                # $a = $d (table ptr)
            self.emit('\tderef')                    # $a = data[$a]
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $d')
            self.emit('\tincd')                     # advance ptr
            self.emit('\tpop $c')
            self.emit('\tdecc')
            self.emit(f'\tjnz {wlp}')
            self.emit('\tout_imm 0x50')             # window→data pad (see comment above)
            self.emit('\tpush $d')                  # save ptr (now @ data section)
            self.emit('\tjal __i2c_sp')
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x7A')
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x40')              # DATA_STREAM
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $d')                   # ptr to first pixel byte
            self.emit('\tldi $c,21')                # 21 pixel bytes
            self.emit(f'{dlp}:')
            self.emit('\tpush $c')
            self.emit('\tpush $d')
            self.emit('\tmov $d,$a')
            self.emit('\tderef')
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $d')
            self.emit('\tincd')
            self.emit('\tpop $c')
            self.emit('\tdecc')
            self.emit(f'\tjnz {dlp}')
            self.emit('\tjal __i2c_sp')
            self.emit('\tout_imm 0x60')             # exit pad (see comment above)
            self.emit('\tret')

        if '__oled_set_window' in helpers:
            # Sets the SSD1327 GDDRAM window via one I²C transaction.
            # Inputs:  $a = col_start (0..63 in 2-pixel col-units)
            #          $b = col_end (inclusive)
            #          $c = row_start (0..127 px)
            #          $d = row_end (inclusive)
            # ~35 B; replaces ~55 B of inline window-set per caller,
            # saves ~20 B per call site (used by oled_text_at and any
            # other future oled_print_buf / oled_clear_line builtins).
            self.emit('__oled_set_window:')
            # Save args that won't survive __i2c_sb. $c is preserved.
            self.emit('\tpush $d')                 # row_e
            self.emit('\tpush $b')                 # col_e
            self.emit('\tpush $a')                 # col_s
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x7A'); self.emit('\tjal __i2c_sb')
            self.emit('\tclr $a'); self.emit('\tjal __i2c_sb')        # CMD_STREAM (= 0x00)
            self.emit('\tldi $a,0x15'); self.emit('\tjal __i2c_sb')   # col cmd
            self.emit('\tpop $a'); self.emit('\tjal __i2c_sb')        # col_s
            self.emit('\tpop $a'); self.emit('\tjal __i2c_sb')        # col_e
            self.emit('\tldi $a,0x75'); self.emit('\tjal __i2c_sb')   # row cmd
            self.emit('\tmov $c,$a'); self.emit('\tjal __i2c_sb')     # row_s
            self.emit('\tpop $a'); self.emit('\tjal __i2c_sb')        # row_e
            self.emit('\tout_imm 0x70')             # window→stop pad (load-bearing per Phase 3)
            self.emit('\tj __i2c_sp')                # tail-call (no ret needed)

        if '__oled_stream_chars' in helpers:
            # Phase 4b' streaming text helper. Pumps expanded glyph
            # data for `count` characters through an already-open OLED
            # window in ONE I²C data-stream transaction. Caller is
            # responsible for the prior window-set transaction
            # (oled_text_at builtin emits it inline before the call).
            #
            #   $d in: page-1 address of an N-byte buffer of font
            #          indices (0..N-1, one per glyph to render).
            #   $c in: count (number of chars).
            #
            # Per char: re-reads the same 5 glyph row bytes from the
            # page-3 font cache (3× per row, but page-3 reads via
            # `derefp3` are 1 instruction so it's cheaper than the
            # push/pop dance for stack reload). Each row's 5 source
            # bits expand to 3 4bpp bytes via the page-1 LUT. After
            # the 5 glyph rows, 6 zero bytes flush the descender +
            # line-gap rows.
            #
            # Marker pads at window→data and data→stop transitions
            # mirror the unexplained Phase 3 stair-stepping fix —
            # don't remove without re-testing.
            font_p3 = self._oled_font_p3_base if hasattr(self, '_oled_font_p3_base') else 0
            lut0_off = self._oled_lut_base if hasattr(self, '_oled_lut_base') else 0
            lut1_off = lut0_off + 32   # second 32-entry LUT (byte 1)
            lut2_off = lut0_off + 64   # third 32-entry LUT (byte 2)
            cl = self.label('osc_cl')        # char loop
            rl = self.label('osc_rl')        # row loop

            self.emit('__oled_stream_chars:')
            self.emit('\tpush $d')                  # save buf [.., buf]
            self.emit('\tpush $c')                  # save count [.., buf, count]
            self.emit('\tjal __i2c_st_only')
            self.emit('\tldi $a,0x7A'); self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x40'); self.emit('\tjal __i2c_sb')   # DATA_STREAM
            self.emit('\tpop $c')                   # count [.., buf]
            self.emit('\tpop $d')                   # buf ptr [.., empty]

            self.emit(f'{cl}:')                     # char loop
            self.emit('\tpush $c')                  # save outer count
            self.emit('\tpush $d')                  # save buf ptr

            # Read idx from page-1 buffer
            self.emit('\tmov $d,$a')
            self.emit('\tderef')                    # $a = idx

            # Compute page-3 font addr = font_p3_base + idx*5
            self.emit('\tmov $a,$b')               # $b = idx
            self.emit('\tsll')                      # 2x
            self.emit('\tsll')                      # 4x
            self.emit('\tadd $b,$a')               # 5x
            if font_p3:
                self.emit(f'\taddi {font_p3},$a')
            self.emit('\tmov $a,$c')               # $c = font addr (preserved)

            # 5 glyph rows × 3 expanded bytes each
            self.emit('\tldi $a,5')
            self.emit('\tpush $a')                  # row counter
            self.emit(f'{rl}:')
            # Each byte uses its own 32-entry LUT keyed by the full
            # 5-bit row byte → no runtime shift/mask. Re-reads the
            # row byte from page 3 (`derefp3` is 1 instruction) for
            # each of the 3 bytes — cheaper than push/pop reload.
            # Byte 0
            self.emit('\tmov $c,$a')
            self.emit('\tderefp3')                  # $a = row byte (5 bits)
            self.emit(f'\taddi {lut0_off},$a')
            self.emit('\tderef')                    # LUT0[row]
            self.emit('\tjal __i2c_sb')
            # Byte 1
            self.emit('\tmov $c,$a')
            self.emit('\tderefp3')
            self.emit(f'\taddi {lut1_off},$a')
            self.emit('\tderef')                    # LUT1[row]
            self.emit('\tjal __i2c_sb')
            # Byte 2
            self.emit('\tmov $c,$a')
            self.emit('\tderefp3')
            self.emit(f'\taddi {lut2_off},$a')
            self.emit('\tderef')                    # LUT2[row]
            self.emit('\tjal __i2c_sb')
            self.emit('\tincc')                     # advance to next row's font byte
            # Row counter — dec sets ZF, no separate cmpi 0 needed
            self.emit('\tpop $a'); self.emit('\tdec'); self.emit('\tpush $a')
            self.emit(f'\tjnz {rl}')
            self.emit('\tpop $a')                   # discard row counter

            # Char loop tail (no descender/line-gap rows — cells are
            # 6 wide × 5 tall, giving a 21 × 25 character grid. Tighter
            # vertical layout for menus + frees ~30 B of helper code
            # vs the previous 6×7 cell which appended 6 zero bytes
            # per char.)
            self.emit('\tpop $d')                   # restore buf ptr
            self.emit('\tincd')                     # buf++
            self.emit('\tpop $c')                   # restore outer count
            self.emit('\tdecc')
            self.emit(f'\tjnz {cl}')
            self.emit('\tout_imm 0x55')             # data→stop pad (load-bearing per Phase 3)
            self.emit('\tj __i2c_sp')                # tail-call (no ret needed)

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
            # `incc` clobbers $a as a side effect (microcode: BO|AI ... AO|CI
            # — the ALU runs through A, leaving A=new C). Without saving A
            # around the increment, the next iteration's `cmpi 10` compares
            # the tens count (e.g. 1) instead of the running ones value, and
            # the loop terminates after one iteration regardless of the
            # input. For input 22 this produced "11°C" on the LCD instead
            # of "22°C". `__print_u8_dec` already wraps incc with push/pop
            # for the same reason; this matches that pattern.
            self.emit('\tpush $a')
            self.emit('\tincc')              # tens++ (clobbers A)
            self.emit('\tpop $a')            # restore ones
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
            # B++ in 1 B (was 3 B: mov $b,$a; inc; mov $a,$b).
            self.emit('\tincb')
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
            # Apply ORA shadow mask so other PA pins claimed by other
            # devices keep their driven bits across the tone toggle.
            # PA1 (bit 1) is tone's owned output. For programs without
            # any other ORA claim, shadow=0 → byte-identical to the
            # pre-shadow raw `ldi 0x02 / clr $a` sequence (which is
            # what the duty-cycle pad below was calibrated against).
            # Precomputed values keep the opcode/cycle counts stable;
            # we don't switch to `ora_imm` because that would change
            # the pad math (ora_imm is fewer cycles than ldi + exw).
            ora_hi = self._port_shadow_mask('ORA') | 0x02
            ora_lo = self._port_shadow_mask('ORA') & ~0x02 & 0xFF
            self.emit('__tone:')
            self.emit(f'{lbl_loop}:')
            self.emit(f'\tldi $a,0x{ora_hi:02X}')
            self.emit('\texw 0 1')         # PA1 HIGH (shadow preserved)
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
            if ora_lo == 0:
                self.emit('\tclr $a')          # 1 B (was the unshadowed path)
            else:
                self.emit(f'\tldi $a,0x{ora_lo:02X}')  # 2 B; LOW path 1 B longer
            self.emit('\texw 0 1')         # PA1 LOW (shadow preserved)
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

        # __cold_call: kernel-resident dispatcher for the AT24C512 cold tier.
        # Entry: $a = helper_id (0..N-1).
        # Looks up (ee64_addr_hi, ee64_addr_lo, byte_count) from a 3-byte-
        # per-entry metadata table at page3[COLD_TABLE_BASE], bulk-reads
        # the helper bytes from AT24C512 into code[COLD_SLOT_BASE] via the
        # chip-agnostic __eeprom_r2c_loop, then jal_r's into the loaded
        # code. The cold helper does its work and `ret`s back here, which
        # then ret's to the original caller.
        #
        # Latency: ~5-10 ms per cold call at 100 kHz I²C for a ~50 B
        # helper (chip read time + slot copy). Slot is overwritten each
        # call, so back-to-back cold calls to different helpers each pay
        # the load cost; reuse of the same id within tight loops would be
        # wasteful — partitioner should avoid that pattern.
        if '__cold_call' in helpers:
            # __cold_call: enter with A = new helper id. Saves the
            # currently-loaded helper id onto the stack, sets the
            # current id to new, calls __cold_load_helper to fetch the
            # helper's bytes into the slot, jal_r's into the slot.
            # After the cold helper returns, pops the saved caller id;
            # if non-sentinel, reloads the caller's bytes into the slot
            # so the caller's `ret` (or next instruction) lands on its
            # own bytes — the cold→cold chaining contract.
            #
            # Sentinel 0xFF in `cold_current_id` means "no cold helper
            # loaded" (caller is main or kernel-resident code).
            # `_main`'s preamble initializes the slot to 0xFF.
            self.emit('__cold_call:')
            # Entry contract: caller did `ldi $b, <id>; jal __cold_call`.
            # So $b = id and $a = caller's parameter for the cold helper.
            # Stack: [..., RA] with RA on top.
            #
            # Save caller's $a (helper's parameter) to kstate
            # cold_call_arg. Cold helper bodies that read $a get a
            # synthesized prologue (`ldi $a,0xB5; deref2`) that
            # reloads $a; helpers that ignore $a pay 3 wasted bytes.
            #
            # kstate_store uses $b as the address register, which would
            # destroy our id. Stash $b on the stack while we run the
            # store, then pop it back.
            self.emit('\tpush_b')                          # save id (in $b)
            self.emit_kstate_store('cold_call_arg')        # page2[0xB5] = $a
            self.emit('\tpop_b')                           # B = id
            self.emit('\tmov $b,$d')                       # save id in D
            self.emit_kstate_load('cold_current_id')       # A = caller's id
            self.emit('\tpush $a')                         # save caller id on stack
            self.emit('\tmov $d,$a')                       # A = new_id
            self.emit_kstate_store('cold_current_id')      # current = new_id
            self.emit('\tjal __cold_load_helper')          # load new helper into slot
            self.emit(f'\tldi $a,{self.cold_slot_base}')
            self.emit('\tjal_r')                           # run cold helper
            # `jal_r` pushes PC+1 (= addr of this next instr) and `ret`
            # does PC = pop()+1, so the slot's `ret` lands on (addr+1)
            # — skipping the first byte of whatever follows. Insert a
            # 1-byte NOP (`move $a,$a`, opcode 0x00) so the off-by-one
            # falls on the NOP, leaving the real epilogue intact.
            # (Symmetric to how `jal imm` works: jal pushes addr_of_imm,
            # ret does pop+1, lands on next instr — but jal_r has no
            # imm byte to absorb the +1.)
            self.emit('\tmov $a,$a ;!keep')                # jal_r off-by-one filler
            # Cold helper returned with its return value in $a. Save
            # it (reusing the cold_call_arg slot — the helper already
            # consumed its input arg at the prologue, so the slot is
            # free) so the post-slot cleanup can clobber $a freely.
            self.emit_kstate_store('cold_call_arg')        # cold_call_arg = retval
            # Restore caller's id and reload its bytes (unless caller
            # was the sentinel = main/kernel).
            self.emit('\tpop $a')                          # A = saved caller id
            self.emit_kstate_store('cold_current_id')      # current = caller id
            self.emit('\tcmpi 0xFF')
            cc_done = self.label('cc_done')
            self.emit(f'\tjz {cc_done}')
            # Caller is itself a cold helper — reload its bytes so its
            # next instruction lands on the right slot byte.
            self.emit('\tjal __cold_load_helper')
            self.emit(f'{cc_done}:')
            # Restore helper's return value to $a before returning to
            # caller (the cleanup above clobbered $a).
            self.emit('\tldi $a,'
                      f'{KERNEL_STATE["cold_call_arg"].offset}')
            self.emit('\tderef2')
            self.emit('\tret')

        # __cold_load_helper: reads `cold_current_id` from kstate, looks
        # up its (addr_hi, addr_lo, len) in the page-3 metadata table,
        # bulk-reads `len` bytes from AT24C512 into code[COLD_SLOT_BASE].
        # Factored out of __cold_call so we can call it twice from the
        # dispatcher: once for the new helper, once for the caller-reload
        # path on cold→cold returns.
        if '__cold_load_helper' in helpers:
            self.emit('__cold_load_helper:')
            # Caller convention: $a = helper id. Both call sites in
            # `__cold_call` (initial-load and chain-reload) leave the
            # id in $a after their respective `kstate_store
            # cold_current_id` (which only touches $b), so we don't
            # need to re-read the id from kstate here. Saves 3 B
            # vs the prior `ldi $a,offset; deref2` reload.
            self.emit('\tmov $a,$b')                       # b = id
            self.emit('\tsll')                             # a = 2*id
            self.emit('\tadd $b,$a')                       # a = 3*id
            # Walk down from len → lo → hi using `decb` to step the
            # base pointer instead of recomputing base + offset for
            # each derefp3. Saves 2 B over the prior `addi 16; mov;
            # addi 2; derefp3; mov; addi 1; derefp3; mov; derefp3`
            # form by reusing $b as a base+offset cursor.
            self.emit(f'\taddi {COLD_TABLE_BASE + 2},$a')  # a = ptr+2 (len addr)
            self.emit('\tmov $a,$b')                       # b = ptr+2
            self.emit('\tderefp3')                         # a = len
            self.emit('\tpush $a')
            self.emit('\tdecb')                            # b = ptr+1
            self.emit('\tmov $b,$a')
            self.emit('\tderefp3')                         # a = addr_lo
            self.emit('\tpush $a')
            self.emit('\tdecb')                            # b = ptr
            self.emit('\tmov $b,$a')
            self.emit('\tderefp3')                         # a = addr_hi
            self.emit('\tpush $a')
            # I²C: START + 0xA0 + addr_hi + addr_lo + REP-START + 0xA1
            self.emit('\texrw 2')
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit('\tldi $a,0xA0')
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $a')                          # addr_hi
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $a')                          # addr_lo
            self.emit('\tjal __i2c_sb')
            self.emit_ddrb(0x00)                           # REP-START
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit('\tldi $a,0xA1')
            self.emit('\tjal __i2c_sb')
            # Bulk-read inlined: len on top of stack. Park count in
            # $c — neither `__i2c_sb` nor `__i2c_rb` touches $c, and
            # `decc` is a single-byte op that updates the count and
            # sets ZF in one go. Saves ~10 B vs the prior kstate-
            # backed counter (which needed push_b/load/dec/store/
            # pop_b around every iteration).
            self.emit('\tpop $a')                          # A = count
            self.emit('\tmov $a,$c')                       # C = count
            self.emit(f'\tldi $b,{self.cold_slot_base}')   # B = dest
            clh_loop = self.label('clh_r2c')
            clh_last = self.label('clh_r2cl')
            self.emit(f'{clh_loop}:')
            self.emit('\tpush_b')
            self.emit('\tjal __i2c_rb')
            self.emit('\tpop_b')
            self.emit('\tmov $d,$a')
            self.emit('\tistc_inc')
            self.emit('\tdecc')                            # C-- (sets ZF)
            self.emit(f'\tjz {clh_last}')
            self.emit_ddrb(0x03)
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x03)
            self.emit_ddrb(0x02)
            self.emit(f'\tj {clh_loop}')
            self.emit(f'{clh_last}:')
            self.emit_ddrb(0x00)
            self.emit_ddrb(0x02)
            self.emit_ddrb(0x03)
            self.emit_ddrb(0x01)
            self.emit_ddrb(0x00)
            self.emit('\tret')

        # __keypad_scan: 4x4 matrix scanner for the keypad on PA4-7 (rows)
        # / PB4-7 (cols). Returns 0-15 (row*4 + col) or 0xFF if no key.
        # Drives one row LOW per slot, reads ORB & 0xF0; if any col bit
        # is LOW, decodes which one and combines with the row index.
        # Restores ORA to the all-rows-HIGH idle state before returning.
        # Resident (called per-poll); ~95B body, acceptable kernel cost.
        if '__keypad_scan' in helpers:
            # Other-device contributions to the ORA shadow — bits the
            # scan must preserve when driving its own row bits. Excludes
            # keypad's own 0xF0 claim (which would otherwise force every
            # row HIGH and make the whole scan a no-op).
            excl = self._port_shadow_mask_excluding('ORA', 'keypad')
            row_masks = [
                excl | 0xE0,   # row 0: PA4 LOW, PA5-7 HIGH
                excl | 0xD0,   # row 1: PA5 LOW
                excl | 0xB0,   # row 2: PA6 LOW
                excl | 0x70,   # row 3: PA7 LOW
            ]
            idle_ora = excl | 0xF0   # all rows HIGH (= full shadow)

            ks_p = [self.label('ks_p') for _ in range(4)]
            ks_common = self.label('ks_common')
            ks_decode = self.label('ks_decode')

            self.emit('__keypad_scan:')
            for row in range(4):
                self.emit(f'\tldi $a,0x{row_masks[row]:02X}')
                self.emit('\texw 0 1')           # ORA = row mask
                self.emit('\tnop')               # settle (column lines)
                self.emit('\texrw 0')            # A = ORB
                self.emit('\tandi 0xF0,$a')      # mask col bits
                self.emit('\tcmpi 0xF0')         # all 1s = no key in row
                self.emit(f'\tjnz {ks_p[row]}')

            # No-key fall-through and pressed paths converge on a
            # single ORA-idle-restore + ret block, saving the
            # duplicated `ldi IDLE / exw 0 1` between them.
            ks_finish = self.label('ks_finish')

            self.emit('\tldi $a,0xFF')           # no-key sentinel
            self.emit(f'\tj {ks_finish}')

            # Pressed handlers: load row*4 into $b, jump to common.
            # Last one falls through to common, saving 2B.
            for row in range(3):
                self.emit(f'{ks_p[row]}:')
                self.emit(f'\tldi $b,0x{(row * 4):02X}')
                self.emit(f'\tj {ks_common}')
            self.emit(f'{ks_p[3]}:')
            self.emit('\tldi $b,0x0C')           # row 3 → b = 12

            # Common: A has the col pattern (e.g. 0xE0 if COL0 is the
            # pressed col in the active row). Decode to col 0-3, add
            # row*4 in $b for the 0-15 key index, fall through to
            # finish (which restores ORA to idle and returns A).
            self.emit(f'{ks_common}:')
            self.emit(f'\tjal {ks_decode}')      # A = col 0-3
            self.emit('\tadd $b,$a')             # A = col + row*4

            # Shared idle-restore + return. Reached from the no-key
            # fall-through (A = 0xFF) and from the pressed common
            # path (A = key index 0-15). The push/pop preserves A
            # across the ORA write.
            self.emit(f'{ks_finish}:')
            self.emit('\tpush $a')
            self.emit(f'\tldi $a,0x{idle_ora:02X}')
            self.emit('\texw 0 1')
            self.emit('\tpop $a')
            self.emit('\tret')

            # Decoder: A in {0xE0, 0xD0, 0xB0, 0x70} → col 0-3.
            # Shift right 4 to align high nibble in bits 0-3, then
            # count trailing 1-bits via tst+slr loop, returning the
            # count in A. Counter lives in $d, NOT $b — the caller
            # (ks_common) holds the row*4 offset in $b and adds it
            # after we return, so we MUST preserve $b.
            ks_dec_loop = self.label('ks_dl')
            ks_dec_done = self.label('ks_dd')
            self.emit(f'{ks_decode}:')
            self.emit('\tslr')
            self.emit('\tslr')
            self.emit('\tslr')
            self.emit('\tslr')                   # A = high nibble
            self.emit('\tldi $d,0x00')           # col counter ($d, not $b)
            self.emit(f'{ks_dec_loop}:')
            self.emit('\ttst 0x01')
            self.emit(f'\tjz {ks_dec_done}')
            self.emit('\tslr')
            self.emit('\tpush $a')
            self.emit('\tincd')                  # incd clobbers $a
            self.emit('\tpop $a')
            self.emit(f'\tj {ks_dec_loop}')
            self.emit(f'{ks_dec_done}:')
            self.emit('\tmov $d,$a')             # A = col index
            self.emit('\tret')

        # __keypad_wait: block until a stable key press is observed,
        # then block until release, then return the 0-15 key index.
        #
        # Algorithm:
        #   loop:
        #     k1 = scan(); if k1 == 0xFF: continue   ; spin until something
        #     delay 5 ms                              ; settle
        #     k2 = scan()
        #     if k2 != k1: continue                  ; bounce / glitch — restart
        #     while scan() != 0xFF: spin             ; wait for release
        #     return k1
        #
        # Debounce is 5 ms based on the typical membrane-keypad bounce
        # window of 1-3 ms. ~26 B body. Depends on __keypad_scan and
        # __delay_Nms (the latter requires __delay_cal calibration to
        # have populated kstate calib_blocks).
        if '__keypad_wait' in helpers:
            kw_poll = self.label('kw_poll')
            kw_release = self.label('kw_release')
            self.emit('__keypad_wait:')
            self.emit(f'{kw_poll}:')
            self.emit('\tjal __keypad_scan')
            self.emit('\tcmpi 0xFF')
            self.emit(f'\tjz {kw_poll}')      # no key, keep polling
            self.emit('\tpush $a')             # save first-scan key
            self.emit('\tldi $b,5')            # 5 ms debounce
            self.emit('\tjal __delay_Nms')
            self.emit('\tjal __keypad_scan')   # second sample
            self.emit('\tpop_b')               # B = first-scan key
            self.emit('\tcmp $b')              # ZF = (A == B)
            self.emit(f'\tjnz {kw_poll}')      # values differ → bounce, restart
            self.emit('\tpush $a')             # save stable key index
            self.emit(f'{kw_release}:')
            self.emit('\tjal __keypad_scan')
            self.emit('\tcmpi 0xFF')
            self.emit(f'\tjnz {kw_release}')   # not released yet
            self.emit('\tpop $a')              # A = stable key index
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
        # __eeprom_seq_rd_to_p3 is always init-only — its sole caller is
        # the OLED font-cache load which runs once in oled_init.
        INIT_ONLY_NAMES.add('__eeprom_seq_rd_to_p3')
        # __oled_init walker runs ONCE per program (gated by the
        # _oled_init_emitted compile-time flag → only one `jal __oled_
        # init` per program). Stage 1 placement frees ~78 B of stage-2
        # kernel space.
        INIT_ONLY_NAMES.add('__oled_init')
        # __oled_fill_K helpers — call-site-aware classification.
        # Init-only (frees ~28 B from runtime kernel → bigger overlay
        # slot) is correct ONLY if every call to that K is from main's
        # init prefix (not inside a loop, not in a user function).
        # The oled_fill builtin tags runtime-context call sites in
        # `_oled_fill_runtime_use`; any K NOT in that set is safely
        # init-only. For programs that legitimately fill the screen
        # at runtime (animation loops, etc.), the helper stays
        # kernel-resident automatically — no silent breakage.
        _ofu = getattr(self, '_oled_fill_runtime_use', set())
        for _fk in sorted(getattr(self, '_oled_fill_ks', set())):
            if _fk not in _ofu:
                INIT_ONLY_NAMES.add(f'__oled_fill_{_fk:02X}')
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
            # __oled_fill_K helpers — call-site-aware (see comment at
            # the INIT_ONLY_NAMES site above). Only K values whose every
            # call is in main's init prefix get init-only treatment;
            # runtime-context callers keep the helper kernel-resident.
            _ofu = getattr(self, '_oled_fill_runtime_use', set())
            for _fk in sorted(getattr(self, '_oled_fill_ks', set())):
                if _fk not in _ofu:
                    init_only_names.add(f'__oled_fill_{_fk:02X}')

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
            # Helper→helper splits stay disabled (would multiply reload
            # thunks across helper call chains and produce bad behavior
            # for short hot helpers like the I²C primitives).
            if caller.startswith('__'):
                return False
            cs = ov_by_name.get(caller, (0, 0, 0))[2]
            ds = ov_by_name.get(callee, (0, 0, 0))[2]
            # User-fn → helper splits are gated more tightly: the
            # helper has to be substantial (≥ 32 B) AND the merged
            # bundle has to be clearly oversized (cs + ds ≥ 80 B). The
            # cycle cost (~1000 cycles per call through a reload thunk)
            # would be catastrophic on hot helpers like __i2c_sb that
            # appear in tight loops; the size gate prevents that. Larger
            # helpers like __keypad_scan or __oled_stream_chars
            # (50 B+) tend to be called from cold paths where the
            # cycle cost is invisible.
            if callee.startswith('__'):
                if ds < 32:
                    return False
                if cs + ds < 80:
                    return False
                return True
            # The thunk costs ~11 B resident. We split if the savings
            # (avoiding a merge that would balloon the bundle) clearly
            # exceed that cost.
            #
            # The `_main_p`-only restriction (pre-2026-04-27) prevented
            # all other user-fn → user-fn cross-overlay splits, forcing
            # the SCC merger to pile every call chain into one overlay.
            # That works for programs where the merged bundle fits the
            # slot, but breaks (e.g. keypad_oled.c) when helpers bring
            # the bundle over. Relaxed thresholds:
            #
            #   - callee ≥ 16 B: enough that the 11 B thunk + ~5 B per
            #     extra call site is paid back by removing the callee
            #     from the caller's overlay. Below 16 B, splitting
            #     usually loses.
            #   - cs + ds ≥ 30 B: filter out toy programs where the
            #     entire bundle is small enough to fit anyway.
            #
            # No `_main_p` restriction; no call_count gate (a single
            # call site can still benefit from the split when helpers
            # would otherwise inflate the merged bundle past the slot).
            if ds < 16:
                return False
            if cs + ds < 30:
                return False
            return True

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
            #
            # OLED runtime helpers join the same tier: any program where
            # all `oled_text_at(...)` calls live in user functions would
            # otherwise see __oled_stream_chars / __oled_set_window
            # bundled into each overlay slot, defeating the streaming
            # architecture's whole point (single resident copy, called
            # from multiple sites). Force-resident them.
            _force_resident_helpers = {
                '__tone', '__play_note', '__delay_Nms',
            }
            # When OLED runtime helpers are present, force the I²C
            # primitives `__i2c_st_only` / `__i2c_sp` resident. Init code
            # (`__oled_init`) calls them at stage 1, AND runtime OLED
            # helpers call them in their bundled overlay copies. Keeping
            # one resident copy is cheaper than (a) bundling into each
            # overlay path that needs them PLUS (b) emitting a renamed
            # `_init` copy in stage 1 — the latter pushes stage 1 over
            # the 254 B limit on programs like `keypad_oled.c`.
            _helpers_used = getattr(self, '_lcd_helpers', set())
            _oled_runtime_used = bool(_helpers_used & {
                '__oled_putc_xfer', '__oled_stream_chars', '__oled_set_window'
            } | {h for h in _helpers_used if h.startswith('__oled_fill_')})
            if _oled_runtime_used:
                _force_resident_helpers |= {'__i2c_st_only', '__i2c_sp'}
            # Cold-tier infrastructure must stay resident — the dispatcher
            # is reached from any cold call site, and the bulk-read loop
            # is called from the dispatcher itself (resident → resident).
            # The cold-tier I²C primitives have to stay reachable too so
            # the dispatcher can drive the AT24C512 without relying on an
            # overlay-loaded helper. Without these forces, the
            # conditionally-resident pruning at the end of step 8b kicks
            # __cold_call back out into overlay-eligible territory, and
            # any cold call from main jumps into whatever overlay
            # happens to be loaded at __cold_call's address.
            if '__cold_call' in _helpers_used:
                _force_resident_helpers |= {
                    '__cold_call', '__cold_load_helper',
                    '__i2c_sb', '__i2c_rb',
                }
                # (C) Helpers transitively called from cold function bodies.
                _force_resident_helpers |= getattr(
                    self, '_cold_function_callee_helpers', set())
            # OLED helpers (__oled_stream_chars, __oled_set_window) used to be
            # in this set to prevent the partitioner from duplicating them
            # into each user-function overlay. Empirically, with the streaming
            # architecture the partitioner only duplicates when there are
            # multiple user-function call sites, and even then the right fix
            # is bundle-mode (one overlay holding the helpers + the user fn
            # that calls them). Force-residing them was preventing the
            # bundling that programs like keypad_oled.c need to fit page 0.

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
        # Persistent across retries: helpers flagged for cold-overlay
        # extraction (their own overlay block, called via reload thunk).
        # Populated by the wrap-recovery path when promotion-to-resident
        # would over-bloat the kernel.
        keep_standalone = set()
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

            # Detect fall-through OR tail-jump-to-helper: if a helper body
            # ends with `j __OTHER` where __OTHER is another bundleable
            # helper, the two MUST be co-located in any overlay that
            # bundles the first — otherwise the bundled body's `j __OTHER`
            # references a target that's neither resident nor in the
            # overlay (the validator at the end of step 9 catches this and
            # raises). Merge them the same way as a literal fall-through.
            #
            # Without this: __lcd_chr (whose body ends with
            # `j __lcd_send_raw`) gets bundled standalone and the
            # `j __lcd_send_raw` target dangles — affects every program
            # that calls lcd_*_u8 / printf_temp_u8 via overlay
            # (lcd_temp_overlay.c, lcd_temp.c after the printf refactor,
            # overlay_dashboard.c, etc.).
            terminators = {'ret', 'hlt'}
            helper_order = sorted(helper_bodies.keys(),
                                  key=lambda h: helper_bodies[h][0])
            helper_label_set = {f'{h}' for h in helper_bodies}
            for idx_h, hname in enumerate(helper_order):
                hstart, hend, hlines = helper_bodies[hname]
                # Find last real instruction (and its operand if any)
                last_instr = ''
                last_operand = None
                for hl in reversed(hlines):
                    stripped = hl.strip()
                    if stripped and not stripped.endswith(':') and not stripped.startswith(';'):
                        parts = stripped.split()
                        last_instr = parts[0]
                        last_operand = parts[1] if len(parts) > 1 else None
                        break
                is_ret_or_hlt = last_instr in terminators
                # Tail-jump to another helper: `j __X` where __X is in
                # helper_bodies. This needs the same co-location handling
                # as fall-through.
                tail_jumps_to = None
                if last_instr == 'j' and last_operand and last_operand in helper_label_set:
                    tail_jumps_to = last_operand
                # is_jump_terminator: any j-prefixed (not jal) that is NOT
                # a tail-jump to a co-bundleable helper.
                is_jump_term = (last_instr.startswith('j') and
                                not last_instr.startswith('jal') and
                                tail_jumps_to is None)
                is_terminator = is_ret_or_hlt or is_jump_term
                # Case 1: literal fall-through (no terminator) — merge with
                # next helper in source order.
                if not is_terminator and tail_jumps_to is None and idx_h + 1 < len(helper_order):
                    next_name = helper_order[idx_h + 1]
                    ns, ne, nlines = helper_bodies[next_name]
                    merged_lines = hlines + [new_resident[ns]] + nlines
                    helper_bodies[hname] = (hstart, ne, merged_lines)
                # Case 2: tail-jump to a specific bundleable helper — merge
                # with THAT helper (not necessarily the next in source).
                elif tail_jumps_to is not None and tail_jumps_to in helper_bodies:
                    ts, te, tlines = helper_bodies[tail_jumps_to]
                    # Strip the trailing `j __X` since __X's label is now
                    # immediately following — the jump is dead. Saves 1 B
                    # per merged tail. Peephole would normally catch this
                    # but is suppressed on overlay sections (manifest
                    # offsets), so we elide it at merge time when sizes
                    # are still mutable.
                    hlines_t = list(hlines)
                    while hlines_t and (not hlines_t[-1].strip() or
                                         hlines_t[-1].strip().startswith(';')):
                        hlines_t.pop()
                    if hlines_t and hlines_t[-1].strip() == f'j {tail_jumps_to}':
                        hlines_t.pop()
                        hlines = hlines_t
                    merged_lines = hlines + [new_resident[ts]] + tlines
                    # Preserve the SOURCE helper's original (hstart, hend) so
                    # downstream "is this index inside helper X's body" checks
                    # correctly identify lines in `new_resident[hstart..hend]`
                    # as belonging to this helper. Earlier code stored the
                    # tail-target's end here, which produced an empty range
                    # for source helpers whose tail target lived elsewhere
                    # in `new_resident` — causing the bundleable filter to
                    # falsely classify deeper helpers (e.g. `__i2c_st_only`
                    # called from inside `__oled_set_window`'s body) as
                    # called from resident code.
                    helper_bodies[hname] = (hstart, hend, merged_lines)
                # Don't remove the merged-in helper from helper_bodies — it
                # can still be bundled independently by overlays that call
                # it directly.

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
            #
            # Helpers extracted to their OWN overlay (rather than promoted to
            # kernel-resident) — used as a fallback when promotion would
            # over-bloat the kernel. Each gets its own overlay block; calls
            # from other overlays go through the cross-overlay reload-thunk
            # mechanism. Pays only ~7 B kernel per (caller, helper) edge
            # vs the helper's full body.
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
                        if h in helper_bodies and h not in keep_resident
                        and h not in keep_standalone)
                    if bundled_size > est_overlay_avail:
                        wrapping_overlays.append((oi, oname, bundled_size,
                            needed_helpers - keep_resident - keep_standalone))
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
                # For LARGE helpers (≥ 32 B), prefer cold-helper extraction
                # over promotion-to-resident. Promotion grows the kernel by
                # the helper's full body; extraction grows it by ~9 B per
                # (caller, helper) edge. The reload thunk path was built for
                # exactly this trade and works for any helper that's
                # called from a single user-fn overlay (cycle cost
                # acceptable on cold paths). Phase 6 wrap-aware promotion
                # used to greedily resident-promote any helper that fit
                # under the 230 B kernel cap, which over-bloated kernels
                # for OLED-heavy programs and prevented later cold-helper
                # extraction from helping (helpers were already resident).
                if best_size >= 32:
                    if best in keep_standalone:
                        # Already extracted; nothing more to do for this helper.
                        # Force exit so we don't loop forever on the same one.
                        break
                    keep_standalone.add(best)
                    # Estimate one thunk per caller of this helper.
                    n_callers = sum(
                        1 for _, ol2, _ in overlay_asm_blocks
                        if any(line2.strip() == f'jal {best}' for line2 in ol2)
                    )
                    thunk_cost = max(1, n_callers) * 9
                    est_kernel_growth += thunk_cost
                    est_overlay_avail = 250 - (est_kernel_base + est_kernel_growth)
                    import sys
                    print(f"  Phase 6: extracting {best} ({best_size}B) to own "
                          f"overlay (thunk cost ~{thunk_cost}B for {n_callers} "
                          f"caller(s))", file=sys.stderr)
                    continue  # re-evaluate wrapping with the extraction applied
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
            # Standalone-overlay helpers are also excluded from bundling —
            # they get their own overlay block emitted later.
            remove_from_bundleable = keep_resident | merged_into_resident | keep_standalone
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

                # Append renamed helper bodies. Track top-level `__X_ovN:`
                # labels we've already emitted in this overlay; if a
                # subsequent helper's merged body would re-emit an
                # already-seen label (because two bundled helpers
                # tail-jumped to the same target and Case-2 merge
                # inlined the body into both), truncate at the duplicate
                # label and emit `j __X_ovN` to direct control flow to
                # the first copy. Saves 4-9 B per duplicated tail.
                seen_emitted_labels = set()
                for hname in sorted_for_ov:
                    hstart, _, hlines = helper_bodies[hname]
                    helper_full = [new_resident[hstart]] + hlines
                    # First, rename labels in each line (same logic as before)
                    renamed = []
                    for hl in helper_full:
                        l = hl
                        for lbl in all_bundled_labels:
                            l = l.replace(f'{lbl}:', f'{lbl}{suffix}:')
                            l = l.replace(f'jal {lbl}', f'jal {lbl}{suffix}')
                            if f'j {lbl}' in l and f'jal {lbl}' not in l:
                                for jtype in ('jnz', 'jz', 'jc', 'jnc'):
                                    if f'{jtype} {lbl}' in l:
                                        break
                                else:
                                    l = l.replace(f'j {lbl}', f'j {lbl}{suffix}')
                        import re as _re
                        for m in _re.finditer(r'\.([\w]+)', l):
                            ref = '.' + m.group(1)
                            if any(ref + ':' in h2.strip() for h2 in [new_resident[hstart]] + hlines):
                                l = l.replace(ref, ref + suffix)
                        renamed.append(l)
                    # Walk renamed; on first duplicate top-level label,
                    # truncate and emit a forward jump.
                    truncated = False
                    for ri, l in enumerate(renamed):
                        s = l.strip()
                        if s.endswith(':') and s.startswith('__') and not s.startswith('.'):
                            lbl_name = s[:-1]
                            if lbl_name in seen_emitted_labels:
                                # Duplicate. Replace this line and the rest of
                                # `renamed` with a single `j lbl_name`.
                                new_olines.append(f'\tj {lbl_name}')
                                truncated = True
                                break
                            seen_emitted_labels.add(lbl_name)
                        new_olines.append(l)
                    if not truncated:
                        # No duplicate encountered; renamed[] fully appended above.
                        pass
                new_fsize = measure_lines(new_olines)
                overlay_asm_blocks[oi] = (oname, new_olines, new_fsize)

            # Remove all bundled helpers from resident.
            # Use ORIGINAL ranges (pre-merge) to avoid accidentally removing
            # resident helpers that were merged into a bundled helper's body.
            for hname in bundleable_helpers:
                orig_start, orig_end = helper_original_ranges.get(hname, (0, 0))
                for ri in range(orig_start, orig_end):
                    new_resident[ri] = ''

            # ── Step 9c: Standalone-helper overlay emission ──
            # The wrap-recovery path may have flagged helpers for extraction
            # to their OWN overlay rather than promotion to kernel. Each
            # gets a new overlay block with:
            #   - the helper's body (primary label kept BARE so the
            #     reload thunk's `jal helper` resolves correctly)
            #   - any non-resident bundleable deps bundled in (transitively)
            # Bundling deps into the standalone overlay (rather than
            # force-resident them) avoids kernel growth, which prevents the
            # cascade where extracting one helper forces deps resident →
            # slot shrinks → another overlay starts wrapping → another
            # extraction. For `keypad_oled.c` specifically: extracting
            # `__oled_stream_chars` while bundling its `__i2c_st_only`
            # dep into the standalone overlay keeps slot at 116 B
            # (instead of dropping to 101 B), so `_poll_once` (104 B) fits
            # without needing to also extract the HOT `__keypad_scan`.
            if keep_standalone:
                import re as _re_sa
                for h in sorted(keep_standalone):
                    if h not in helper_bodies:
                        continue
                    hstart, _, hlines = helper_bodies[h]
                    # Remove the standalone helper's body from new_resident.
                    s_orig_start, s_orig_end = helper_original_ranges.get(h, (0, 0))
                    for ri in range(s_orig_start, s_orig_end):
                        if ri < len(new_resident):
                            new_resident[ri] = ''
                    new_oi = len(overlay_asm_blocks)
                    suffix = f'_ov{new_oi}'
                    # Collect non-resident bundleable deps transitively. A
                    # dep is bundled into THIS standalone overlay if:
                    #   - its body is in helper_bodies
                    #   - it's NOT already kernel-resident (in _NO_OVERLAY)
                    #   - it's NOT already merged into the helper's body
                    #     (tail-jump merge produces inline copy)
                    #   - it's NOT another standalone helper (which has
                    #     its own overlay; cross-call would need its own
                    #     reload thunk — not handled here)
                    merged_in = set()
                    for hl in hlines:
                        s2 = hl.strip()
                        if s2.endswith(':') and s2.startswith('__') and not s2.startswith('.'):
                            merged_in.add(s2[:-1])
                    needed_deps = []  # ordered (topo-sortish via insertion order)
                    seen_dep = {h} | merged_in
                    frontier = [h]
                    while frontier:
                        cur = frontier.pop()
                        _, _, cur_lines = helper_bodies.get(cur, (0, 0, []))
                        for hl in cur_lines:
                            s_c = hl.strip()
                            if not s_c.startswith('jal '):
                                continue
                            tgt = s_c.split()[1]
                            if not tgt.startswith('__'):
                                continue
                            if tgt not in helper_bodies:
                                continue
                            if tgt in seen_dep:
                                continue
                            if f'{tgt}:' in _NO_OVERLAY:
                                continue
                            if tgt in keep_standalone:
                                continue
                            seen_dep.add(tgt)
                            needed_deps.append(tgt)
                            frontier.append(tgt)
                    # Build the all-bundled-labels set so renaming covers
                    # every non-primary label that appears in our overlay.
                    all_bundled_labels = set()
                    for dep in [h] + needed_deps:
                        _, _, dep_lines = helper_bodies[dep]
                        for dl in dep_lines:
                            s_dl = dl.strip()
                            if s_dl.endswith(':') and s_dl.startswith('__') and not s_dl.startswith('.'):
                                lbl_ = s_dl[:-1]
                                if lbl_ != h:
                                    all_bundled_labels.add(lbl_)
                    # Helper for renaming a single line.
                    def _rename_line(line, scope_lines):
                        result = line
                        for lbl in all_bundled_labels:
                            result = result.replace(f'{lbl}:', f'{lbl}{suffix}:')
                            result = result.replace(f'jal {lbl}', f'jal {lbl}{suffix}')
                            if f'j {lbl}' in result and f'jal {lbl}' not in result:
                                for jtype in ('jnz', 'jz', 'jc', 'jnc'):
                                    if f'{jtype} {lbl}' in result:
                                        break
                                else:
                                    result = result.replace(f'j {lbl}', f'j {lbl}{suffix}')
                        for m in _re_sa.finditer(r'\.([\w]+)', result):
                            ref = '.' + m.group(1)
                            if any(ref + ':' in h2.strip() for h2 in scope_lines):
                                result = result.replace(ref, ref + suffix)
                        return result
                    # Emit primary label (BARE — for reload-thunk lookup).
                    renamed = [f'{h}:']
                    # Emit helper body (renamed).
                    for hl in hlines:
                        renamed.append(_rename_line(hl, hlines))
                    # Emit each bundled dep's body (renamed). Use the same
                    # label-emit dedup as Fix #2 (skip if a top-level
                    # `__name_ovK:` label has already been emitted).
                    seen_emit = {h}
                    for lbl in all_bundled_labels:
                        # already-emitted from helper body (its own merged labels)
                        seen_emit.add(lbl + suffix)
                    for dep in needed_deps:
                        _, _, dep_lines = helper_bodies[dep]
                        dep_label_orig = f'{dep}:'
                        # Emit the dep's primary label (renamed).
                        renamed.append(_rename_line(dep_label_orig, dep_lines))
                        for dl in dep_lines:
                            renamed.append(_rename_line(dl, dep_lines))
                    block_size = measure_lines(renamed)
                    overlay_asm_blocks.append((h, renamed, block_size))
                    func_to_overlay_idx[h] = new_oi
                    overlay_meta.append((new_oi, h, block_size))
                    # Add (caller, helper) edges for every overlay that has a
                    # bare `jal h` call site. Step 9a will emit the reload thunks.
                    for ci, (cname, clines, _) in enumerate(overlay_asm_blocks):
                        if ci == new_oi:
                            continue
                        for line2 in clines:
                            if line2.strip() == f'jal {h}':
                                reload_thunk_edges.add((cname, h))
                                break

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
            import os as _dbg_p6
            _phase6_disabled = _dbg_p6.environ.get('MK1_DISABLE_PHASE6') == '1'
            for oi, (oname, olines, ofsize) in enumerate(overlay_asm_blocks):
                cur_lines = olines
                if _phase6_disabled:
                    continue
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
                    # Short prefix `__ovrl_` (was `__ovreload_`) so the
                    # generated label fits the ESP32 assembler's 32-char
                    # symbol buffer for helper-callee thunks like
                    # `_show_oled_stream_chars`.
                    thunk = f'__ovrl_{safe_caller}_{safe_callee}'
                    replaced = False
                    oname, olines, _ofsize = overlay_asm_blocks[caller_idx]
                    new_olines = []
                    # Rewrite EVERY `jal callee` in this overlay (not just
                    # the caller's body) so calls extracted into Phase 6
                    # abstraction thunks (`__ovthunk_*`) also go through the
                    # reload thunk. Without this, a 7× repeated call pattern
                    # extracted into __ovthunk_0_0 would still contain a bare
                    # `jal callee` whose target lives in a different overlay
                    # — breaking the step 9b self-containment validator.
                    for line in olines:
                        s = line.strip()
                        if s == f'jal {callee}':
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
            INIT_PREFIXES = ('ddrb_imm', 'exrw 2', 'exw ', 'ldi $d,', 'ddra_imm', 'ora_imm', 'push_imm')
            # Helpers that are safe to call during init (don't end init phase)
            # __delay_cal included even though it's NOT always init-only (the
            # `_prefer_delay_cal_init` retry can demote it to overlay) — when
            # it's an overlay, the call still appears in init-compatible
            # context because programs typically order
            # `i2c_init(); delay_calibrate(); lcd_init();` and the calibrate
            # call must not end the init phase or `jal __lcd_init` ends up
            # stuck in runtime while __lcd_init body lives in stage 1
            # (section-mismatch error). The overlay-loaded form is still
            # safe in init phase because overlay_load doesn't itself need
            # any post-init kernel state.
            INIT_COMPAT_HELPERS = {'__i2c_stream', '__i2c_st', '__i2c_sp',
                                   '__i2c_rs',
                                   '__lcd_init', '__tone_setup',
                                   '__delay_cal'}

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
                            # If no MORE init-compatible jals are coming, end
                            # init phase AFTER this one. Without this look-
                            # ahead, init-prefix-matching lines after the
                            # last init jal (e.g. an `exrw 2; ddrb 0x01;
                            # ddrb 0x03` START sequence prefixing a runtime
                            # rtc_read_temp transaction) get pulled into
                            # init, leaving the rest of the runtime
                            # transaction (`ldi $a, 0xD0; jal __i2c_sb`)
                            # without a START. Empirically: `lcd_init();
                            # rtc_read_temp(); printf(...)` produced 0
                            # output because the START went to init and the
                            # address byte went to runtime, so the bus had
                            # no START before the address — slave didn't
                            # respond. Terminating init phase right after
                            # the last init-jal sends the runtime START to
                            # runtime where it belongs.
                            has_more_init_jal = False
                            for fli in range(mi + 1, len(main_lines)):
                                fs = main_lines[fli].strip()
                                if fs.startswith('jal '):
                                    ft = fs.split()[1]
                                    if is_init_only(ft) or ft in INIT_COMPAT_HELPERS:
                                        has_more_init_jal = True
                                    # any jal (init or not) terminates the
                                    # look-ahead — first jal decides
                                    break
                            if not has_more_init_jal:
                                init_extracted.append(line)
                                in_init_phase = False
                                if _iidbg:
                                    print(f"  [init-dbg] LAST-INIT-JAL: {s!r}", file=sys.stderr)
                                continue
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
            # Cold-helper metadata: write (addr_hi, addr_lo, len) into
            # page3[COLD_TABLE_BASE + id*3] for each cold helper. Three
            # `ldi $a,N; ldi $b,M; iderefp3` triples per helper = 15 B.
            # Lives in the runtime kernel (not stage-1 init) so it
            # re-runs on reset; that's fine — all writes are to constant
            # addresses with constant values.
            cold_helpers = getattr(self, 'cold_helpers', [])
            if cold_helpers:
                preamble.append('\t; cold-helper metadata table init')
                for cold_id, ee_addr, length, _name in cold_helpers:
                    entries = [
                        ((ee_addr >> 8) & 0xFF, COLD_TABLE_BASE + cold_id * 3 + 0),
                        (ee_addr & 0xFF,        COLD_TABLE_BASE + cold_id * 3 + 1),
                        (length & 0xFF,         COLD_TABLE_BASE + cold_id * 3 + 2),
                    ]
                    for value, p3_addr in entries:
                        preamble.append(f'\tldi $a,{value}')
                        preamble.append(f'\tldi $b,{p3_addr}')
                        preamble.append('\tiderefp3')
                # cold_current_id = 0xFF sentinel (no cold helper loaded
                # initially). Required for the cold→cold dispatcher's
                # save/restore logic to correctly skip caller-reload
                # when the chain unwinds back to main/kernel.
                preamble.append('\t; cold_current_id = 0xFF (sentinel)')
                preamble.append('\tldi $a,0xFF')
                preamble.append('\tldi $b,0xB4')
                preamble.append('\tideref2')
            if getattr(self, '_ee_preload_bytes', 0) > 0:
                preamble.append('\tjal __eeprom_preload')
            preamble.extend([
                self._i2c_ddrb_str(0x03) + '\t; bus recovery: SCL low, SDA low',
                self._i2c_ddrb_str(0x01) + '\t; SCL high, SDA low',
                self._i2c_ddrb_str(0x00) + '\t; SCL high, SDA high (STOP/idle)',
            ])
            main_lines = ['_main:'] + preamble + runtime_main
            if getattr(self, '_needs_tone_init', False):
                tone_ddra = self._port_shadow_mask('DDRA') | 0x02
                # DDRA-setup must land AFTER the precompute (which calls
                # __tone_setup overlay and that path may temporarily clobber
                # DDRA) and BEFORE any user-code path that calls __play_note
                # / __tone. Two precompute forms exist in compiler output:
                #   (a) `jal __tone_precompute` — eeprom-mode, function call
                #   (b) inline `.__precomp:` loop with trailing `jnz .__precomp`
                #       — non-eeprom mode (tone_isolation.c et al)
                # Pre-2026-04-26 only form (a) was matched, so form (b)
                # programs ran with DDRA=0 (whatever __delay_cal left it),
                # PA1 stayed INPUT, no audio reached the piezo. Match both.
                _inserted = False
                for _mi, _ml in enumerate(main_lines):
                    if _ml.strip() == 'jal __tone_precompute':
                        main_lines.insert(_mi + 1, f'\tddra_imm 0x{tone_ddra:02X}')
                        _inserted = True
                        break
                if not _inserted:
                    # Form (b): find last `jnz .__precomp` (precompute loop
                    # back-edge) and insert immediately after.
                    for _mi in range(len(main_lines) - 1, -1, -1):
                        _ml_s = main_lines[_mi].strip()
                        if _ml_s.startswith('jnz') and '.__precomp' in _ml_s:
                            main_lines.insert(_mi + 1, f'\tddra_imm 0x{tone_ddra:02X}')
                            _inserted = True
                            break
                if not _inserted:
                    # Fallback: insert at top of runtime_main (after preamble).
                    # This covers any future precompute form. Costs nothing
                    # for programs that already had a match.
                    main_lines.insert(len(['_main:'] + preamble),
                                      f'\tddra_imm 0x{tone_ddra:02X}')

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
                # NOTE: a 2026-04-26 attempt to remove the 9-clock loop
                # (in the hope it was the root cause of keypad scan
                # failures on the DFRobot AiP31068L LCD) regressed
                # `tone duration timing` and `rgb lcd smoke test` in
                # hw_regression. The recovery is genuinely load-bearing
                # for tone+lcd programs.
                loader.append('.ov_done:')
                loader.append('\tldi $a,9')
                loader.append('.ov_br:')
                loader.append(self._i2c_ddrb_str(0x02))   # SCL LOW, SDA released
                loader.append(self._i2c_ddrb_str(0x00))   # SCL HIGH, SDA released
                loader.append('\tdec')
                loader.append('\tjnz .ov_br')
                loader.append(self._i2c_ddrb_str(0x01))   # SDA LOW, SCL HIGH (STOP prep)
                loader.append(self._i2c_ddrb_str(0x00))   # SDA HIGH = STOP
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
            # Cold-helper-overlay extraction: when wrap-safety can't find a
            # promotion candidate (kernel would over-shrink), try moving the
            # helper to its OWN overlay instead, called via the existing
            # __ovreload_caller_callee thunk mechanism. Kernel grows only by
            # ~7 B per (caller, helper) edge instead of the helper's full body,
            # so this works when promotion-to-resident wouldn't.
            if _wrapping_indices and _placement_retries < 5 and bundleable_helpers:
                # Estimate one reload-thunk overhead per unique caller×helper
                # edge as ~9 B (label + 6 instructions).
                THUNK_COST = 9
                proj_region_solo = _ov_avail - THUNK_COST
                # Pick helper that:
                #   - is bundled in some wrapping overlay
                #   - alone (its body) fits proj_region_solo
                #   - extraction shrinks the wrapping overlay below proj_region_solo
                best_solo = None
                best_solo_score = 0
                for h in bundleable_helpers:
                    if h in keep_standalone:
                        continue
                    hsize_h = measure_lines(helper_bodies.get(h, (0, 0, []))[2]) + 1
                    if hsize_h > proj_region_solo:
                        continue  # helper alone wouldn't fit
                    if hsize_h <= THUNK_COST:
                        continue  # not worth the thunk cost
                    # Check effect on each wrapping overlay
                    fits_after = 0
                    total_shrink = 0
                    for oi in _wrapping_indices:
                        oname_w, olines_w, sz_w = overlay_asm_blocks[oi]
                        has_h = any(ol.strip().startswith(f'{h}_ov') and
                                    ol.strip().endswith(':') for ol in olines_w)
                        if not has_h:
                            continue
                        new_sz = sz_w - hsize_h
                        if new_sz <= proj_region_solo:
                            fits_after += 1
                            total_shrink += (sz_w - new_sz)
                    score = fits_after * 1000 + total_shrink
                    if score > best_solo_score:
                        best_solo_score = score
                        best_solo = (h, hsize_h, fits_after)
                if best_solo:
                    h, hsize_h, fits_after = best_solo
                    import sys
                    print(f"  Cold-helper extraction: moving {h} ({hsize_h}B) to own "
                          f"overlay (fits {fits_after} of {len(_wrapping_indices)} "
                          f"wrapping overlays after extraction)",
                          file=sys.stderr)
                    keep_standalone.add(h)
                    bundleable_helpers = [hh for hh in bundleable_helpers if hh != h]
                    # Note: the helper's bundleable deps (e.g. __i2c_st_only)
                    # do NOT get force-resident here. Step 9c bundles them
                    # into the standalone overlay alongside the helper body,
                    # avoiding kernel growth. This prevents the cascade where
                    # forcing one dep resident shrinks the slot enough that
                    # another (unrelated) overlay starts wrapping and a hot
                    # helper has to be extracted to compensate. Trade-off:
                    # small dep ends up in two overlays (the standalone +
                    # whichever original overlay still calls it via another
                    # bundled helper); 6-12 B duplication beats 6 B kernel
                    # growth when slot is the binding constraint.
                    _placement_retries += 1
                    _retry_bundling = True
                    continue
            if _wrapping_indices:
                details = ', '.join(
                    f"{overlay_asm_blocks[i][0]}={overlay_asm_blocks[i][2]}B"
                    for i in sorted(_wrapping_indices)
                )
                import os as _dbg_os
                if _dbg_os.environ.get('MK1_DUMP_WRAPPING') == '1':
                    import sys as _dbg_sys
                    _ldr_b = measure_lines(_loader_chk)
                    print(f"[KDUMP] kernel total = {_kernel_chk}B "
                          f"(loader={_ldr_b}B, main={main_size}B, helpers={runtime_helper_size}B)",
                          file=_dbg_sys.stderr)
                    print(f"[KDUMP] slot available = {_ov_avail}B (page_0 250B - kernel {_kernel_chk}B)",
                          file=_dbg_sys.stderr)
                    # `helper_bodies[h][2]` is the *merged* body — it
                    # includes any tail-jump-target's bytes, which double-
                    # counts shared helpers like __lcd_send_raw across
                    # __lcd_chr and __lcd_cmd. Use `helper_original_ranges`
                    # for each helper's own pre-merge byte count, then
                    # also report the shared (merged-in) helpers
                    # separately so the table sums to actual kernel size.
                    _kernel_helpers = []
                    _named = set()
                    for _ke in _NO_OVERLAY:
                        _hn = _ke[:-1] if _ke.endswith(':') else _ke
                        if _hn in helper_bodies:
                            _named.add(_hn)
                            _ostart, _oend = helper_original_ranges.get(_hn, (0, 0))
                            _own_lines = new_resident[_ostart+1:_oend]  # skip label
                            _kernel_helpers.append((_hn, measure_lines(_own_lines)))
                    # Tail-jump targets that aren't in _NO_OVERLAY but are
                    # in helper_bodies: these are co-located helpers
                    # (e.g. __lcd_send_raw) whose bytes ALSO live in the
                    # kernel because their callers (__lcd_chr/__lcd_cmd)
                    # tail-jump into them. Report them so the sum lines up.
                    for _hn, (_ostart, _oend, _) in helper_bodies.items():
                        if _hn in _named:
                            continue
                        _own_lines = new_resident[_ostart+1:_oend]
                        _sz = measure_lines(_own_lines)
                        if _sz > 0:
                            _kernel_helpers.append((_hn + ' (shared)', _sz))
                    _kernel_helpers.sort(key=lambda x: -x[1])
                    print(f"[KDUMP] kernel-resident helpers ({len(_kernel_helpers)} entries):",
                          file=_dbg_sys.stderr)
                    for _hn, _hb in _kernel_helpers:
                        print(f"[KDUMP]   {_hn}: {_hb}B", file=_dbg_sys.stderr)
                    if bundleable_helpers:
                        _bun = []
                        for _h in bundleable_helpers:
                            _bun.append((_h, measure_lines(helper_bodies.get(_h, (0,0,[]))[2])))
                        _bun.sort(key=lambda x: -x[1])
                        print(f"[KDUMP] bundleable helpers (placed in overlays): "
                              f"{', '.join(f'{n}={s}B' for n,s in _bun)}",
                              file=_dbg_sys.stderr)
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

        # EEPROM-backed overlays: stored in EEPROM, preloaded into freed
        # SRAM space during init. Historically the destination was always
        # page 3 (using `iderefp3`) — that page's [0..meta_base-1] region
        # is freed up after self-copy moves the kernel image to code. But
        # programs with few page-1 globals (or sparse page-2 native
        # overlay use) often have *more* free space in page 1 or page 2.
        # Picking the destination based on which page has the most slack
        # roughly doubles the EEPROM-backed budget for those programs and
        # unblocks corpus programs that currently overflow on the page-3
        # cap alone.
        #
        # Strategy: prefer page 3 if it fits (preserves existing behavior
        # for programs that already work). Else fall back to page 1, then
        # page 2. Error if none has space — caller knows the EEPROM
        # payload is bigger than the largest free SRAM region.
        if has_eeprom:
            ee_total = sum(fsize for _, _, _, fsize, _ in ee_overlays)
            # Free space in each candidate destination, after native overlay
            # placement above:
            #   p3: kernel-image region freed by self-copy ≈ p3_code_offset
            #   p1: leftover after globals + native p1 overlays = p1_capacity
            #   p2: leftover within OVERLAY_BYTES = p2_capacity
            _opts = [
                ('p3', p3_code_offset, 0),
                ('p1', p1_capacity, p1_code_offset),
                ('p2', p2_capacity, p2_code_offset),
            ]
            _fits = [(t, cap, start) for t, cap, start in _opts if cap >= ee_total]
            if not _fits:
                raise Exception(
                    f"EEPROM payload {ee_total}B exceeds every preload "
                    f"destination's free space (p3={p3_code_offset}B, "
                    f"p1={p1_capacity}B, p2={p2_capacity}B)"
                )
            _p3_fit = next((o for o in _fits if o[0] == 'p3'), None)
            if _p3_fit is not None:
                preload_target, _, preload_dest_start = _p3_fit
            else:
                preload_target, _, preload_dest_start = max(_fits, key=lambda x: x[1])

            # Re-map EEPROM overlay offsets into the chosen destination's
            # address range. Each entry's 5th tuple slot is the runtime
            # address the loader's deref will read.
            cur_off = preload_dest_start
            for ei in range(len(ee_overlays)):
                idx, name, asm_lines, fsize, _ee_offset = ee_overlays[ei]
                ee_overlays[ei] = (idx, name, asm_lines, fsize, cur_off)
                cur_off += fsize

            # Merge the EEPROM-backed entries into the target page's
            # overlay list. Manifest emission below reads these lists; the
            # `pages` table assigns the correct deref op (deref/deref2/
            # derefp3) based on which list each name lives in.
            if preload_target == 'p3':
                # Existing behavior: ee bytes occupy the freed kernel-image
                # region BELOW where native p3 overlays start.
                p3_overlays = list(ee_overlays) + list(p3_overlays)
                has_p3 = True
            elif preload_target == 'p1':
                # ee bytes go AFTER native p1 overlays + globals.
                p1_overlays = list(p1_overlays) + list(ee_overlays)
                has_p1 = True
                p1_code_offset = cur_off
                p1_capacity = 256 - cur_off
            elif preload_target == 'p2':
                p2_overlays = list(p2_overlays) + list(ee_overlays)
                has_p2 = True
                p2_code_offset = cur_off
                p2_capacity = P2_OVERLAY_BYTES - cur_off

            # Stash for the preload code emitter below — chooses ideref op.
            self._ee_preload_target = preload_target

            import sys
            print(f"  EEPROM-backed: {len(ee_overlays)} overlays ({ee_total}B) "
                  f"preloaded from EEPROM 0x{ee_overlay_base:04X} → "
                  f"{preload_target}[{preload_dest_start}..{preload_dest_start+ee_total-1}]",
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
            # Pick the right ideref op for the chosen preload destination.
            # Set by `has_eeprom` block above; defaults to page 3 for safety.
            _preload_target = getattr(self, '_ee_preload_target', 'p3')
            _ideref_op = {'p3': 'iderefp3',
                          'p1': 'ideref',
                          'p2': 'ideref2'}[_preload_target]
            # Page 3 historically relied on C being zero from CPU reset
            # (saves 2B). Pages 1/2 need C initialized to the preload start
            # offset (where native overlays + globals end). Emit `ldi $c,N`
            # only when the target needs a non-zero starting offset.
            ee0 = ee_overlays[0][4] if ee_overlays else 0
            self_copy.extend([
                '; ── EEPROM preload (inline sequential read) ──',
                '\texrw 2',
                self._i2c_ddrb_str(0x01),
                self._i2c_ddrb_str(0x03),
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
                self._i2c_ddrb_str(0x02),        # SCL LOW, SDA released
                self._i2c_ddrb_str(0x00),        # SCL rises → idle
                self._i2c_ddrb_str(0x01),        # SDA falls with SCL HIGH → REP START
                self._i2c_ddrb_str(0x03),        # SCL falls → ready for address byte
                '\tldi $a,0xAF',
                '\tjal __i2c_sb',
                f'\tldi $d,{ee_preload_bytes}',
            ])
            # $c is the page-X write address counter. For p3 (default), the
            # CPU-reset-zeroed C is fine — saves 2B. For p1/p2 destinations
            # the start offset is non-zero (lives after globals + native
            # overlays), so emit an explicit ldi.
            if ee0 != 0:
                self_copy.append(f'\tldi $c,{ee0}')
            self_copy.extend([
                '.ee_byte:',
                # Inner bit loop: explicit push/pop counter. B accumulates
                # the byte; sllb shifts left (bit 0 always 0 after) then
                # `incb` sets bit 0 = 1 if SDA was high (1B vs 4B for the
                # mov/ori/mov OR sequence). Earlier sentinel-bit trick
                # (B=0x01, jc on shifted-out CF) saved ~12B but read only
                # 7 bits per byte — sllb's CF-set exited *before* the 8th
                # read, leaving every byte's bit 0 cleared.
                '\tldi $b,0',
                '\tpush_imm 8',
                '.ee_bit:',
                '\tmov $b,$a',
                '\tsll',                  # A = B << 1; bit 0 = 0 after shift
                '\tmov $a,$b',            # B = shifted
                self._i2c_ddrb_str(0x00),        # SCL HIGH (release, slave drives SDA)
                '\texrw 0',               # A = port B (bit 0 = SDA)
                '\ttst 0x01',
                '\tjz .ee_bz',
                '\tincb',                 # B[0] = 0 after sllb, so OR-1 == +1
                '.ee_bz:',
                self._i2c_ddrb_str(0x02),        # SCL LOW
                '\tpop $a',
                '\tdec',
                '\tpush $a',
                '\tjnz .ee_bit',
                '\tpop $a',
                # B=read_byte (from sentinel loop), C=address counter.
                # Need: A=byte, B=address for the page-X store op.
                # Old code used push/pop to save byte across C→B; the two-mov
                # version saves 2B by routing byte through A directly.
                '\tmov $b,$a',            # A = B (byte)
                '\tmov $c,$b',            # B = C (address)
                f'\t{_ideref_op}',        # page<X>[B=addr] = A=byte
                self._i2c_ddrb_str(0x03),
                self._i2c_ddrb_str(0x01),
                self._i2c_ddrb_str(0x03),
                # Pre-collapsed to `incc` / `decd` (would also be caught by
                # the section-aware final peephole on this `code` section).
                '\tincc',
                '\tdecd',
                '\tjnz .ee_byte',
                self._i2c_ddrb_str(0x03),
                self._i2c_ddrb_str(0x01),
                self._i2c_ddrb_str(0x00),
            ])
        self._ee_preload_bytes = 0  # disable kernel-side preload emission

        # Optional: dump entire page 3 to OI right after preload completes.
        # `MK1_DEBUG_DUMP_PAGE3=1` injects a 256-iteration loop that emits
        # each page-3 byte before main runs. The OI stream then carries 256
        # bytes of page-3 dump prefixing the program's normal output —
        # RUNLOG (with the firmware's expanded ring buffer) sees both. Use
        # this to verify what the EEPROM bit-bang actually wrote (the bug
        # class `--dump-bufs` doesn't cover — bit-bang corruption only
        # shows in live page-3 contents post-preload). Cost: ~12 bytes of
        # stage-1 init code; programs near the 254 B limit may overflow
        # with it on.
        import os as _dbg_os
        if _dbg_os.environ.get('MK1_DEBUG_DUMP_PAGE3') == '1':
            self_copy.extend([
                '; ── DEBUG: dump page 3 (256B) to OI ──',
                '\tldi $b,0',     # B = page3 index
                '\tldi $d,0',     # D = counter (decd wraps 0→255 for 256 iters)
                '.dp3_dump:',
                '\tmov $b,$a',
                '\tderefp3',      # A = page3[B]
                '\tout',          # OI = A
                '\tincb',
                '\tdecd',
                '\tjnz .dp3_dump',
            ])
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
                self._i2c_ddrb_str(0x01),
                self._i2c_ddrb_str(0x03),
                '\tldi $a,0xAE',
                '\tjal __i2c_sb',
                f'\tldi $a,{_ee_hi_byte}',
                '\tjal __i2c_sb',
                '\tclr $a',
                '\tjal __i2c_sb',
                self._i2c_ddrb_str(0x03),
                self._i2c_ddrb_str(0x01),
                self._i2c_ddrb_str(0x00),
                '\texrw 2',
                self._i2c_ddrb_str(0x01),
                self._i2c_ddrb_str(0x03),
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
                self._i2c_ddrb_str(0x00),
                '\texrw 0',
                '\ttst 0x01',
                '\tjz .eepl_bz',
                '\tmov $b,$a',
                '\tori 0x01,$a',
                '\tmov $a,$b',
                '.eepl_bz:',
                self._i2c_ddrb_str(0x02),
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
                self._i2c_ddrb_str(0x03),
                self._i2c_ddrb_str(0x01),
                self._i2c_ddrb_str(0x03),
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
                self._i2c_ddrb_str(0x03),
                self._i2c_ddrb_str(0x01),
                self._i2c_ddrb_str(0x00),
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
        # Defer manifest+pages emission until AFTER the p2 overlay
        # bytes are emitted. The C++ assembler's stack section uses a
        # monotonic write pointer (stack_size); the firmware's upload
        # loader copies `r.stack_size` bytes to MK1's stack page. If
        # manifest+pages emit FIRST (advancing stack_size to ~198) and
        # `section stack_code` overlays follow (writing at stack_size=
        # 198+), the manifest claims overlays start at offset 0 while
        # the actual bytes are at offset 198 — the loader's `deref2`
        # reads zeros, the slot loads blank, execution falls through to
        # the page's HLT fill. Fix: stack_code bytes go first (at
        # offsets matching the manifest's claim), then `org
        # P2_MANIFEST_BASE` pads up with 0x00 (forward, benign), then
        # the manifest writes at 192-197.
        manifest_directives = [
            '\tsection stack',
            f'\torg {P2_MANIFEST_BASE}',
            '__manifest:',
        ]
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
            manifest_directives.append(f'\tbyte {offset}')
            manifest_directives.append(f'\tbyte {fsize}')

        # Page table: [page] per overlay (1 byte each)
        # page values: 3=page3(derefp3), 1=page1(deref), 2=page2(deref2)
        # EEPROM-backed overlays are preloaded into page3, so they're page 3.
        manifest_directives.append('__pages:')
        p3_set = {name for _, name, _, _, _ in p3_overlays}
        p1_set = {name for _, name, _, _, _ in p1_overlays}
        p2_set = {name for _, name, _, _, _ in p2_overlays}
        for _, name, asm_lines, fsize, offset in all_placed_ordered:
            if name in p3_set:
                manifest_directives.append('\tbyte 3')
            elif name in p1_set:
                manifest_directives.append('\tbyte 1')
            elif name in p2_set:
                manifest_directives.append('\tbyte 2')
            else:
                manifest_directives.append('\tbyte 3')  # fallback: page3

        # __ov_entry: marks the start of the overlay region (address OVERLAY_REGION).
        # Emitted after manifest so it resolves to the correct code address.
        # Resident dispatch calls the requested function label after
        # _overlay_load copies the code.
        assembled.append('\tsection code')
        assembled.append(f'\torg {OVERLAY_REGION}')
        assembled.append('\tsection page3_code')
        assembled.append('__ov_entry:')

        # Pad page3 buffer up to where the partitioner claimed the first
        # NATIVE p3 overlay starts. `p3_code_offset` was set to
        # `meta_base + meta_table_size` (line ~8410) which adds headroom
        # over the actual kernel image size — so manifest claims the
        # overlay is at offset X but the assembler's natural page3_size
        # at this point is just `kernel_image_size`. Without this pad,
        # the bytes emit at kernel_image_size (e.g. 123) and the
        # manifest claims X (e.g. 135), so the loader's `derefp3` reads
        # zeros from page3[X], the slot loads blank, execution falls
        # into HLT fill or reset. Same bug class as the page-2 manifest
        # issue fixed in `73d9a6d`. The forward `org` pads with 0x00
        # which is fine — those bytes are never executed (they sit in
        # the gap between the kernel image and the first overlay).
        #
        # EEPROM-backed overlays appear in `p3_overlays` (they're loaded
        # into page3 at runtime via __eeprom_preload) but their bytes
        # live in the eeprom section, not in page3 at upload time. They
        # have manifest offsets starting at 0 (where the runtime preload
        # places them). Filter them out of the alignment calc — their
        # offsets are smaller than kernel_image_size and would force a
        # backward org that wipes the kernel image bytes. The native p3
        # overlays' offsets start AFTER the eeprom-preload region by
        # design (they're placed at p3_code_offset += eeprom-preload
        # size at the point eeprom slots are reserved).
        ee_overlay_names = {name for _, name, _, _, _ in ee_overlays} if has_eeprom else set()
        native_p3_offsets = [off for _, name, _, _, off in p3_overlays
                             if name not in ee_overlay_names]
        if native_p3_offsets:
            first_native_p3_offset = min(native_p3_offsets)
            assembled.append('\tsection page3')
            assembled.append(f'\torg {first_native_p3_offset}')

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
            oled_init_info = getattr(self, '_oled_init_data', None)
            if oled_init_info is not None:
                oled_base, oled_bytes = oled_init_info
                emit_allocated_until(oled_base)
                assembled.append(f'; OLED (SSD1327) init data at data[{oled_base}..{oled_base+len(oled_bytes)-1}]')
                for b in oled_bytes:
                    assembled.append(f'\tbyte {b}')
                    emitted += 1
                self._oled_init_already_emitted = True
            for tbl_base, table, ch, xv, yv in getattr(self, '_oled_putc_tables', []):
                emit_allocated_until(tbl_base)
                assembled.append(f'; oled_putc {ch!r} at ({xv},{yv}) table @ data[{tbl_base}..{tbl_base+len(table)-1}]')
                for b in table:
                    assembled.append(f'\tbyte {b}')
                    emitted += 1
            for buf_base, buf, s, xv, yv in getattr(self, '_oled_text_at_bufs', []):
                emit_allocated_until(buf_base)
                assembled.append(f'; oled_text_at({xv},{yv},{s!r}) idx buf @ data[{buf_base}..{buf_base+len(buf)-1}]')
                for b in buf:
                    assembled.append(f'\tbyte {b}')
                    emitted += 1
            if hasattr(self, '_oled_lut_base'):
                emit_allocated_until(self._oled_lut_base)
                assembled.append(f'; OLED 4bpp expansion LUTs at data[{self._oled_lut_base}..{self._oled_lut_base+95}] '
                                 f'(3 × 32 entries: byte0/byte1/byte2)')
                for b in self._oled_lut_data:
                    assembled.append(f'\tbyte {b}')
                    emitted += 1
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

        # Now that the p2 overlay bytes have been emitted (at stack
        # offsets 0..N matching the manifest's offset claims), emit the
        # manifest+pages metadata at P2_MANIFEST_BASE (=0xC0=192). Pass
        # 2's `org 192` pads stack[N..191] with 0x00, then the manifest
        # bytes write at 192-197. Final stack_size = ~198 — the firmware
        # uploads all 198 bytes to MK1's stack page, both overlays and
        # manifest land at their correct offsets.
        assembled.extend(manifest_directives)

        # EEPROM-backed overlay code: stored in EEPROM at ee_overlay_base.
        # At init time, the self_copy preload loop reads these bytes back
        # into page3. MK1 is standalone — no ESP32 runtime role.
        # The ESP32 writes er.eeprom (which this section appends to) to the
        # AT24C32 EEPROM during upload; after that the ESP32 has no role.
        if ee_overlays:
            # Pad from start of EEPROM buffer to ee_overlay_base. The
            # C++ assembler's `section eeprom` write pointer starts at
            # 0, NOT at 0x10. If a prior `section eeprom` already emitted
            # the 16-byte header (when self.eeprom_data is non-empty),
            # skip it; otherwise emit zeros from 0 to cover the header
            # region too. Without this, ee_overlay_base=0x100 became
            # eeprom[0xF0] in the buffer and the runtime preload (which
            # reads address 0x100 from the AT24C32) got bytes 16-bytes
            # into the first overlay — out_imm 0xD1 looked like jal
            # 0xAC. Bug existed since the inline preload landed.
            if self.eeprom_data:
                pad_start = self.eeprom_alloc  # header + globals already emitted
            else:
                pad_start = 0  # nothing emitted yet — start from buffer offset 0
            pad_count = ee_overlay_base - pad_start
            if pad_count > 0:
                assembled.append('\tsection eeprom')
                assembled.append(f'; pad {pad_count}B from 0x{pad_start:04X} → 0x{ee_overlay_base:04X}')
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

            # Order matters: emit `j __init_after_mc` BEFORE pre-mc
            # helpers, not after. Two reasons:
            # (1) The CPU falls through from the mini-copy loop's
            #     `jnz` — without an unconditional jump first, it
            #     would execute the helpers' first instructions.
            # (2) If the j were emitted after the helpers (whose
            #     bodies end with `ret`), the peephole's dead-code-
            #     elim pass (which drops anything between a
            #     terminator and the next label) would strip it.
            #     Pre-fix symptom: lcd_init runs in init phase, but
            #     after the mini-copy loop the CPU falls into
            #     __i2c_st_only's bytes, then __i2c_sp's, then
            #     pops a return address from the stack — pure UB.
            _cur_addr = _measure_code_section(assembled)
            if _cur_addr < _mc_end:
                _pad_needed = _mc_end - _cur_addr
                print(f"  Init-code placement: jumping over mini-copy target "
                      f"({_cur_addr}->{_mc_end}) before init code",
                      file=sys.stderr)
                assembled.append('\tj __init_after_mc')

            if _pre_mc_helpers:
                print(f"  Init-helper pre-mini-copy placement: {len(_pre_mc_helpers)} "
                      f"helper(s) ({sum(h[3] for h in _pre_mc_helpers)}B) fit in "
                      f"{_pre_mc_budget}B slack before mini-copy target — saves "
                      f"equivalent bytes of padding", file=sys.stderr)
                for name, start, end, fsize in _pre_mc_helpers:
                    body = _rename_shared_refs(self.code[start:end])
                    assembled.extend(body)

            # Pad up to mc_end so __init_after_mc lands at the
            # correct address (just past the mini-copy target).
            # Always emit the label — j __init_after_mc above needs it.
            _cur_addr = _measure_code_section(assembled)
            if _cur_addr < _mc_end:
                _pad_needed = _mc_end - _cur_addr
                assembled.append('__init_mc_pad:')
                for _ in range(_pad_needed):
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
                # Ensure I²C send-byte helper is available — required for the
                # `--eeprom` mode preload, and a useful default for any
                # program that calls i2c_init() (most do go on to send
                # bytes via __i2c_sb directly or via __lcd_chr / __i2c_st_only
                # which transitively call __i2c_sb).
                #
                # __i2c_rb (read byte) is NOT added here — it's only
                # needed if the program actually reads from an I²C device
                # at runtime (RTC reads, EEPROM array reads, i2c_read_byte
                # builtin, eeprom_read_byte builtin). All those builtins
                # add __i2c_rb themselves, so adding it here was dead
                # weight (~38 B resident) for the many write-only programs
                # (LCD-only, OLED-only, tone+keypad, etc.). The dead-code
                # eliminator at line ~3030 was also patched in the same
                # spirit.
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
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
                # ORB latch = 0 (will drive PB low when DDRB later goes
                # output for I2C). Shadow OR'd so any other PB-claimed
                # bits (e.g. a future keypad row pull-down) keep their
                # latch state. For programs without other ORB claims,
                # shadow=0 → byte-identical to raw `exw 0 0`.
                _orb_init = self._port_shadow_mask('ORB')
                if _orb_init != 0:
                    self.emit(f'\tldi $a,0x{_orb_init:02X}')  # +1B per other ORB user
                self.emit('\texw 0 0')         # ORB = A (shadow preserved)
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

            if name == 'keypad_init':
                # 4x4 matrix keypad on PA4-7 (rows, output) + PB4-7
                # (cols, input with external 8K pull-ups to VCC).
                # Hardware-validated 2026-04-26 with `keypad_smoke.c` and
                # `keypad_scan_test.c`; per-key map captured in worklog.
                #
                # Phase 1: register pin claims so any other emitter
                # (tone, lcd, i2c) that goes through emit_ddra / emit_ora
                # automatically preserves these bits. No scan helper
                # yet — `keypad_scan()` builtin is Phase 2.
                #
                # Why each claim:
                #   DDRA bits 4-7 = 1   rows must stay outputs
                #   ORA  bits 4-7 = 1   rows IDLE state is HIGH (pulled
                #                       low only during the active row's
                #                       scan slot, which the scan helper
                #                       will manage explicitly in Phase 2)
                # Cols (PB4-7) are inputs — DDR bit = 0. The current
                # claim model only tracks always-output bits, so we
                # can't "claim PB4-7 as input" via the registry. We
                # ASSERT no other device has claimed DDRB[4-7] as
                # output instead — if a future device does, this
                # raises at compile time. Symmetric assertion on
                # ORB[4-7] would be redundant (an input pin's ORB
                # latch is don't-care since it doesn't drive).
                ddrb_existing = self._port_shadow_mask('DDRB')
                if ddrb_existing & 0xF0:
                    raise Exception(
                        "keypad_init(): DDRB bits 4-7 (column input "
                        f"pins PB4-7) are claimed as output by another "
                        f"device (DDRB shadow=0x{ddrb_existing:02X}, "
                        f"conflict=0x{ddrb_existing & 0xF0:02X}). "
                        "Keypad columns need PB4-7 as inputs."
                    )
                self._claim_port_bits('DDRA', 0xF0, 'keypad')
                self._claim_port_bits('ORA',  0xF0, 'keypad')
                # Mark for downstream: the scan helper (Phase 2) will
                # check this flag rather than rebuilding the inference.
                self._keypad_init_called = True
                # Emit runtime port setup at the call site. emit_ddra /
                # emit_ora apply the now-updated shadow mask, so the
                # actual byte we write is `(0xF0 | other_devices_bits)`
                # — preserves PA1 (tone) etc. if those have also been
                # claimed already.
                self.emit_ddra(0xF0, comment='keypad rows PA4-7 output')
                self.emit_ora(0xF0,  comment='keypad rows idle HIGH')
                # DDRB / ORB intentionally untouched. PB4-7 default to
                # input (DDR=0) post-reset and stay that way unless
                # something else claims them; the conflict check above
                # rejects that. I2C's dynamic DDRB toggling (0x03/0x01/
                # 0x00) only touches bits 0-1 so col bits stay at 0.
                return

            if name == 'keypad_scan':
                # Phase 2 — scan helper returns 0-15 (key index, where
                # row 0 col 0 = 0, row 3 col 3 = 15) or 0xFF if no key
                # is pressed. Body emitted in `_emit_i2c_helpers`.
                # Caller must have called `keypad_init()` first;
                # without it the port shadow mask wouldn't include
                # the keypad row claims and the scan would emit ORA
                # writes that fail to preserve row direction.
                if not getattr(self, '_keypad_init_called', False):
                    raise Exception(
                        "keypad_scan() called without prior keypad_init(). "
                        "keypad_init() registers the PA4-7 / ORA shadow "
                        "claims that the scan helper relies on; without "
                        "it the scan would emit incorrect ORA values.")
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__keypad_scan')
                self.emit('\tjal __keypad_scan')
                return

            if name == 'keypad_wait':
                # Phase 3 — block until a key is pressed and released,
                # return the 0-15 key index. Two-sample debounce ~5 ms
                # apart via the calibrated `__delay_Nms`. Requires
                # `keypad_init()` (port claims) and an eventual
                # `delay_calibrate()` (we set `_needs_delay_calibrate`
                # to auto-insert one if absent — same pattern as
                # `delay()`).
                if not getattr(self, '_keypad_init_called', False):
                    raise Exception(
                        "keypad_wait() called without prior keypad_init(). "
                        "Call keypad_init() before any keypad_wait()/keypad_scan().")
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__keypad_wait')
                self._lcd_helpers.add('__keypad_scan')   # called from body
                self._lcd_helpers.add('__delay_Nms')     # debounce
                self._needs_delay_calibrate = True
                self.emit('\tjal __keypad_wait')
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

            if name in ('oled_init', 'oled_fill', 'oled_clear'):
                # SSD1327 128x128 4bpp OLED driver — Phase 2.
                #
                # Three builtins share an init walker (`__oled_init`,
                # one per program) and per-K specialized fill helpers
                # (`__oled_fill_<K>`, one per distinct fill byte):
                #
                #   oled_init()    — emit init transaction (DISPLAYOFF,
                #                    config, CMDLOCK, DISPLAYON,
                #                    contrast, full-screen window).
                #                    Idempotent: subsequent calls are
                #                    no-ops.
                #   oled_fill(K)   — fill GDDRAM with constant byte K.
                #                    Lazy-inits if oled_init() wasn't
                #                    called explicitly. Multiple
                #                    distinct K values per program are
                #                    allowed; each gets its own ~80B
                #                    helper.
                #   oled_clear()   — convenience for oled_fill(0x00).
                #
                # Sentinel choice for the init walker: SSD1327 uses
                # 0xFD/FE/FC as real commands, so we use 0xF0..0xF3 for
                # END/START/STOP/DELAY (none appear in init payload).
                if name == 'oled_init':
                    if len(args) != 0:
                        raise Exception(
                            f"oled_init() takes no arguments, got {len(args)}")
                    self._oled_ensure_init()
                    return

                if name == 'oled_fill':
                    if len(args) != 1:
                        raise Exception(
                            f"oled_fill(byte) takes exactly 1 argument, got {len(args)}")
                    fill_val = self._const_eval(args[0])
                    if fill_val is None:
                        raise Exception(
                            "oled_fill() argument must be a compile-time constant")
                    fill_val &= 0xFF
                else:  # oled_clear
                    if len(args) != 0:
                        raise Exception(
                            f"oled_clear() takes no arguments, got {len(args)}")
                    fill_val = 0x00

                self._oled_ensure_init()
                if not hasattr(self, '_oled_fill_ks'):
                    self._oled_fill_ks = set()
                self._oled_fill_ks.add(fill_val)
                helper_name = f'__oled_fill_{fill_val:02X}'
                self._lcd_helpers.add(helper_name)
                # Tag the call site as init-time vs runtime so the
                # partitioner can decide whether the helper goes to
                # stage-1 init code (frees ~28 B from the runtime kernel)
                # or stays kernel-resident. Heuristic:
                #   - Inside any loop (`_break_label` set): runtime.
                #   - In a non-main user function: runtime (callable repeatedly).
                #   - In main with no loop active: init-time.
                # Conservative: defaults to runtime on doubt. Misses
                # "init-only function called once from main's init
                # prefix" but covers the dominant case (oled_clear()
                # called directly in main's init prefix).
                if not hasattr(self, '_oled_fill_runtime_use'):
                    self._oled_fill_runtime_use = set()
                in_loop = getattr(self, '_break_label', None) is not None
                in_non_main = (getattr(self, 'current_func', None) != 'main')
                if in_loop or in_non_main:
                    self._oled_fill_runtime_use.add(fill_val)
                self.emit(f'\tjal {helper_name}')
                return

            if name == 'oled_putc':
                # Render a single 5x5 glyph at cell (x, y). All three
                # arguments must be compile-time constants. The
                # builtin allocates a 27-byte table in the data page
                # (6 window bytes + 21 4bpp pixel bytes) and emits a
                # short call to the shared `__oled_putc_xfer` runtime
                # helper which streams the table across two I²C
                # transactions (window-set + data-stream).
                #
                # Cost: ~5B per call site + 27B/call data + ~80B for
                # the shared helper. ~21× smaller per-call than
                # inlining all 32 byte-sends, so multi-letter strings
                # actually fit page 0.
                #
                # Cell grid: 21 cols × 18 rows on a 128×128 OLED
                # (6 px wide × 7 px tall cells). Cursor state and
                # auto-advance are deferred to Phase 4b.
                if len(args) != 3:
                    raise Exception(
                        f"oled_putc(char, x, y) takes 3 arguments, got {len(args)}")
                cv = self._const_eval(args[0])
                xv = self._const_eval(args[1])
                yv = self._const_eval(args[2])
                if cv is None or xv is None or yv is None:
                    raise Exception(
                        "oled_putc() arguments must all be compile-time constants")
                self._emit_oled_putc(chr(cv & 0xFF), xv, yv)
                return

            if name == 'oled_set_cursor':
                # Phase 4a: compile-time-only cursor. Updates the
                # codegen's tracked cursor position (`_oled_cursor_x`,
                # `_oled_cursor_y`) so that subsequent `oled_text(...)`
                # calls render starting from the new cell. Both args
                # must be compile-time constants. Emits no code.
                if len(args) != 2:
                    raise Exception(
                        f"oled_set_cursor(x, y) takes 2 arguments, got {len(args)}")
                xv = self._const_eval(args[0])
                yv = self._const_eval(args[1])
                if xv is None or yv is None:
                    raise Exception(
                        "oled_set_cursor() arguments must be compile-time constants")
                if not (0 <= xv <= 20):
                    raise Exception(
                        f"oled_set_cursor() x must be in [0, 20], got {xv}")
                if not (0 <= yv <= 17):
                    raise Exception(
                        f"oled_set_cursor() y must be in [0, 17], got {yv}")
                self._oled_cursor_x = xv
                self._oled_cursor_y = yv
                return

            if name == 'oled_text':
                # Phase 4a: render a string literal at the current
                # compiler-tracked cursor, advancing the cursor per
                # char (wraps at col 21 → next row, row 18 → row 0).
                # Each char compiles to one `oled_putc(c, x, y)`
                # equivalent at known (x, y) — same code-size cost
                # as a direct sequence of putc calls.
                #
                # The cursor lives only on the codegen object during
                # compilation, so two `oled_text()` calls in the same
                # program continue from where the previous one left
                # off. Cross-function cursor flow is sound because
                # the cursor is updated at *emit time*, not call
                # time — calling order in the source is what counts.
                #
                # Phase 4b will replace this with a runtime cursor in
                # kstate to support `oled_printf` (where digit count
                # from `%d` varies at runtime).
                if len(args) != 1 or args[0][0] != 'string':
                    raise Exception(
                        "oled_text() requires a string literal argument")
                s = args[0][1]
                cur_x = getattr(self, '_oled_cursor_x', 0)
                cur_y = getattr(self, '_oled_cursor_y', 0)
                for ch in s:
                    if ch not in self._OLED_FONT_5X5:
                        raise Exception(
                            f"oled_text(): char {ch!r} (0x{ord(ch):02X}) is not "
                            f"in the embedded 5x5 font. Supported: "
                            f"{''.join(sorted(self._OLED_FONT_5X5.keys()))}")
                    if cur_x > 20:
                        cur_x = 0
                        cur_y += 1
                    if cur_y > 17:
                        cur_y = 0
                    self._emit_oled_putc(ch, cur_x, cur_y)
                    cur_x += 1
                self._oled_cursor_x = cur_x
                self._oled_cursor_y = cur_y
                return

            if name == 'oled_text_at':
                # Phase 4b' streaming text: render a string literal
                # at cell (x, y) in ONE I²C transaction. All three
                # args must be compile-time constants.
                #
                # Compile-time work:
                #   - Validate (x, y, x+len-1) lies within the cell
                #     grid (21 cols × 18 rows).
                #   - Translate each char to its font index, allocate
                #     a buffer of indices in page 1.
                #   - Emit window-set transaction inline (the cell
                #     window covers exactly N×3 col-units wide × 7 px
                #     tall; the chip auto-increments inside it).
                #   - Emit `ldi $d, buf; ldi $c, N; jal __oled_stream
                #     _chars` to pump expanded glyph data.
                #
                # Per-call cost: ~22 B inline (window-set) + N B page-1
                # buffer (one byte per char). For a 21-char line:
                # 22 + 21 = 43 B. For comparison, Phase 4a was 21 ×
                # 27 = 567 B per 21-char line.
                if len(args) != 3:
                    raise Exception(
                        f"oled_text_at(x, y, str) takes 3 arguments, got {len(args)}")
                xv = self._const_eval(args[0])
                yv = self._const_eval(args[1])
                if xv is None or yv is None or args[2][0] != 'string':
                    raise Exception(
                        "oled_text_at(x, y, str) — x, y must be compile-time "
                        "constants and str must be a string literal")
                s = args[2][1]
                if len(s) == 0:
                    return  # zero-length text is a no-op
                if not (0 <= xv <= 20):
                    raise Exception(
                        f"oled_text_at() x must be in [0, 20], got {xv}")
                if not (0 <= yv <= 24):
                    raise Exception(
                        f"oled_text_at() y must be in [0, 24], got {yv}")
                if xv + len(s) > 21:
                    raise Exception(
                        f"oled_text_at(): {len(s)}-char string at x={xv} "
                        f"would extend past col 20 (would end at col "
                        f"{xv + len(s) - 1}). Wrap onto another line "
                        f"or shorten the string.")
                self._oled_ensure_init()
                self._oled_ensure_font_p3()
                self._oled_ensure_lut()
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__oled_stream_chars')
                self._lcd_helpers.add('__oled_set_window')
                # Translate chars to font indices and allocate page-1 buf
                buf = []
                for ch in s:
                    if ch not in self._oled_font_index_map:
                        raise Exception(
                            f"oled_text_at(): char {ch!r} (0x{ord(ch):02X}) "
                            f"is not in the embedded 5x5 font. Supported: "
                            f"{''.join(sorted(self._OLED_FONT_5X5.keys()))}")
                    buf.append(self._oled_font_index_map[ch])
                buf_base = self.data_alloc
                self.data_alloc += len(buf)
                if not hasattr(self, '_oled_text_at_bufs'):
                    self._oled_text_at_bufs = []
                self._oled_text_at_bufs.append((buf_base, buf, s, xv, yv))
                # Window-set via shared sub-helper. Cells are 6 wide × 5
                # tall (no descender / line-gap rows), giving a 21 × 25
                # cell grid. Two adjacent rows abut directly; insert
                # blank space by skipping a y value.
                col_s = xv * 3
                col_e = col_s + 3 * len(s) - 1   # inclusive
                row_s = yv * 5
                row_e = row_s + 4
                # `clr $X` (= sub $X,$X) is 1 B; `ldi $X, 0` is 2 B.
                # Save a byte per zero arg at call sites that land at
                # the origin.
                if col_s == 0:
                    self.emit('\tclr $a')
                else:
                    self.emit(f'\tldi $a,{col_s}')
                self.emit(f'\tldi $b,{col_e}')
                if row_s == 0:
                    self.emit('\tsub $c,$c')        # zero $c (clr only handles $a)
                else:
                    self.emit(f'\tldi $c,{row_s}')
                self.emit(f'\tldi $d,{row_e}')
                self.emit('\tjal __oled_set_window')
                # Stream chars
                self.emit(f'\tldi $d,{buf_base}')
                self.emit(f'\tldi $c,{len(s)}')
                self.emit('\tjal __oled_stream_chars')
                return

            if name == 'oled_print_buf':
                # Runtime-data streaming text: render `count` chars
                # from the page-1 byte array `buf` at cell (x, y).
                # Each byte in buf must be a font index (0..N-1)
                # produced by user code at runtime (e.g., a keymap
                # lookup, a sprintf-into-buf, a menu's selected
                # entry). The buffer can be updated between calls.
                #
                # Args: oled_print_buf(buf, count, x, y) where
                #   buf    — name of a `unsigned char NAME[N]`
                #            page-1 array (compile-time-known address).
                #   count  — compile-time const, 1..21.
                #   x, y   — compile-time const cell coords.
                #
                # Per call site: ~10 instructions inline. No new
                # helpers — reuses __oled_set_window and __oled_
                # stream_chars. The hot path is identical to
                # oled_text_at; only the buffer source differs
                # (runtime user array vs compile-time string).
                if len(args) != 4:
                    raise Exception(
                        f"oled_print_buf(buf, count, x, y) takes 4 arguments, got {len(args)}")
                if args[0][0] != 'var':
                    raise Exception(
                        "oled_print_buf(): first arg must be a page-1 array variable name")
                buf_name = args[0][1]
                if buf_name not in self.globals:
                    if buf_name in self.page3_globals:
                        raise Exception(
                            f"oled_print_buf(): buffer '{buf_name}' is in page 3; "
                            f"only page-1 arrays are supported (the streaming helper "
                            f"reads via `deref`, not `derefp3`)")
                    raise Exception(
                        f"oled_print_buf(): unknown array '{buf_name}'")
                buf_addr = self.globals[buf_name]
                nchars = self._const_eval(args[1])
                xv = self._const_eval(args[2])
                yv = self._const_eval(args[3])
                if nchars is None or xv is None or yv is None:
                    raise Exception(
                        "oled_print_buf(): count, x, y must be compile-time constants")
                if not (1 <= nchars <= 21):
                    raise Exception(
                        f"oled_print_buf(): count must be in [1, 21], got {nchars}")
                if not (0 <= xv <= 20):
                    raise Exception(
                        f"oled_print_buf(): x must be in [0, 20], got {xv}")
                if not (0 <= yv <= 24):
                    raise Exception(
                        f"oled_print_buf(): y must be in [0, 24], got {yv}")
                if xv + nchars > 21:
                    raise Exception(
                        f"oled_print_buf(): {nchars} chars at x={xv} would "
                        f"extend past col 20")
                self._oled_ensure_font_p3()
                self._oled_ensure_lut()
                self._oled_ensure_init()
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__oled_stream_chars')
                self._lcd_helpers.add('__oled_set_window')
                # Window-set
                col_s = xv * 3
                col_e = col_s + 3 * nchars - 1
                row_s = yv * 5
                row_e = row_s + 4
                if col_s == 0:
                    self.emit('\tclr $a')
                else:
                    self.emit(f'\tldi $a,{col_s}')
                self.emit(f'\tldi $b,{col_e}')
                if row_s == 0:
                    self.emit('\tsub $c,$c')
                else:
                    self.emit(f'\tldi $c,{row_s}')
                self.emit(f'\tldi $d,{row_e}')
                self.emit('\tjal __oled_set_window')
                # Stream from runtime buf
                self.emit(f'\tldi $d,{buf_addr}')
                self.emit(f'\tldi $c,{nchars}')
                self.emit('\tjal __oled_stream_chars')
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

            if name == 'i2c_send_const':
                # Send a compile-time-constant byte over I²C, fully
                # unrolled at the call site. ~3.4× faster than
                # i2c_send_byte() (the per-bit test/shift/branch chain
                # in __i2c_sb is folded at emit time) but ~10× larger:
                # ~52B per call vs ~5B for `ldi $a,K; jal __i2c_sb`.
                # Use only in hot loops where the bus byte is known at
                # compile time (bulk fills, EEPROM clears, etc.). For
                # one-shot byte sends, prefer i2c_send_byte().
                if len(args) != 1:
                    raise Exception(
                        f"i2c_send_const(byte) takes exactly 1 argument, got {len(args)}")
                val = self._const_eval(args[0])
                if val is None:
                    raise Exception(
                        "i2c_send_const() argument must be a compile-time constant")
                self.emit_unrolled_i2c_send_const(val)
                return

            if name == 'i2c_fill_const':
                # Send a compile-time-constant byte over I²C `count`
                # times in a tight asm loop using the unrolled body.
                # Counts in [1, 256] use a single inner loop; counts
                # > 256 must be a multiple of 256 and use an outer/
                # inner pair (max 65536). Both arguments must be
                # compile-time constants. Backs the OLED fill helper
                # and is intended for any bulk-write context (EEPROM
                # erase, framebuffer clear, byte stream of known
                # length).
                if len(args) != 2:
                    raise Exception(
                        f"i2c_fill_const(byte, count) takes 2 arguments, got {len(args)}")
                val = self._const_eval(args[0])
                count = self._const_eval(args[1])
                if val is None:
                    raise Exception(
                        "i2c_fill_const() byte argument must be a compile-time constant")
                if count is None:
                    raise Exception(
                        "i2c_fill_const() count argument must be a compile-time constant")
                self.emit_unrolled_i2c_fill_const(val, count)
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
                # Uses STOP + START between register-pointer write and
                # data read — matching the proven sequence in main.cpp's
                # readDS3231Temp() for maximum hardware reliability.
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self._lcd_helpers.add('__i2c_sp')
                self._lcd_helpers.add('__i2c_st_only')
                # Set register pointer: START + 0xD0 + 0x00
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xD0')
                self.emit('\tjal __i2c_sb')
                self.emit('\tclr $a')
                self.emit('\tjal __i2c_sb')
                # STOP + START (no RS) + 0xD1 + read byte + NACK + STOP
                self.emit('\tjal __i2c_sp')
                self.emit('\tjal __i2c_st_only')
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
                # Uses REPEATED-START (jal __i2c_rs) between register-pointer
                # write and data read — same as rtc_read_seconds and the new
                # rtc_read_reg. STOP+START would release the bus, and the
                # DS3231 loses the register pointer on STOP, so a subsequent
                # START would return whatever the auto-incremented internal
                # pointer points at (typically register 0/1, giving wildly
                # wrong "temperature" values like 8 or 11 instead of 24).
                # Empirically: STOP+START version produced wrong values when
                # called after lcd_init (which leaves the DS3231 in a
                # different internal state than i2c_init alone).
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

            if name == 'rtc_read_reg':
                # Read DS3231 register N (passed in args[0]). Generic
                # parameterized version of rtc_read_seconds (reg=0x00) and
                # rtc_read_temp (reg=0x11). Lowers to a callable shared
                # `__rtc_read_reg` helper rather than inlining the I2C
                # transaction per call site — for programs that read multiple
                # registers (overlay_dashboard.c reads 4: hours, minutes,
                # seconds, temp) this saves 3× the bytes vs inlining.
                if not args:
                    self.gen_error("rtc_read_reg() requires 1 argument: register byte")
                    return
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__rtc_read_reg')
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self._lcd_helpers.add('__i2c_rs')
                # Pass reg byte in $a — helper preserves it through the
                # device-address write and uses it as the register pointer.
                c = self._const_eval(args[0])
                if c is not None:
                    self.emit(f'\tldi $a,{c & 0xFF}')
                else:
                    self.gen_expr(args[0])
                self.emit('\tjal __rtc_read_reg')
                return

            if name == 'i2c_ack':
                # Send ACK (pull SDA LOW, clock SCL once).
                # 4-step pattern matching __eeprom_r2c_loop: drop SCL LOW
                # *before* releasing SDA so the SDA rise happens with SCL
                # already LOW. The shorter 3-step pattern (0x01→0x02 in
                # one VIA write) relies on RC timing (SCL falling faster
                # than SDA rising via pullup) to avoid glitching a STOP
                # condition; AT24C512 sequential reads were dropping after
                # byte 1 with the 3-step form. +1 ddrb is worth bus safety.
                self.emit_ddrb(0x03)   # SDA LOW, SCL LOW
                self.emit_ddrb(0x01)   # SDA LOW, SCL HIGH
                self.emit_ddrb(0x03)   # SDA LOW, SCL LOW (SCL falls first)
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

            if name == 'ee64_write_byte':
                # ee64_write_byte(addr16, data) — write one byte to AT24C512
                # at I²C address 0x50 (write byte 0xA0). 16-bit memory addr.
                # Hardware-test scaffolding for the cold-tier mechanism;
                # production cold-loading uses the runtime dispatcher
                # (Phase 4) which emits the same shape directly. ACK-polls
                # the write cycle so no __delay_cal dependency.
                if not args or len(args) < 2:
                    self.gen_error("ee64_write_byte(addr16, data)")
                    return
                addr_c = self._const_eval(args[0])
                if addr_c is None:
                    self.gen_error("ee64_write_byte: address must be a constant")
                    return
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                addr_hi = (addr_c >> 8) & 0xFF
                addr_lo = addr_c & 0xFF
                # START + dev write + addr_hi + addr_lo + data + STOP
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xA0')
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $a,{addr_hi}')
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $a,{addr_lo}')
                self.emit('\tjal __i2c_sb')
                c = self._const_eval(args[1])
                if c is not None:
                    self.emit(f'\tldi $a,{c & 0xFF}')
                else:
                    self.gen_expr(args[1])
                self.emit('\tjal __i2c_sb')
                self.emit_ddrb(0x03)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x00)
                # ACK poll until write cycle completes (chip NACKs while busy)
                lbl = self.label('e64wp')
                self.emit(f'{lbl}:')
                self.emit('\texrw 2')
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xA0')
                self.emit('\tjal __i2c_sb')
                self.emit('\tpush $a')
                self.emit_ddrb(0x03)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x00)
                self.emit('\tpop $a')
                self.emit('\ttst 0x01')
                self.emit(f'\tjnz {lbl}')
                return

            if name == 'cold_call':
                # cold_call(id) — invoke a cold-tier helper by id.
                # Loads the helper from AT24C512 into the runtime cold
                # slot via __cold_call, jumps in. Helper metadata
                # (ee64 addr + length) must already be in
                # page3[COLD_TABLE_BASE + id*3] (3 bytes per entry).
                # Phase 4 scaffolding: caller is responsible for seeding
                # the metadata (typically via poke3). Phase 5 wires the
                # partitioner to populate it automatically and to keep
                # cold helpers' bytes synchronized with the EEPROM image.
                if not args:
                    self.gen_error("cold_call(id)")
                    return
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self._lcd_helpers.add('__cold_call')
                self._lcd_helpers.add('__cold_load_helper')
                # Pass id in $b so caller's $a is preserved (the cold
                # helper expects its first arg in $a). `push_imm`
                # would clobber $a as a side effect.
                c = self._const_eval(args[0])
                if c is not None:
                    self.emit(f'\tldi $b,{c & 0xFF} ;!keep')
                else:
                    self.gen_expr(args[0])
                    self.emit('\tmov $a,$b')
                self.emit('\tjal __cold_call')
                return

            if name == 'ee64_read_to_code':
                # ee64_read_to_code(ee_addr16, code_addr, count) — bulk
                # sequential I2C read from AT24C512 → code page via istc.
                # Mirrors eeprom_read_to_code but targets the 64KB chip
                # (device 0xA0/0xA1, full 16-bit addressing). Reuses the
                # chip-agnostic __eeprom_r2c_loop for the read loop body.
                if len(args) < 3:
                    self.gen_error("ee64_read_to_code(ee_addr, code_addr, count)")
                ee_addr = self._const_eval(args[0])
                code_addr = self._const_eval(args[1])
                count = self._const_eval(args[2])
                if ee_addr is None or code_addr is None or count is None:
                    self.gen_error("ee64_read_to_code: all args must be constants")
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self._lcd_helpers.add('__eeprom_r2c_loop')
                # START + dev write + addr_hi + addr_lo + REP-START + dev read
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xA0')
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $a,{(ee_addr >> 8) & 0xFF}')
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $a,{ee_addr & 0xFF}')
                self.emit('\tjal __i2c_sb')
                self.emit_ddrb(0x00)
                self.emit_ddrb(0x01)
                self.emit_ddrb(0x03)
                self.emit('\tldi $a,0xA1')
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $b,{code_addr & 0xFF}')
                self.emit(f'\tldi $a,{count & 0xFF}')
                self.emit('\tpush $a')
                self.emit('\tjal __eeprom_r2c_loop')
                self.emit('\tpop $a')
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

    # ── Pass: elide `j X` immediately followed by `X:` ───────────────
    # The merge step in _overlay_partition strips this pattern for
    # bundleable helpers, but the equivalent at kernel/init level
    # (e.g. __lcd_chr falling into __lcd_send_raw) survives because
    # peephole sees a label between the two and historically had no
    # rule for label-adjacency. 1B per occurrence; ~5 in the corpus
    # at the kernel/flat level.
    _dead_j_saved = 0
    new_lines = []
    i = 0
    while i < len(lines):
        cur = lines[i].strip()
        if cur.startswith('j ') and not cur.startswith('jal') and \
           not any(cur.startswith(jt + ' ') for jt in ('jnz', 'jz', 'jc', 'jnc')):
            target = cur.split(None, 1)[1] if len(cur.split()) > 1 else ''
            # Look ahead past blank/comment lines for the next label.
            k = i + 1
            while k < len(lines):
                nxt = lines[k].strip()
                if not nxt or nxt.startswith(';'):
                    k += 1
                    continue
                break
            if k < len(lines) and lines[k].strip() == f'{target}:':
                _dead_j_saved += 1
                i += 1
                continue
        new_lines.append(lines[i])
        i += 1
    lines = new_lines
    if _dead_j_saved:
        import sys
        print(f"  Peephole: elided {_dead_j_saved} dead `j` jumps "
              f"to immediate next label (saved {_dead_j_saved}B)",
              file=sys.stderr)

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

    def _prepare_compile(prefer_delay_cal_init=False, capture_log=False,
                         auto_cold_fns=None, force_cold_helpers=None,
                         extract_main_body=False, extract_main_chunks=1):
        gen = MK1CodeGen(optimize=args.optimize)
        # Stash the in-flight gen so the exception handler can recover
        # `_matcher_fires` even on compile failure (populated during
        # _simplify_function_bodies, before _overlay_partition can throw).
        _prepare_compile._last_gen = gen
        gen._prefer_delay_cal_init = prefer_delay_cal_init
        if auto_cold_fns:
            gen._auto_cold_user_fns = set(auto_cold_fns)
        if force_cold_helpers:
            gen._force_cold_helpers = set(force_cold_helpers)
        if extract_main_body:
            gen._extract_main_body = True
            gen._extract_main_body_chunks = max(1, int(extract_main_chunks))
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
            elif 'section ee64' in s: section = 'ee64'; continue
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

    # Phase 3 skinny-kernel mode: when the program uses cold tier, helpers
    # currently classified as init-only (`__lcd_init`, `__oled_init`,
    # `__delay_cal`, …) can be promoted to the cold tier. The dispatcher
    # rewriter rewrites their call sites whether the caller lives in init
    # or runtime, and `_extract_force_cold_helpers` runs BEFORE
    # `_overlay_partition` so the partitioner sees a smaller program —
    # often small enough to skip init_extraction entirely and compile flat.
    #
    # Listed here so the auto-cold loop can seed them when a program
    # overflows AFTER Phase 2.5 has exhausted its kernel-section
    # candidates. Keep narrow: only helpers whose body is leaf-style
    # (ends in ret) and whose every call site is reachable through the
    # dispatcher contract. Adding speculative entries that don't actually
    # exist in the code is harmless — `_extract_force_cold_helpers`
    # silently skips missing names — but adding entries that DO exist
    # but tail-jump (no `ret`) costs a wasted compile pass.
    PHASE3_INIT_BUILTINS = (
        '__lcd_init',
        '__oled_init',
        '__delay_cal',
        '__rtc_read_reg',
        '__lcd_send_raw',
    )

    def _phase3_pick_init_builtin(g, asm_lines, already_helper):
        """Find a known init builtin that's still in the kernel image and
        not already forced to cold. Used when Phase 2.5 has exhausted
        normal kernel-section candidates but the program is still over
        budget and init_extraction is active.

        Walk asm_lines looking for top-level labels whose name matches
        PHASE3_INIT_BUILTINS. Return the first match that's not already
        a force-cold target. None if no candidate found."""
        present = set()
        for line in asm_lines:
            s = line.strip()
            if (s.endswith(':') and not s.startswith('.')
                    and not s.startswith(';')):
                name = s[:-1]
                if name in PHASE3_INIT_BUILTINS:
                    present.add(name)
        for name in PHASE3_INIT_BUILTINS:
            if name in present and name not in already_helper:
                return name
        return None

    def _phase25_compute_kernel_bytes(g, asm_lines):
        """Tally bytes in the kernel section of the assembled output.
        Mirrors `_phase25_pick_overflow_helper`'s walk so the auto-cold
        loop's overflow check matches what the picker would see."""
        init_extr = getattr(g, '_init_extraction_done', False)
        overlay_mode = hasattr(g, '_overlay_kernel_size')
        two_byte = {'ldsp', 'ldsp_b', 'stsp', 'push_imm', 'jal',
                    'jc', 'jz', 'jnc', 'jnz', 'j', 'ldi', 'cmp',
                    'addi', 'subi', 'andi', 'ori', 'ld', 'st',
                    'ldp3', 'stp3', 'ocall', 'tst', 'out_imm',
                    'cmpi', 'ddrb_imm', 'ddra_imm', 'ora_imm',
                    'orb_imm'}

        def _is_kernel_section(s):
            if init_extr or overlay_mode:
                return ('section page3_kernel' in s
                        or 'section page3_code' in s)
            return ('section code' in s
                    and 'data_code' not in s
                    and 'stack_code' not in s
                    and 'page3_code' not in s)

        # Default starting section: 'kernel' for flat mode, 'init' for
        # init-extraction / overlay (because the implicit pre-marker
        # bytes are stage-1 init code there).
        in_kernel = (not init_extr and not overlay_mode)
        kernel_bytes = 0
        for line in asm_lines:
            s = line.strip()
            if 'section ' in s or s.strip().startswith('org '):
                in_kernel = _is_kernel_section(s)
                continue
            if not in_kernel:
                continue
            if not s or s.endswith(':') or s.startswith(';'):
                continue
            mn = s.split()[0]
            if mn == 'byte':
                kernel_bytes += 1
            elif mn == 'cmp':
                parts = s.split()
                kernel_bytes += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
            elif mn == 'ddrb2_imm':
                kernel_bytes += 3
            elif mn == 'ddrb3_imm':
                kernel_bytes += 4
            elif mn in two_byte:
                kernel_bytes += 2
            else:
                kernel_bytes += 1
        return kernel_bytes

    def _phase25_pick_overflow_helper(g, asm_lines, already_helper, already_user):
        """Phase 2.5: detect post-compile runtime kernel overflow and pick
        the next-largest helper to auto-promote to cold tier.

        Returns the helper name (e.g. `__rtc_read_reg`) or `None`.
        `None` means either no overflow OR no eligible candidate.

        Eligibility:
          - Helper lives in `section page3_kernel` (init_extraction /
            overlay modes) or `section code` (flat mode).
          - Body ends in `ret` (leaf-style; same constraint
            `_extract_force_cold_helpers` enforces).
          - Body fits in COLD_SLOT_BYTES.
          - Not already in `already_helper` (this iteration) or
            represented as a single-underscore user fn already in
            `already_user`.
          - Net byte effect on the runtime kernel must be positive
            (body bytes saved > 2 × kernel-side call sites bloated
            by `ldi $b,id ;!keep`).
          - Promotion must not push the stage-1 init code past its
            254 B budget (each init-side call site adds +2 B from
            the same rewriter).

        Picker bias: largest net runtime savings first. Ties broken
        by larger body, then name.
        """
        cold_slot = getattr(g, 'cold_slot_size', 0) or 0
        max_code = (256 - cold_slot) if cold_slot > 0 else 250
        init_extr = getattr(g, '_init_extraction_done', False)
        overlay_mode = hasattr(g, '_overlay_kernel_size')

        # Walk lines: tracked sections + per-helper byte tally. Match
        # _measure_asm_bytes' two-byte set so the picker's size estimate
        # lines up with what the partitioner would measure post-extract.
        two_byte = {'ldsp', 'ldsp_b', 'stsp', 'push_imm', 'jal',
                    'jc', 'jz', 'jnc', 'jnz', 'j', 'ldi', 'cmp',
                    'addi', 'subi', 'andi', 'ori', 'ld', 'st',
                    'ldp3', 'stp3', 'ocall', 'tst', 'out_imm',
                    'cmpi', 'ddrb_imm', 'ddra_imm', 'ora_imm',
                    'orb_imm'}

        def _is_kernel_section(s):
            if init_extr or overlay_mode:
                return ('section page3_kernel' in s
                        or 'section page3_code' in s)
            return ('section code' in s
                    and 'data_code' not in s
                    and 'stack_code' not in s
                    and 'page3_code' not in s)

        # Walk every line in the asm: track current section + (in
        # kernel) the current helper. ALSO scan call references per
        # section — dispatcher rewriting (`jal __X` → `ldi $b,id;
        # jal __cold_call`) adds +2 B at every call site, and we
        # need to know which sections those sites live in to model
        # the per-stage byte impact (kernel = page3_kernel; init =
        # `code`; everything else is bookkeeping).
        # Asm defaults to `section code` before any explicit
        # `section ...` marker — assembler convention. In flat mode
        # the whole code section IS the kernel (no separate init
        # stage); in init-extraction / overlay modes the default
        # `code` section holds stage-1 init code and the runtime
        # kernel lives in `section page3_kernel`. Set both
        # `cur_section` and `in_kernel` consistently with that.
        if init_extr or overlay_mode:
            cur_section = 'init'
            in_kernel = False
        else:
            cur_section = 'kernel'
            in_kernel = True
        kernel_bytes = 0
        init_bytes = 0
        helper_lines = {}     # name → list of body lines (no label)
        kernel_calls = {}     # name → kernel-section call count
        init_calls = {}       # name → init-section call count
        other_calls = {}      # name → other-section call count
        cur_helper = None
        def _is_init_section(s):
            return ('section code' in s
                    and 'data_code' not in s
                    and 'stack_code' not in s
                    and 'page3_code' not in s
                    and 'page3_kernel' not in s)
        for line in asm_lines:
            s = line.strip()
            if 'section ' in s or s.strip().startswith('org '):
                in_kernel = _is_kernel_section(s)
                if in_kernel:
                    cur_section = 'kernel'
                elif _is_init_section(s):
                    cur_section = 'init'
                else:
                    cur_section = 'other'
                cur_helper = None
                continue
            # Count call-site references per section. The dispatcher
            # rewriter visits both kernel `self.code` and already-
            # extracted cold function bodies (via `_rewrite` in
            # `_extract_force_cold_helpers`), and rewrites both
            # `jal __X` and `j __X`.
            for prefix in ('jal __', 'j __'):
                if s.startswith(prefix):
                    tgt = s[len(prefix) - 2:].split()[0].split(';')[0].strip()
                    if tgt.startswith('__'):
                        bucket = (kernel_calls if cur_section == 'kernel'
                                  else init_calls if cur_section == 'init'
                                  else other_calls)
                        bucket[tgt] = bucket.get(tgt, 0) + 1
                    break
            # Tally init-section bytes for the post-promotion init
            # overflow check.
            if cur_section == 'init' and s and not s.endswith(':') and not s.startswith(';'):
                mn = s.split()[0]
                if mn == 'byte':
                    init_bytes += 1
                elif mn == 'cmp':
                    parts = s.split()
                    init_bytes += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
                elif mn == 'ddrb2_imm':
                    init_bytes += 3
                elif mn == 'ddrb3_imm':
                    init_bytes += 4
                elif mn in two_byte:
                    init_bytes += 2
                else:
                    init_bytes += 1
            if not in_kernel:
                continue
            # Top-level label? (no leading dot, ends with `:`).
            if (s.endswith(':') and not s.startswith('.')
                    and not s.startswith(';')):
                name = s[:-1]
                # Helpers begin with `__` (kernel) or `_` (user fn).
                # Pick `__` only — user fns go through `_auto_cold_user_fns`
                # which is a separate path.
                if name.startswith('__'):
                    cur_helper = name
                    helper_lines[cur_helper] = []
                else:
                    cur_helper = None
                continue
            # Body line under cur_helper: count bytes + capture line for
            # leaf-check.
            if cur_helper is not None and not s.startswith(';'):
                helper_lines[cur_helper].append(s)
            # Tally kernel_bytes for the overflow check.
            if not s or s.startswith(';'):
                continue
            mn = s.split()[0]
            if mn == 'byte':
                kernel_bytes += 1
            elif mn == 'cmp':
                parts = s.split()
                kernel_bytes += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
            elif mn == 'ddrb2_imm':
                kernel_bytes += 3
            elif mn == 'ddrb3_imm':
                kernel_bytes += 4
            elif mn in two_byte:
                kernel_bytes += 2
            else:
                kernel_bytes += 1

        if kernel_bytes <= max_code:
            return None  # no overflow

        # Score candidates. Body bytes computed the same way the
        # extractor measures: count instruction sizes; ignore label.
        def _measure(body):
            n = 0
            for ln in body:
                if not ln or ln.startswith(';') or ln.endswith(':'):
                    continue
                mn = ln.split()[0]
                if mn == 'byte':
                    n += 1
                elif mn == 'cmp':
                    parts = ln.split()
                    n += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
                elif mn == 'ddrb2_imm':
                    n += 3
                elif mn == 'ddrb3_imm':
                    n += 4
                elif mn in two_byte:
                    n += 2
                else:
                    n += 1
            return n

        def _ends_in_ret(body):
            for ln in reversed(body):
                if not ln or ln.startswith(';') or ln.endswith(':'):
                    continue
                return ln.split()[0] == 'ret'
            return False

        # Dispatcher infrastructure: must remain kernel-resident.
        # `__cold_call` / `__cold_load_helper` ARE the cold-tier loader;
        # `__i2c_sb` / `__i2c_rb` are called from `__cold_load_helper`'s
        # bulk-read loop. Promoting any of them creates a chicken-egg
        # bootstrap failure (the dispatcher would have to load itself).
        DISPATCHER_INFRA = {
            '__cold_call', '__cold_load_helper',
            '__i2c_sb', '__i2c_rb',
        }
        # Stage-1 init budget. Init-extraction reserves 14 B for the
        # copy loop; the resident dispatcher infrastructure already
        # lives in init too. Total stage-1 must fit in 254 B (the
        # `init_limit_local` the auto-cold loop already polices for
        # the `_prefer_dc` retry path).
        INIT_LIMIT = 254
        candidates = []
        for name, body in helper_lines.items():
            if name in already_helper:
                continue
            if name in DISPATCHER_INFRA:
                continue
            sz = _measure(body)
            if sz == 0 or sz > COLD_SLOT_BYTES:
                continue
            if not _ends_in_ret(body):
                continue
            # Per-section call counts.
            kc = kernel_calls.get(name, 0)
            ic = init_calls.get(name, 0)
            # Kernel net savings: body removed (-sz) + calls grow
            # by 2 B each (+2*kc). Net = sz - 2*kc.
            kernel_net = sz - 2 * kc
            if kernel_net <= 0:
                # Promotion would not actually shrink the kernel —
                # body savings cancelled by call-site bloat. Skip.
                continue
            # Init-side bloat: each init call grows by 2 B. Skip if
            # this would push stage-1 past its budget.
            init_after = init_bytes + 2 * ic
            if init_after > INIT_LIMIT:
                continue
            # Score: maximize kernel net savings, prefer larger
            # bodies on ties (frees more contiguous space), name
            # tiebreaker.
            candidates.append((kernel_net, sz, name))
        if not candidates:
            return None
        candidates.sort(key=lambda t: (-t[0], -t[1], t[2]))
        return candidates[0][2]

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

    # Outer auto-cold retry: when overlay placement reports that a USER
    # function or kernel HELPER would wrap into kernel, the partitioner
    # has tried every placement option and there is no SRAM slot big
    # enough. Promote the wrapping symbol to the cold tier and
    # recompile. User functions go through `_auto_cold_user_fns`
    # (Phase 1, treated as synthetic `ee64`). Helpers go through
    # `_force_cold_helpers` (Phase 2, body extracted post-emission).
    # Iterates until compile succeeds or no further auto-promotion is
    # possible (cap to avoid runaway loops).
    import re as _re_ac
    import os as _env_ac
    _auto_cold = set()        # user fns
    _force_helpers = set()    # kernel helpers (start with __)
    # Test/debug seed: env vars let regression fixtures force the
    # auto-cold path deterministically rather than relying on overlay-
    # wrap to trigger it. Names are comma-separated bare identifiers.
    _seed_user = _env_ac.environ.get('MK1_FORCE_COLD_USER', '').strip()
    if _seed_user:
        for _n in _seed_user.split(','):
            _n = _n.strip()
            if _n:
                _auto_cold.add(_n)
    _seed_helper = _env_ac.environ.get('MK1_FORCE_COLD_HELPER', '').strip()
    if _seed_helper:
        for _n in _seed_helper.split(','):
            _n = _n.strip()
            if _n:
                # Helpers are referenced as `__name`; canonicalize so the
                # extractor's label match works whether the user wrote
                # `__name` or `name`.
                if not _n.startswith('__'):
                    _n = '__' + _n.lstrip('_')
                _force_helpers.add(_n)
    _MAX_AUTO_COLD = 16
    _prefer_dc = True
    _extract_main_body_tried = False
    _extract_main_body_disabled = False
    _extract_main_chunks = 1
    _MAX_EXTRACT_CHUNKS = 4
    while True:
        try:
            gen, asm, lines, _code_bytes_try, _init_limit_try, _compile_log = _prepare_compile(
                prefer_delay_cal_init=_prefer_dc, capture_log=True,
                auto_cold_fns=_auto_cold,
                force_cold_helpers=_force_helpers,
                extract_main_body=_extract_main_body_tried,
                extract_main_chunks=_extract_main_chunks)
            # Init-extract overflow: redo with delay_cal as overlay.
            if (_code_bytes_try > _init_limit_try and _prefer_dc):
                import sys as _retry_sys
                print(f"  Placement retry: init-only __delay_cal overflowed stage 1 "
                      f"({_code_bytes_try}B > {_init_limit_try}B); "
                      f"using runtime/overlay placement",
                      file=_retry_sys.stderr)
                _prefer_dc = False
                continue
            # Phase 2.5: post-compile runtime kernel overflow.
            # `wrap into kernel` only fires during overlay partitioning
            # — once auto-cold has carried the program past wrap, the
            # remaining kernel can still be too big for the runtime
            # code page (256 - cold_slot_size). Detect that here and
            # promote the next-largest non-cold leaf helper.
            _overflow_pick = _phase25_pick_overflow_helper(
                gen, lines, _force_helpers, _auto_cold)
            if _overflow_pick is not None and (
                    len(_auto_cold) + len(_force_helpers) < _MAX_AUTO_COLD):
                import sys as _ov_sys
                print(f"  Auto-cold (overflow): promoting helper "
                      f"{_overflow_pick} to cold tier "
                      f"(post-compile kernel overflow)",
                      file=_ov_sys.stderr)
                _force_helpers.add(_overflow_pick)
                continue
            # Phase 3 skinny-kernel: Phase 2.5 found no kernel-section
            # candidate, but the program may still overflow AND have
            # init-classified helpers (`__lcd_init`, etc.) that aren't
            # in the partitioner's runtime-helper pool. Seed one and
            # retry — the dispatcher rewriter will rewrite both init
            # and runtime call sites, and `_extract_force_cold_helpers`
            # runs ahead of `_overlay_partition` so the partitioner
            # sees the post-promotion code (often small enough to skip
            # init_extraction entirely → flat mode → dispatcher always
            # reachable from init).
            #
            # Only fires when cold tier is already active (force-helpers
            # or auto-cold has touched this program) — we don't want to
            # spontaneously enable cold tier on programs that fit
            # comfortably in their existing mode.
            _cold_active = bool(_force_helpers or _auto_cold)
            # Direct overflow check: walk asm `lines` and tally kernel-
            # section bytes vs `256 - cold_slot_size` (or 250 in flat
            # mode without a cold slot). The compile_log captures only
            # stderr emitted DURING gen.compile(), but the
            # `!! CODE OVERFLOW` warning prints after the auto-cold
            # loop finishes — so checking for it in _compile_log
            # always misses. This direct check fires the same way
            # the final report does.
            _kernel_bytes = _phase25_compute_kernel_bytes(gen, lines)
            _max_code = (256 - getattr(gen, 'cold_slot_size', 0)) \
                if getattr(gen, 'cold_slot_size', 0) > 0 else 250
            _overflow = (_kernel_bytes > _max_code
                         or _code_bytes_try > _init_limit_try)
            if (_cold_active and _overflow
                    and len(_auto_cold) + len(_force_helpers) < _MAX_AUTO_COLD):
                _p3_pick = _phase3_pick_init_builtin(
                    gen, lines, _force_helpers)
                if _p3_pick is not None:
                    import sys as _p3_sys
                    print(f"  Auto-cold (skinny-kernel): promoting init "
                          f"builtin {_p3_pick} to cold tier",
                          file=_p3_sys.stderr)
                    _force_helpers.add(_p3_pick)
                    continue
            # Phase 4: main-body extraction. Last-resort auto-cold —
            # when Phase 2.5 / 3 have exhausted helper candidates and
            # the program still overflows, wrap main's body in a
            # synthetic ee64 function and re-compile. The synthetic
            # gets cold-promoted by the standard ee64 pathway, main
            # shrinks to a one-line shim, and the kernel-page budget
            # gains back the bytes that were main's body.
            #
            # Only fires when cold tier is already active (auto-cold
            # has run) and the program is still overflowing — i.e.
            # this is the SECOND wave of wins after helper promotion
            # plateaus. Doesn't fire spontaneously for programs that
            # fit cleanly without it.
            if (_cold_active and _overflow
                    and not _extract_main_body_disabled):
                if not _extract_main_body_tried:
                    import sys as _p4_sys
                    print(f"  Auto-cold (main-body): wrapping main's body "
                          f"in synthetic _synth_main_body cold helper",
                          file=_p4_sys.stderr)
                    _extract_main_body_tried = True
                    continue
                # Already tried single-chunk; the runtime kernel still
                # overflows but the synthetic itself fit (no slot-
                # overflow exception). Splitting reduces the largest
                # cold helper, shrinks the slot, grows the runtime
                # budget — BUT ONLY if the synthetic IS the largest.
                # If something else (e.g. __i2c_stream, __delay_cal)
                # dominates the slot, splitting just adds main-side
                # cold_call overhead without shrinking the slot, and
                # makes overflow worse. Skip the split in that case.
                if _extract_main_chunks < _MAX_EXTRACT_CHUNKS:
                    _cold_helpers = getattr(gen, 'cold_helpers', [])
                    _largest = max(
                        _cold_helpers,
                        key=lambda h: h[2],
                        default=None)
                    _largest_is_synth = (
                        _largest is not None
                        and 'synth_main_body' in _largest[3])
                    if _largest_is_synth:
                        _extract_main_chunks += 1
                        import sys as _p4i_sys
                        print(f"  Auto-cold (main-body): still overflowing; "
                              f"splitting synthetic into "
                              f"{_extract_main_chunks} chunks",
                              file=_p4i_sys.stderr)
                        continue
            break
        except Exception as _e:
            _msg = str(_e)
            _total = len(_auto_cold) + len(_force_helpers)
            # Phase 4 may produce a synthetic _synth_main_body that
            # exceeds the cold slot. Disable the extraction and
            # re-raise on next iteration if needed; the program
            # falls back to its pre-extraction overflow report.
            if (_extract_main_body_tried
                    and "'_synth_main_body" in _msg
                    and 'exceeds slot' in _msg):
                # Slot overflow on the synthetic. If there's room to
                # split further, bump the chunk count and retry —
                # multiple smaller chunks may individually fit even
                # when one big one didn't. Cap at _MAX_EXTRACT_CHUNKS
                # so a fundamentally-too-big main eventually surfaces
                # as the original overflow report instead of cycling.
                if _extract_main_chunks < _MAX_EXTRACT_CHUNKS:
                    _extract_main_chunks += 1
                    import sys as _p4s_sys
                    print(f"  Auto-cold (main-body): synthetic body "
                          f"too large for slot; retrying with "
                          f"{_extract_main_chunks}-chunk split",
                          file=_p4s_sys.stderr)
                    continue
                import sys as _p4r_sys
                print(f"  Auto-cold (main-body): synthetic body too "
                      f"large for slot even at {_MAX_EXTRACT_CHUNKS} "
                      f"chunks; reverting to no-extraction compile",
                      file=_p4r_sys.stderr)
                _extract_main_body_tried = False
                _extract_main_body_disabled = True
                _extract_main_chunks = 1
                continue
            if 'wrap into kernel' not in _msg or _total >= _MAX_AUTO_COLD:
                _emit_partial_metrics(getattr(_prepare_compile, '_last_gen', None), _msg)
                raise
            # Parse `wrapping=_name=46B[, _other=NB,...]`. Single-
            # underscore names are user fns; double-underscore are
            # kernel helpers; skip phase-splits (`_name_pN`).
            _wraps = _re_ac.findall(r'(_+[A-Za-z][\w]*)=\d+B', _msg)
            _picked_user = None
            _picked_helper = None
            for _w in _wraps:
                if _re_ac.search(r'_p\d+$', _w):
                    continue
                if _w.startswith('__'):
                    if _w in _force_helpers:
                        continue
                    if _picked_helper is None:
                        _picked_helper = _w
                else:
                    _user = _w[1:]
                    if _user in _auto_cold:
                        continue
                    if _picked_user is None:
                        _picked_user = _user
            # Prefer user fns first (cheaper, Phase 1 path) before
            # falling back to helper extraction.
            import sys as _ac_sys
            if _picked_user is not None:
                print(f"  Auto-cold: promoting {_picked_user} to cold tier "
                      f"(overlay placement would wrap)", file=_ac_sys.stderr)
                _auto_cold.add(_picked_user)
            elif _picked_helper is not None:
                print(f"  Auto-cold: promoting helper {_picked_helper} to "
                      f"cold tier (kernel doesn't fit)", file=_ac_sys.stderr)
                _force_helpers.add(_picked_helper)
            else:
                _emit_partial_metrics(getattr(_prepare_compile, '_last_gen', None), _msg)
                raise
    if _compile_log:
        import sys as _compile_log_sys
        print(_compile_log, end='', file=_compile_log_sys.stderr)

    if args.output:
        with open(args.output, 'w') as f: f.write(asm + '\n')
    else:
        print(asm)

    # Page & EEPROM utilisation report
    import sys
    # Cold-tier reserves [cold_slot_base..255] in the code page for the
    # runtime dispatcher to load helpers into. Program code must not
    # extend into that range — otherwise the dispatcher's istc_inc
    # writes corrupt program bytes and execution falls into garbage
    # after the cold helper returns. Narrow MAX_CODE accordingly.
    cold_slot_size = getattr(gen, 'cold_slot_size', 0) or 0
    if cold_slot_size > 0:
        MAX_CODE = 256 - cold_slot_size
    else:
        MAX_CODE = 250  # safe code page limit (6 B reserved for halt-trap)
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
        elif 'section ee64' in s: section = 'ee64'; continue
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

    ee64_used = getattr(gen, 'ee64_alloc', 0) if gen is not None else 0
    cold_count = len(getattr(gen, 'cold_helpers', []) if gen is not None else [])
    if ee64_used > 0:
        metrics['ee64_used'] = ee64_used
        metrics['ee64_free'] = 65536 - ee64_used
        metrics['cold_helpers'] = cold_count
        print(f"  EE64:       {ee64_used}B used, {65536 - ee64_used}B free  "
              f"(AT24C512, {cold_count} cold helper{'s' if cold_count != 1 else ''})",
              file=sys.stderr)

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
