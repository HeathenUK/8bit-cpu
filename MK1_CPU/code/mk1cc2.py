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

import sys, re, argparse

# ── Tokenizer (from mk1cc.py) ────────────────────────────────────────

TOKEN_PATTERNS = [
    ('MCOMMENT', r'/\*[\s\S]*?\*/'),
    ('COMMENT',  r'//[^\n]*'),
    ('CHAR',     r"'(?:[^'\\]|\\[nrt0\\'])'"),  # character literals
    ('NUMBER',   r'0[xX][0-9a-fA-F]+|0[bB][01]+|\d+'),
    ('STRING',   r'"(?:[^"\\]|\\.)*"'),
    ('IDENT',    r'[a-zA-Z_]\w*'),
    ('OP2',      r'&&|\|\||[+\-&|^]=|==|!=|<=|>=|<<|>>|\+\+|\-\-'),
    ('OP1',      r'[+\-*/%&|^~!<>=(){},;\[\]?:]'),
    ('SKIP',     r'[ \t]+'),
    ('NEWLINE',  r'\n'),
    ('MISMATCH', r'.'),
]
KEYWORDS = {'if', 'else', 'while', 'for', 'do', 'return', 'unsigned', 'char',
            'void', 'int', 'switch', 'case', 'default', 'break', 'continue',
            'u8', 'u16', 'eeprom'}

class Token:
    def __init__(self, type, value, line):
        self.type, self.value, self.line = type, value, line

def tokenize(source):
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
                else:
                    self.expect(';')
                    if size is None:
                        raise SyntaxError(f'Line {self.peek().line}: array {name} needs size or initializer')
                    globals_.append((name, [0]*size, storage))
            elif self.match(';'):
                globals_.append((name, 0, storage))
            else: raise SyntaxError(f'Line {self.peek().line}: unexpected after {name}')
        return globals_, functions

    def parse_type(self):
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
        if t.value in ('unsigned', 'char', 'int', 'u8', 'u16'): return self.parse_local_decl()
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
            return ('local_arr', name, size, init_vals)
        init = self.parse_expr() if self.match('=') else None
        self.expect(';'); return ('local', name, init, typ)

    def parse_expr(self): return self.parse_assign()
    def parse_assign(self):
        left = self.parse_ternary()
        if self.peek().value in ('=','+=','-=','&=','|=','^='):
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
        if t.type == 'NUMBER': self.advance(); return ('num', t.value)
        if t.type == 'STRING':
            self.advance()
            # Strip quotes and decode escapes
            s = t.value[1:-1]
            s = s.replace('\\n', '\n').replace('\\r', '\r').replace('\\t', '\t')
            s = s.replace('\\0', '\0').replace('\\\\', '\\')
            return ('string', s)
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

    def label(self, prefix='L'):
        self.label_id += 1
        return f'.{prefix}{self.label_id}'

    def emit(self, line):
        self.code.append(line)
        # Invalidate B cache on B-modifying instructions
        s = line.strip()
        if (',$b' in s and s.startswith('mov')) or s.startswith('pop_b') or s == 'swap' or s.startswith('jal'):
            self.b_expr = None

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

        # Compile main first (starts at address 0, no j _main jump needed)
        # Then compile other functions after main
        main_fn = None
        other_fns = []
        for fn in functions:
            if fn[0] == 'main':
                main_fn = fn
            else:
                other_fns.append(fn)

        if main_fn:
            self.compile_function(*main_fn)

        for fn in other_fns:
            self.compile_function(*fn)

        # Emit I2C/LCD helpers AFTER all functions
        self._emit_i2c_helpers()

        # Emit tone init code (ddra + precomp loop) as prefix to _main.
        # Must be AFTER all tone()/silence() calls are compiled (so note table is complete)
        # and BEFORE dead function elimination.
        if (getattr(self, '_needs_tone_init', False) and
            hasattr(self, '_note_table') and self._note_table and
            not getattr(self, 'eeprom_mode', False)):
            # Non-overlay mode: insert precomp loop into main.
            # (In overlay mode, the precomp loop is in the init section instead.)
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

        # Dead function elimination
        self._eliminate_dead_functions()

        # Classify helpers (detect runtime I2C, build dynamic _NO_OVERLAY)
        self._classify_helpers()

        # Two-stage boot overlay partitioning
        self._overlay_partition()

        # Emit page 1 globals (skip if init extraction handled everything)
        page1_vars = [(n, i) for n, i, s in globals_ if n in self.globals]
        if page1_vars and not getattr(self, '_init_extraction_done', False):
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

        # LCD init data is in EEPROM — emitted via eeprom_data section

        # Emit lcd_print string data in page 3
        if hasattr(self, '_lcd_print_strings') and not getattr(self, '_init_extraction_done', False):
            self.emit('\tsection page3')
            for p3_off, s in self._lcd_print_strings:
                self.emit(f'; string at page3 offset {p3_off}')
                for ch in s:
                    self.emit(f'\tbyte {ord(ch)}')
                self.emit('\tbyte 0')  # null terminator
            self.emit('\tsection code')

        # Emit note table data in page 1 (data page)
        if hasattr(self, '_note_table'):
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
        # Note: EEPROM I2C helpers only kept alive when EEPROM tier is active
        # (detected by presence of __eeprom_load in code, not just eeprom_mode flag)

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

    def _classify_helpers(self):
        """Classify helpers into resident vs overlay-eligible categories.
        Sets self._needs_runtime_i2c and self._dynamic_no_overlay."""
        helpers = getattr(self, '_lcd_helpers', set())

        # Runtime I2C: needed if program uses LCD output or EEPROM reads at runtime
        # (not just init-time operations like delay_cal or lcd_init)
        runtime_i2c_markers = {'__lcd_chr', '__lcd_cmd', '__lcd_send', '__lcd_print',
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
                    no_ov.add('__lcd_send:')  # shared by chr and cmd

        # I2C byte-level helpers: resident if called from runtime code.
        # __i2c_sb and __i2c_rb are called from overlays + resident LCD code.
        # __i2c_st and __i2c_sp are init-only: __lcd_chr inlines the START,
        # and STOP is inlined in __lcd_send. Only the init sentinel loop
        # calls them as standalone functions.
        if '__i2c_sb' in helpers:
            no_ov.add('__i2c_sb:')
        if '__i2c_rb' in helpers:
            no_ov.add('__i2c_rb:')
        # __eeprom_rd: resident only if user code accesses eeprom arrays at runtime
        if '__eeprom_rd' in helpers:
            if getattr(self, '_needs_runtime_eeprom_rd', False):
                no_ov.add('__eeprom_rd:')
            # else: init-only (LCD init), will be handled by init extraction
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
        # ALL helpers in _NO_OVERLAY are conditionally resident — the knapsack
        # + deferred classification (step 8b) decides which to keep resident vs
        # bundle. This handles LCD, I2C, tone, delay, and any future helpers.
        self._conditionally_resident = set()
        for label in no_ov:
            name = label.rstrip(':')
            if name.startswith('__'):
                self._conditionally_resident.add(name)

    def _emit_i2c_helpers(self):
        """Emit I2C/LCD helper subroutines if any lcd_cmd/lcd_char builtins were used."""
        helpers = getattr(self, '_lcd_helpers', set())
        if not helpers:
            return

        # Always emit __i2c_sb (send byte, B=byte, C=counter) — shared by all helpers
        # Optimized: uses ddrb_imm (bus-safe, preserves A), merged tst+shift,
        # zero NOPs, no push/pop in ACK clock.
        self.emit('__i2c_sb:')
        self.emit('\tmov $a,$b')
        self.emit('\tldi $a,8')
        self.emit('\tmov $a,$c')
        lbl_s = self.label('isb')
        lbl_h = self.label('isbh')
        lbl_n = self.label('isbn')
        self.emit(f'{lbl_s}:')
        self.emit('\tmov $b,$a')
        self.emit('\ttst 0x80')
        self.emit(f'\tjnz {lbl_h}')
        self.emit('\tddrb_imm 0x03')
        self.emit('\tddrb_imm 0x01')
        self.emit('\tddrb_imm 0x03')
        self.emit(f'\tj {lbl_n}')
        self.emit(f'{lbl_h}:')
        self.emit('\tddrb_imm 0x02')
        self.emit('\tddrb_imm 0x00')
        self.emit('\tddrb_imm 0x02')
        self.emit(f'{lbl_n}:')
        self.emit('\tmov $b,$a')
        self.emit('\tsll')
        self.emit('\tmov $a,$b')
        self.emit('\tmov $c,$a')
        self.emit('\tdec')
        self.emit('\tmov $a,$c')
        self.emit(f'\tjnz {lbl_s}')
        # ACK clock: no NOPs needed (ddrb_imm has built-in settling),
        # no push/pop needed (ddrb_imm preserves A)
        self.emit('\tddrb_imm 0x02')   # SCL LOW, SDA released
        self.emit('\tddrb_imm 0x00')   # SCL HIGH (read ACK)
        self.emit('\texrw 0')          # A = port B (bit 0 = ACK)
        self.emit('\tddrb_imm 0x02')   # SCL LOW
        self.emit('\tret')

        # __i2c_st_only: just START (no address)
        if '__i2c_st_only' in helpers:
            self.emit('__i2c_st_only:')
            self.emit('\texrw 2')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x03')
            self.emit('\tret')

        # __i2c_st: START + send address 0x4E
        self.emit('__i2c_st:')
        self.emit('\texrw 2')
        self.emit('\tddrb_imm 0x01')
        self.emit('\tddrb_imm 0x03')
        self.emit('\tldi $a,0x4E')
        self.emit('\tjal __i2c_sb')
        self.emit('\tret')

        # __i2c_sp: STOP
        self.emit('__i2c_sp:')
        self.emit('\tddrb_imm 0x03')
        self.emit('\tddrb_imm 0x01')
        self.emit('\tddrb_imm 0x00')
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
            self.emit('\tddrb_imm 0x00')   # SCL HIGH
            self.emit('\texrw 0')          # A = port B
            self.emit('\ttst 0x01')        # test SDA
            self.emit(f'\tjz {lbl_rz}')
            self.emit('\tmov $b,$a')       # A = accumulated
            self.emit('\tori 0x01,$a')     # set bit 0
            self.emit('\tmov $a,$b')       # B = result
            self.emit(f'{lbl_rz}:')
            self.emit('\tddrb_imm 0x02')   # SCL LOW
            self.emit('\tmov $d,$a')       # A = counter
            self.emit('\tdec')
            self.emit('\tmov $a,$d')       # D--
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
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x03')
            self.emit('\tldi $a,0xAE')
            self.emit('\tjal __i2c_sb')
            # addr high
            self.emit('\tpop $a')
            self.emit('\tjal __i2c_sb')
            # addr low
            self.emit('\tmov $d,$a')
            self.emit('\tjal __i2c_sb')
            # STOP
            self.emit('\tddrb_imm 0x03')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x00')
            # START + device addr read
            self.emit('\texrw 2')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x03')
            self.emit('\tldi $a,0xAF')
            self.emit('\tjal __i2c_sb')
            # Read byte via __i2c_rb (D = byte, then A = byte)
            self.emit('\tjal __i2c_rb')
            self.emit('\tmov $d,$a')       # A = read byte
            # NACK + STOP
            self.emit('\tddrb_imm 0x00')
            self.emit('\tddrb_imm 0x02')
            self.emit('\tddrb_imm 0x03')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x00')
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
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x03')
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_ns}:')
            self.emit('\tldi $b,0xFD')
            self.emit('\tcmp $b')
            self.emit(f'\tjnz {lbl_nr}')
            # STOP
            self.emit('\tddrb_imm 0x03')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x00')
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
            self.emit('\tddrb_imm 0x00')    # SCL HIGH
            self.emit('\tnop')
            self.emit('\texrw 0')           # read port
            self.emit('\ttst 0x01')
            self.emit(f'\tjz {lbl_rz}')
            self.emit('\tmov $b,$a')
            self.emit('\tori 0x01,$a')
            self.emit('\tmov $a,$b')
            self.emit(f'{lbl_rz}:')
            self.emit('\tddrb_imm 0x02')    # SCL LOW
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
            # Data-driven LCD init: store I2C byte sequence in page 3,
            # use compact loop to send. Keeps overlay small (~35B vs ~85B).
            # Sentinels: 0xFE=START, 0xFD=STOP, 0xFF=END, else=send byte
            START, STOP, END, DELAY = 0xFE, 0xFD, 0xFF, 0xFC
            lcd_init_data = []
            # No PCF reset (0x00) — backlight defaults to on, reset turns it off
            BL = 0x08  # backlight bit — keep on throughout init
            # Reset nibble 0x03 × 3 with required delays between them
            # HD44780 spec: >4.1ms after 1st, >100µs after 2nd
            lcd_init_data += [START, 0x34|BL, 0x30|BL, STOP, DELAY]  # 1st + 5ms delay
            lcd_init_data += [START, 0x34|BL, 0x30|BL, STOP, DELAY]  # 2nd + 5ms delay
            lcd_init_data += [START, 0x34|BL, 0x30|BL, STOP]         # 3rd (no delay needed)
            lcd_init_data += [START, 0x24|BL, 0x20|BL, STOP]         # nibble 0x02 (4-bit)
            # HD44780 init: Function Set, Display Off, Clear, Entry Mode, Display On
            for cmd in [0x28, 0x0C, 0x01, 0x06]:
                hi = cmd & 0xF0
                lo = (cmd & 0x0F) << 4
                lcd_init_data += [START, hi|0x04|BL, hi|BL, lo|0x04|BL, lo|BL, STOP]
                if cmd == 0x01:
                    lcd_init_data.append(DELAY)  # clear display needs ~2ms
            lcd_init_data.append(END)

            # Store LCD init data in EEPROM
            eeprom_addr = self.eeprom_alloc
            self.eeprom_data.append((eeprom_addr, lcd_init_data))
            self.eeprom_alloc += len(lcd_init_data)
            data_base = self.data_alloc
            self.data_alloc += len(lcd_init_data)
            self._lcd_init_eeprom = (eeprom_addr, data_base, lcd_init_data)

            lbl_loop = self.label('li_lp')
            lbl_ns = self.label('li_ns')
            lbl_send = self.label('li_sd')
            lbl_adv = self.label('li_av')
            lbl_done = self.label('li_dn')

            # Read LCD init data from EEPROM using individual random reads
            # into data page, then process with sentinel loop.
            # NOTE: __eeprom_rd clobbers ALL registers (A,B,C,D).
            # Both countdown and data_ptr survive on the stack.
            # Stack layout: [count, ptr] with count on top.
            count = len(lcd_init_data)
            lbl_rd = self.label('li_rd')
            eeprom_lo = eeprom_addr & 0xFF
            eeprom_hi = (eeprom_addr >> 8) & 0xFF
            eeprom_offset = eeprom_lo - data_base
            self.emit('__lcd_init:')
            # Stack: just the data_ptr. Count in a fixed data page location.
            # Store count at data[255] (top of data page, won't conflict)
            self.emit(f'\tldi $a,{count}')
            self.emit('\tldi $b,255')
            self.emit('\tideref')             # data[255] = count
            self.emit(f'\tpush_imm {data_base}')   # ptr on stack
            self.emit(f'{lbl_rd}:')
            self.emit('\tpop $a')             # A = ptr
            self.emit('\tpush $a')            # re-save
            self.emit(f'\taddi {eeprom_offset},$a') if eeprom_offset else None
            self.emit(f'\tldi $b,{eeprom_hi}')
            self.emit('\tjal __eeprom_rd')    # A = byte
            self.emit('\tpop_b')             # B = ptr
            self.emit('\tideref')             # data[B] = A
            self.emit('\tmov $b,$a')          # A = ptr
            self.emit('\tinc')                # A = ptr+1
            self.emit('\tpush $a')            # save new ptr
            # Decrement count from data[255]
            self.emit('\tldi $a,255')
            self.emit('\tderef')              # A = data[255] = count
            self.emit('\tdec')
            self.emit('\tldi $b,255')
            self.emit('\tideref')             # data[255] = count-1
            self.emit(f'\tjnz {lbl_rd}')
            self.emit('\tpop $a')             # clean ptr from stack
            # Process from data page
            self.emit(f'\tldi $d,{data_base}')
            self.emit(f'{lbl_loop}:')
            self.emit('\tmov $d,$a')
            self.emit('\tderef')
            self.emit(f'\tldi $b,{END}')
            self.emit('\tcmp $b')
            self.emit(f'\tjz {lbl_done}')
            self.emit(f'\tldi $b,{START}')
            self.emit('\tcmp $b')
            self.emit(f'\tjnz {lbl_ns}')
            self.emit('\tjal __i2c_st')
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_ns}:')
            self.emit(f'\tldi $b,{STOP}')
            self.emit('\tcmp $b')
            lbl_nd = self.label('li_nd')
            self.emit(f'\tjnz {lbl_nd}')
            self.emit('\tjal __i2c_sp')
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_nd}:')
            self.emit(f'\tldi $b,{DELAY}')
            self.emit('\tcmp $b')
            self.emit(f'\tjnz {lbl_send}')
            # Calibrated ~5ms delay using ipm from page3[240]
            # Same inner loop as __delay_cal for accuracy.
            # Outer loop: 5 iterations × 1ms = 5ms
            self.emit('\tpush $d')         # save sentinel pointer
            lbl_outer = self.label('li_do')
            lbl_inner = self.label('li_di')
            self.emit(f'\tldi $c,5')       # 5ms outer count
            self.emit(f'{lbl_outer}:')
            self.emit('\tldi $a,240')
            self.emit('\tderefp3')         # A = page3[240] = ipm
            self.emit(f'{lbl_inner}:')
            self.emit('\tdec')
            self.emit(f'\tjnz {lbl_inner}')
            self.emit('\tmov $c,$a')
            self.emit('\tdec')
            self.emit('\tmov $a,$c')
            self.emit(f'\tjnz {lbl_outer}')
            self.emit('\tpop $d')          # restore sentinel pointer
            self.emit(f'\tj {lbl_adv}')
            self.emit(f'{lbl_send}:')
            self.emit('\tjal __i2c_sb')
            self.emit(f'{lbl_adv}:')
            self.emit('\tmov $d,$a')
            self.emit('\tinc')
            self.emit('\tmov $a,$d')
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
            self.emit('\tret')

        # Merged __lcd_cmd / __lcd_chr — differ only in flags (EN vs EN+RS+BL)
        if '__lcd_chr' in helpers and '__lcd_cmd' not in helpers:
            # Only lcd_chr used at runtime — merge chr+send into single function
            # Use only __lcd_chr label (no __lcd_send) to avoid stripper
            # treating them as separate functions
            self.emit('__lcd_chr:')
            self.emit('\tpush $a')        # save caller's A (will be discarded)
            self.emit('\tldi $a,0x09')    # flags: RS + BL (hardcoded)
        elif '__lcd_chr' in helpers:
            self.emit('__lcd_chr:')
            self.emit('\tpush $a')
            self.emit('\tldi $a,0x09')    # flags: RS + BL
            self.emit('\tj __lcd_send')
            if '__lcd_cmd' in helpers:
                self.emit('__lcd_cmd:')
                self.emit('\tpush $a')
                self.emit('\tldi $a,0x00')
            self.emit('__lcd_send:')
        elif '__lcd_cmd' in helpers:
            self.emit('__lcd_cmd:')
            self.emit('__lcd_send:')
            self.emit('\tpush $a')
            self.emit('\tldi $a,0x00')

        if '__lcd_cmd' in helpers or '__lcd_chr' in helpers:
            self.emit('\tpush $a')        # push flags to stack
            # Inline I2C START + PCF8574 addr (saves 4B vs jal __i2c_st)
            self.emit('\texrw 2')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x03')
            self.emit('\tldi $a,0x4E')
            self.emit('\tjal __i2c_sb')
            # High nibble: compute once, send with EN then without
            self.emit('\tmov $d,$a')      # A = char
            self.emit('\tandi 0xF0,$a')   # high nibble
            self.emit('\tpop_b')         # B = flags
            self.emit('\tpush $b')        # keep flags
            self.emit('\tor $b,$a')       # A = nibble | flags
            self.emit('\tpush $a')        # save nibble+flags
            self.emit('\tori 0x04,$a')    # + EN
            self.emit('\tjal __i2c_sb')   # send with EN
            self.emit('\tpop $a')         # restore nibble+flags (no EN)
            self.emit('\tjal __i2c_sb')   # send without EN
            # Low nibble: shift, compute once, send with EN then without
            self.emit('\tmov $d,$a')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tandi 0xF0,$a')
            self.emit('\tpop_b')         # flags
            self.emit('\tor $b,$a')
            self.emit('\tpush $a')        # save nibble+flags
            self.emit('\tori 0x04,$a')    # + EN
            self.emit('\tjal __i2c_sb')
            self.emit('\tpop $a')         # restore (no EN)
            self.emit('\tjal __i2c_sb')
            # Inline STOP — saves 3B vs jal __i2c_sp when __i2c_sp
            # is the only runtime caller and can be stripped from kernel
            self.emit('\tddrb_imm 0x03')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x00')
            self.emit('\tpop $a')         # balance push at entry (discards caller A)
            self.emit('\tret')

        if '__lcd_print' in helpers:
            # Print null-terminated string from page 3. A = page3 offset.
            lbl_lp = self.label('lp_lp')
            lbl_dn = self.label('lp_dn')
            self.emit('__lcd_print:')
            self.emit('\tmov $a,$d')         # D = pointer
            self.emit(f'{lbl_lp}:')
            self.emit('\tmov $d,$a')         # A = pointer
            self.emit('\tpush $a')           # save pointer
            self.emit('\tderefp3')           # A = page3[pointer]
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

        # __delay_cal: calibrate delay against SQW, store D/4 in data[0]
        # Single-phase: counts HIGH phase only (D_half), stores D_half/2 = D/4.
        # Saves ~23 bytes vs dual-phase counting.
        if '__delay_cal' in helpers:
            lbl_s1 = self.label('ds1')
            lbl_s2 = self.label('ds2')
            lbl_cal = self.label('dcal')
            lbl_chi = self.label('dchi')
            lbl_done = self.label('dcdn')
            self.emit('__delay_cal:')
            # Configure DS3231 SQW = 1Hz
            self.emit('\texrw 2')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x03')
            self.emit('\tldi $a,0xD0')
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x0E')
            self.emit('\tjal __i2c_sb')
            self.emit('\tclr $a')
            self.emit('\tjal __i2c_sb')
            self.emit('\tjal __i2c_sp')
            self.emit('\tclr $a')
            self.emit('\texw 0 3')         # clear DDRA before sync
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
            # Calibrate: count bare (dec+jnz) iterations during SQW HIGH.
            # A loops 256→0 (bare dec+jnz). B counts A-wraps (overflows).
            # Store B in data[0] = iterations per ~1ms at 500kHz.
            # delay(N) runs flat N×B iterations of the same dec+jnz.
            # Accuracy: <2% at 500kHz, ~30% at 250kHz (cal overhead mismatch).
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
            self.emit('\tmov $b,$a')       # A = B (ipm)
            self.emit('\tldi $b,240')
            self.emit('\tiderefp3')        # page3[240] = ipm
            # SQW left enabled — no coupling issue, saves ~20B
            self.emit('\tret')

        # __delay_Nms: B = delay count (ms). Reads ipm from data[0].
        # Computes total = N × ipm (16-bit), runs flat dec+jnz countdown.
        # Same inner loop as calibration. No per-ms overhead.
        # At 500kHz: ipm≈54, <2% error for any N from 1-255.
        # B = N on entry. Uses C for multiply accumulator.
        if '__delay_Nms' in helpers:
            lbl_mul = self.label('dmul')
            lbl_loop = self.label('dlp')
            lbl_done = self.label('ddn')
            self.emit('__delay_Nms:')
            # Multiply B × ipm: result in D:C (hi:lo)
            self.emit('\tldi $a,240')
            self.emit('\tderefp3')         # A = ipm from page3[240]
            self.emit('\tmov $a,$c')       # C = ipm
            self.emit('\tldi $d,0')        # D = result high byte
            self.emit('\tclr $a')          # A = result low byte
            # Add C to D:A, B times
            self.emit(f'{lbl_mul}:')
            self.emit('\tadd $c,$a')       # A += C (with carry)
            self.emit('\tjnc .noc%d' % self.label_id)
            self.emit('\tpush $a')
            self.emit('\tmov $d,$a')
            self.emit('\tinc')
            self.emit('\tmov $a,$d')
            self.emit('\tpop $a')
            self.emit('.noc%d:' % self.label_id)
            self.emit('\tmov $b,$a')       # check B
            self.emit('\tdec')
            self.emit('\tmov $a,$b')
            self.emit(f'\tjnz {lbl_mul}')
            # D:A = total iterations (D=high, A=low). Need B=high, A=low.
            self.emit('\tpush $a')         # save low byte
            self.emit('\tmov $d,$a')       # A = D = high byte
            self.emit('\tmov $a,$b')       # B = high byte
            self.emit('\tpop $a')          # A = low byte restored
            # 16-bit countdown: A counts down, B decrements on A wrap
            self.emit(f'{lbl_loop}:')
            self.emit('\tdec')
            self.emit(f'\tjnz {lbl_loop}')
            self.emit('\tmov $b,$a')       # A = B (high byte)
            self.emit('\ttst 0xFF')        # B == 0?
            self.emit(f'\tjz {lbl_done}')
            self.emit('\tdec')
            self.emit('\tmov $a,$b')
            self.emit('\tclr $a')
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
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
            self.emit('\tldi $a,240')
            self.emit('\tderefp3')         # A = ipm from page3[240]
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
            self.emit('\tadd $c,$a')       # A = (D<<4) + (A>>4) = half_period
            self.emit('\tmov $a,$c')       # C = half_period
            self.emit('\tret')

        # __tone: play square wave on PA1.
        # C = half_period (dec+jnz iterations per half-cycle).
        # D:B = 16-bit cycle count (D=high, B=low).
        # DDRA must already be 0x02.
        if '__tone' in helpers:
            lbl_loop = self.label('ttl')
            lbl_hi = self.label('thi')
            lbl_lo = self.label('tlo')
            lbl_done = self.label('tdn')
            self.emit('__tone:')
            self.emit('\tddra_imm 0x02')   # PA1 = output (only during tone)
            self.emit(f'{lbl_loop}:')
            self.emit('\tora_imm 0x02')    # PA1 HIGH
            self.emit('\tmov $c,$a')       # A = half_period
            self.emit(f'{lbl_hi}:')
            self.emit('\tdec')
            self.emit(f'\tjnz {lbl_hi}')
            self.emit('\tora_imm 0x00')    # PA1 LOW
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
            self.emit('\tdec')
            self.emit('\tmov $a,$d')       # D = D-1
            self.emit(f'\tj {lbl_loop}')
            self.emit(f'{lbl_done}:')
            self.emit('\tora_imm 0x00')    # PA1 LOW before disabling
            self.emit('\tddra_imm 0x00')   # PA1 = input (stop driving bus)
            self.emit('\tret')

        # __eeprom_r2c_loop: sequential I2C read → code page via istc.
        # Entry: B = code dest start addr. Count on stack (pushed by caller).
        # I2C must already be in sequential read mode.
        # B is saved/restored around __i2c_rb via stack.
        # Count tracked in data[7] (set by caller's push→pop into data[7]).
        if '__eeprom_r2c_loop' in helpers:
            lbl_loop = self.label('r2c')
            lbl_last = self.label('r2cl')
            P3_COUNT = 242   # overlay byte count (page 3 kernel state)
            self.emit('__eeprom_r2c_loop:')
            # Entry: B = code dest, stack = [ret_addr, count, ...]
            # Save B (dest), store count to page3, restore B.
            self.emit('\tpush_b')            # save dest
            self.emit('\tldsp 3')            # A = count (past: dest_pushed, ret_addr)
            self.emit(f'\tldi $b,{P3_COUNT}')
            self.emit('\tiderefp3')          # page3[242] = count
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
            self.emit(f'\tldi $a,{P3_COUNT}')
            self.emit('\tderefp3')           # A = remaining
            self.emit('\tdec')               # A = remaining-1, ZF set
            self.emit(f'\tldi $b,{P3_COUNT}')
            self.emit('\tiderefp3')          # page3[242] = remaining-1 (ZF preserved)
            self.emit('\tpop_b')             # B = dest+1 (ZF preserved: pop_b has no FI)
            self.emit(f'\tjz {lbl_last}')    # ZF from dec survives ldi+ideref
            # ACK
            self.emit('\tddrb_imm 0x03')     # SDA LOW, SCL LOW
            self.emit('\tddrb_imm 0x01')     # SDA LOW, SCL HIGH
            self.emit('\tddrb_imm 0x03')     # SCL LOW
            self.emit('\tddrb_imm 0x02')     # SDA released
            self.emit(f'\tj {lbl_loop}')
            # NACK + STOP (DDRB already 0x02 from __i2c_rb)
            self.emit(f'{lbl_last}:')
            self.emit('\tddrb_imm 0x00')     # SCL HIGH (NACK)
            self.emit('\tddrb_imm 0x02')     # SCL LOW
            self.emit('\tddrb_imm 0x03')     # SDA LOW
            self.emit('\tddrb_imm 0x01')     # SCL HIGH
            self.emit('\tddrb_imm 0x00')     # STOP
            self.emit('\tret')

        # __play_note: A = data page offset into note table.
        # Reads [half_period, cyc_lo, cyc_hi] from data page (page 1) via deref.
        # Note: ratio→half_period precomputation happens during init.
        # If half_period=0 and __delay_Nms available, plays silence (cyc_lo = ms).
        # Otherwise sets C=half_period and calls __tone.
        if '__play_note' in helpers:
            has_silence = '__delay_Nms' in helpers
            lbl_sil = self.label('pnsil') if has_silence else None
            self.emit('__play_note:')
            # Note table in page 1: [half_period, cyc_lo, cyc_hi]
            # (precomputed from ratio during init via __tone_setup)
            self.emit('\tmov $a,$d')       # D = base offset
            self.emit('\tderef')           # A = data[offset] = half_period
            if has_silence:
                self.emit('\ttst 0xFF')
                self.emit(f'\tjz {lbl_sil}')   # half_period=0 → silence
            # A = half_period (precomputed), D = base offset
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
                    'setjmp','ocall','tst','out_imm','cmpi','ddrb_imm','ddra_imm',
                    'ora_imm','orb_imm'}

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
                    s == 'jal __delay_cal' or s == 'jal __lcd_init'):
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
        INIT_ONLY_NAMES = {
            '__delay_cal', '__lcd_init', '__tone_setup',
            '__i2c_st', '__i2c_sp',  # START/STOP inlined in __lcd_chr at runtime
        }
        # __eeprom_rd is init-only when not used by runtime eeprom array access
        if not getattr(self, '_needs_runtime_eeprom_rd', False):
            INIT_ONLY_NAMES.add('__eeprom_rd')
        # I2C helpers that are init-only when no runtime I2C is needed
        I2C_INIT_ONLY = {'__i2c_sb', '__i2c_sp', '__i2c_st', '__i2c_st_only', '__i2c_rb'}

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
            for name, start, end, fsize in overlay_eligible:
                if remaining_size <= (240 - kernel_estimate - main_overhead):
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
            init_only_names = {'__delay_cal', '__lcd_init', '__tone_setup'}
            if not getattr(self, '_needs_runtime_i2c', False):
                init_only_names.update({'__i2c_sb', '__i2c_sp', '__i2c_st', '__i2c_rb'})

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
            INIT_PREFIXES_SET = ('ddrb_imm', 'exrw 2', 'ldi $d,', 'ddra_imm', 'push_imm')
            INIT_JAL_TARGETS = {'__delay_cal', '__lcd_init', '__tone_setup'}
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
                           s.startswith('.rcv') or s.startswith('mov $c,$a') or
                           s == 'nop')
                if s.startswith('jnz .') or s.startswith('j .'):
                    is_init = True
                # ldi before init-compatible jal
                if s.startswith('ldi $a,') or s.startswith('ldi $b,'):
                    for nli in range(mi + 1, min(mi + 4, len(user_code))):
                        ns = user_code[nli].strip()
                        if ns.startswith('jal '):
                            if ns.split()[1] in INIT_JAL_TARGETS:
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

            # Build output: init code + shared helpers + copy loop
            new_code = []

            # Init extraction path doesn't use page2 overlays — no SP init needed.
            # Auto-insert delay_calibrate if calibrated delays needed
            if getattr(self, '_needs_delay_calibrate', False):
                has_delay_cal = any('jal __delay_cal' in l for l in init_inline)
                if not has_delay_cal:
                    insert_at = len(init_inline)
                    for ci, cl in enumerate(init_inline):
                        if cl.strip().endswith(';!keep') or cl.strip() == 'jal __delay_cal':
                            insert_at = ci + 1
                    init_inline = (init_inline[:insert_at] +
                                   ['\tjal __delay_cal'] +
                                   init_inline[insert_at:])
                    if not hasattr(self, '_lcd_helpers'):
                        self._lcd_helpers = set()
                    self._lcd_helpers.add('__delay_cal')

            # Stage 1: inline code, skip over init-only helpers, shared helpers, copy loop
            new_code.extend(init_inline)
            new_code.append('\tj __init_done')
            new_code.extend(init_helper_lines)
            # Shared helpers: survive self-copy (at addresses > kernel_size)
            new_code.extend(shared_helper_lines)
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

            # Kernel in page3_kernel (resets code PC to 0 for self-copy)
            new_code.append('\tsection page3_kernel')
            new_code.extend(runtime_kernel)

            # LCD init data — no emission needed here (stored in EEPROM)
            if hasattr(self, '_lcd_print_strings'):
                if not any('section page3' in l for l in new_code[-5:]):
                    new_code.append('\tsection page3')
                for p3_off, s in self._lcd_print_strings:
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
                if s.startswith('jal '):
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

        # ── Step 5: Build overlay slots via library inlining ──
        # Each user function gets its own overlay containing ALL library helpers
        # it depends on. Library helpers are never split from their callers.
        # If a user+library combination exceeds the overlay region → fatal error.
        overlay_slots = []  # list of ([(name, start, end, fsize), ...], total_size)

        for comp in components:
            slot = [(n, *ov_by_name[n]) for n in comp]
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
                # so they must be in ONE overlay slot together.
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

        for idx, (slot_funcs, slot_size) in enumerate(overlay_slots):
            combined_lines = []
            for name, start, end, fsize in slot_funcs:
                combined_lines.extend(self.code[start:end])
                func_to_overlay_idx[name] = idx
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
                    # Thin dispatch: save args, load overlay, restore args, call directly.
                    # This avoids page3 scratch and keeps the loader minimal.
                    nparams = self.func_params.get(target.lstrip('_'), 0)
                    if nparams >= 2:
                        new_resident.append('\tpush $b')        # save arg2
                        new_resident.append('\tpush $a')        # save arg1
                    elif nparams == 1:
                        new_resident.append('\tpush $a')        # save arg1
                    # Initialize overlay cache on first call
                    if not cache_init_emitted:
                        new_resident.append('\tldi $a,0xFF')
                        new_resident.append('\tldi $b,249')
                        new_resident.append('\tiderefp3')       # page3[249] = 0xFF
                        cache_init_emitted = True
                    new_resident.append(f'\tldi $a,{idx}')
                    new_resident.append('\tjal _overlay_load')  # load only, returns
                    # Restore args and call overlay directly
                    if nparams >= 2:
                        new_resident.append('\tpop $a')         # restore arg1
                        new_resident.append('\tpop $b')         # restore arg2
                    elif nparams == 1:
                        new_resident.append('\tpop $a')         # restore arg1
                    new_resident.append(f'\tjal __ov_entry')    # call overlay
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

            for hname in list(cond_res):
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
                # Skip other helper bodies too (they might also be bundled)
                in_other_helper = False
                for oh, (ohs, ohe, _) in helper_bodies.items():
                    if oh != hname and ohs <= ci < ohe:
                        in_other_helper = True
                        break
                if in_other_helper:
                    continue
                cs = cline.strip()
                if f'jal {hname}' in cs:
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

        # Remove kept-resident helpers from bundleable list
        if keep_resident:
            bundleable_helpers = [h for h in bundleable_helpers if h not in keep_resident]
            # Also remove from deps (they're now resident, not bundled)
            for h in keep_resident:
                deps.pop(h, None)
            for h in deps:
                deps[h] -= keep_resident

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
            # internal labels from fall-through merging, e.g., __lcd_send inside __lcd_cmd)
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

        # Rebuild overlay_meta with updated sizes
        overlay_meta = [(idx, name, fsize) for idx, (name, _, fsize) in enumerate(overlay_asm_blocks)]

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
                if s.startswith('jal '):
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
            'jal __delay_cal', 'jal __lcd_init',
            'push_b', 'pop_b', 'iderefp3',
        }
        INIT_PREFIXES = ('ddrb_imm', 'exrw 2', 'ldi $d,', 'ddra_imm', 'push_imm')
        # Helpers that are safe to call during init (don't end init phase)
        INIT_COMPAT_HELPERS = {'__i2c_stream', '__i2c_st', '__i2c_sp',
                               '__delay_cal', '__lcd_init', '__tone_setup'}

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
                           s.startswith('.rcv') or s.startswith('mov $c,$a') or
                           s == 'nop')
                if s.startswith('jnz .') or s.startswith('j .via') or s.startswith('j .br') or s.startswith('j .rcv'):
                    is_init = True
                # ldi before init-compatible jal is also init
                if s.startswith('ldi $a,') or s.startswith('ldi $b,'):
                    for nli in range(mi + 1, min(mi + 4, len(main_lines))):
                        ns = main_lines[nli].strip()
                        if ns.startswith('jal '):
                            t = ns.split()[1]
                            if t in INIT_COMPAT_HELPERS or is_init_only(t):
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
                else:
                    in_init_phase = False
                    runtime_main.append(line)
            else:
                runtime_main.append(line)

        # Strip trailing unreachable ret
        while runtime_main and runtime_main[-1].strip() == 'ret':
            runtime_main.pop()
        # Bus recovery at _main start: self-copy and overlay loading use
        # derefp3/deref which toggle control signals (E0=SCL), corrupting
        # the I2C bus. A STOP condition resets the bus to idle before any
        # runtime I2C transactions.
        bus_recovery = [
            '\tddrb_imm 0x03',   # SCL low, SDA low
            '\tddrb_imm 0x01',   # SCL high, SDA low
            '\tddrb_imm 0x00',   # SCL high, SDA high (STOP/idle)
        ]
        main_lines = ['_main:'] + bus_recovery + runtime_main

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
                                     p3_count=0, p1_count=0, use_cache=False):
            """Generate the overlay loader assembly.
            Layout in page3 after kernel:
              __manifest: [offset0, size0, offset1, size1, ...]  (2B each)
              __pages:    [page0, page1, ...]                     (1B each)
            Page values: 3=derefp3, 1=deref, 2=deref2 (future: 0=EEPROM)
            The loader reads page from __pages[index] and dispatches.
            No index-range tracking needed — trivial to add new page types.
            """
            num_pages = sum([has_p3, has_p1, has_p2])
            loader = [
                '; ── overlay loader (kernel) ──',
                '_overlay_load:',
                '\tmov $a,$c',                  # C = index
                '\tsll',                        # A = index * 2
                '\taddi __manifest,$a',          # A = __manifest + index*2
                # Read manifest entry [offset, size]
                '\tmov $a,$d',                  # D = manifest addr
                '\tderefp3',                    # A = src_offset
                '\tmov $a,$c',                  # C = src offset
                '\tmov $d,$a',                  # A = manifest addr
                '\tinc',                        # A = manifest addr + 1
                '\tderefp3',                    # A = size
                f'\taddi {overlay_region},$a',  # A = overlay_region + size = end
                '\tmov $a,$d',                  # D = end addr
                f'\tldi $b,{overlay_region}',   # B = dest start
            ]

            # Page dispatch: read __pages[index] to determine which deref to use
            if num_pages > 1:
                loader += [
                    '\tpush $d',                # save end addr
                    '\tpush $c',                # save src offset
                    # Reload index from original argument (it's still in the
                    # return address area... no, we need it fresh).
                    # Actually C was clobbered. But we saved index*2 earlier.
                    # Simpler: re-derive index from B and manifest layout.
                    # Or: just read __pages + index. We need index.
                    # Index was in C at entry, then C was overwritten with src_offset.
                    # We need to save index somewhere. Use the stack:
                ]
                # Hmm, we lost the index. Let me restructure to save it earlier.
                # Restart the loader with index preservation:
                loader = [
                    '; ── overlay loader (kernel) ──',
                    '_overlay_load:',
                    '\tpush $a',                    # save index for page lookup
                    '\tmov $a,$c',                  # C = index
                    '\tsll',                        # A = index * 2
                    '\taddi __manifest,$a',          # A = __manifest + index*2
                    '\tmov $a,$d',                  # D = manifest addr
                    '\tderefp3',                    # A = src_offset
                    '\tmov $a,$c',                  # C = src offset
                    '\tmov $d,$a',                  # A = manifest addr
                    '\tinc',
                    '\tderefp3',                    # A = size
                    f'\taddi {overlay_region},$a',
                    '\tmov $a,$d',                  # D = end addr
                    f'\tldi $b,{overlay_region}',   # B = dest start
                    # Now read page: __pages[index]
                    '\tpop $a',                     # A = index (saved at entry)
                    '\taddi __pages,$a',             # A = __pages + index
                    '\tderefp3',                    # A = page number (1,2,3)
                    # Dispatch on page number
                    '\tcmpi 2',                     # page >= 2?
                ]
                if has_p2:
                    loader.append('\tjz .copy_p2')  # page == 2
                    loader.append('\tjc .copy_p3')  # page == 3 (>2)
                else:
                    loader.append('\tjc .copy_p3')  # page >= 2 means page 3
                # Fall through to page 1 (default)
            else:
                # Single page — no dispatch needed, no push/pop
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
                    '\tmov $c,$a',          # A = src (from C)
                    '\tinc',
                    '\tmov $a,$c',          # C = src + 1
                    '\tmov $b,$a',          # A = B (dest)
                    '\tcmp $d',             # dest == end?
                    f'\tjnz {label}',
                ]
                if not is_last:
                    loader.append('\tj .ov_done')

            # Bus recovery + return
            loader.append('.ov_done:')
            loader.append('\tddrb_imm 0x03')
            loader.append('\tddrb_imm 0x01')
            loader.append('\tddrb_imm 0x00')
            loader.append('\tret')
            return loader

        # ── Step 13: Two-pass sizing ──
        # Pass 1: optimistic estimate (page3 only → smallest loader)
        # If overlays don't fit in page3, we'll re-estimate with more pages
        loader_pass1 = generate_kernel_loader(OVERLAY_REGION, 0, False, False, True, 2, 0, use_cache=False)
        loader_size = measure_lines(loader_pass1)
        main_size = measure_lines(main_lines)

        # Extract _NO_OVERLAY helpers from other_resident into kernel.
        # These are runtime helpers (tone, I2C, etc.) that must be callable
        # from overlay code — they live permanently in the code page kernel.
        runtime_resident_helpers = []
        runtime_helper_names = set()
        for label_s in _NO_OVERLAY:
            name = label_s.rstrip(':')
            if name.startswith('__'):
                runtime_helper_names.add(name)
        if runtime_helper_names:
            rh_lines = []
            non_rh_lines = []
            in_helper = False
            for line in other_resident:
                s = line.strip()
                if s.endswith(':') and not s.startswith('.') and s.startswith('_'):
                    hname = s[:-1]
                    if hname in runtime_helper_names:
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
        init_only_callers = {'__lcd_init', '__eeprom_rd', '__delay_cal', '__tone_setup'}
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
        kernel_max_est = KERNEL_SIZE_est + shared_helper_size + 60  # headroom for knapsack
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
        # Page 2 needs SP init (3B) which increases init code. Estimate if
        # init code can afford it (must stay ≤ 250B including selfcopy).
        # If the program has LCD init (heavy init helpers), init is ~240B
        # without SP init. Adding 3B pushes to 243B + selfcopy 14B = 257B > 250.
        # In that case, disable page 2 to avoid the SP init overhead.
        has_heavy_init = hasattr(self, '_lcd_helpers') and '__lcd_init' in getattr(self, '_lcd_helpers', set())
        p2_capacity = 196 if not has_heavy_init else 0

        p3_overlays = []
        p1_overlays = []
        p2_overlays = []

        # Best-fit placement: for each overlay, pick the page with least remaining waste
        def place_overlay(idx, name, asm_lines, fsize):
            nonlocal p3_code_offset, p3_capacity, p1_code_offset, p1_capacity, p2_code_offset, p2_capacity
            candidates = []
            if fsize <= p3_capacity:
                candidates.append((p3_capacity - fsize, 3))
            if fsize <= p1_capacity:
                candidates.append((p1_capacity - fsize, 1))
            if fsize <= p2_capacity:
                candidates.append((p2_capacity - fsize, 2))
            if not candidates:
                return False
            # Best fit: smallest remaining space after placement
            candidates.sort()
            _, page = candidates[0]
            if page == 3:
                p3_overlays.append((idx, name, asm_lines, fsize, p3_code_offset))
                p3_code_offset += fsize; p3_capacity -= fsize
            elif page == 1:
                p1_overlays.append((idx, name, asm_lines, fsize, p1_code_offset))
                p1_code_offset += fsize; p1_capacity -= fsize
            elif page == 2:
                p2_overlays.append((idx, name, asm_lines, fsize, p2_code_offset))
                p2_code_offset += fsize; p2_capacity -= fsize
            return True

        _placement_retries = 0
        _retry_bundling = False
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
                if not placed:
                    # Placement failed. Find the largest bundled helper in this
                    # overlay and force it resident to shrink the overlay.
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
                    if biggest_helper and _placement_retries < 3:
                        import sys
                        print(f"  Placement retry: making {biggest_helper} ({biggest_size}B) resident to fit {name} ({fsize}B)",
                              file=sys.stderr)
                        _NO_OVERLAY.add(f'{biggest_helper}:')
                        bundleable_helpers = [h for h in bundleable_helpers if h != biggest_helper]
                        _placement_retries += 1
                        _retry_bundling = True
                        break  # break out of placement loop to retry
                    raise Exception(
                        f"overlay '{name}' ({fsize}B) exceeds ALL SRAM cache. "
                        f"p3={p3_capacity}B free, p1={p1_capacity}B free, p2={p2_capacity}B free"
                    )

        has_p1 = len(p1_overlays) > 0
        has_p2 = len(p2_overlays) > 0
        has_p3 = len(p3_overlays) > 0

        if not has_p1 and not has_p2 and not has_p3:
            return  # nothing placed

        # ── Step 15: Final kernel sizing ──
        # Generate kernel with only the page copy loops actually needed
        for _pass in range(3):

            loader = generate_kernel_loader(OVERLAY_REGION_est, meta_base,
                                            has_p1, has_p2, has_p3,
                                            len(p3_overlays), len(p1_overlays), use_cache=False)
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
                                        len(p3_overlays), len(p1_overlays), use_cache=False)
        loader_size = measure_lines(loader)
        KERNEL_SIZE = loader_size + main_size + runtime_helper_size
        OVERLAY_REGION = KERNEL_SIZE + shared_helper_size

        # Fix p3 overlay offsets if meta_base changed from estimate
        final_meta_base = max(p3_used, OVERLAY_REGION)
        final_p3_start = final_meta_base + meta_table_size
        if p3_overlays:
            old_first_offset = p3_overlays[0][4]
            if old_first_offset != final_p3_start:
                delta = final_p3_start - old_first_offset
                p3_overlays = [(idx, n, a, f, off + delta)
                               for idx, n, a, f, off in p3_overlays]
        meta_base = final_meta_base

        # Validate overlay region
        # Overlay region starts after kernel + shared helpers
        if OVERLAY_REGION + max_slot_size > 252:  # 252: leave 4B safety margin
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
        if p3_overlays:
            old_first = p3_overlays[0][4]
            if old_first != final_p3_start:
                delta = final_p3_start - old_first
                p3_overlays = [(i, n, a, f, off + delta) for i, n, a, f, off in p3_overlays]
        # Regenerate loader with correct OVERLAY_REGION
        loader = generate_kernel_loader(OVERLAY_REGION, meta_base,
                                        has_p1, has_p2, has_p3,
                                        len(p3_overlays), len(p1_overlays), use_cache=False)
        loader_size = measure_lines(loader)
        KERNEL_SIZE = RESET_STUB + loader_size + actual_main_size + runtime_helper_size
        OVERLAY_REGION = KERNEL_SIZE + shared_helper_size
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
            '\tmov $d,$a',                      # A = D (count)
            '\tdec',
            '\tmov $a,$d',                      # D--
            '\tjnz .selfcopy_loop',
            '\tj 0',                            # jump to kernel at code[0]
        ]

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
        # After first run, page3 has calibrated ipm and precomputed note table,
        # so _main can replay without re-calibrating.
        assembled.append('\tj _main')
        assembled.extend(loader)
        assembled.extend(runtime_resident_helpers)
        assembled.extend(runtime_main_lines)
        # Shared helpers in page3 at [KERNEL_SIZE..OVERLAY_REGION-1].
        # These survive self-copy (self-copy copies OVERLAY_REGION bytes).
        # During init, a mini-copy bootstraps them from page3 to code page.
        assembled.extend(shared_helpers_ov)

        # Phase 2: Manifest — stay in page3_code (same section as kernel).
        # 'byte' in page3_code goes through emitCode→emitPage3, keeping
        # code_size and page3_size in sync. The loader's meta_base (from
        # code_size) then matches the actual page3 data position.
        # (DO NOT switch to 'section page3' — that desynchs them.)

        # Emit 2-byte manifest entries: [src_offset, size]
        # Label prevents peephole dead-code elimination (manifest follows kernel's hlt)
        assembled.append('__manifest:')
        # Overlays ordered: page3 first, then page1, then page2.
        # Page determined at runtime by index range vs p3_count/p1_count.
        all_placed_ordered = list(p3_overlays) + list(p1_overlays) + list(p2_overlays)

        # Rebuild func_to_overlay_idx with new ordering
        # (the call rewriting at step 8 already used the old indices,
        # but the manifest is emitted in the new order. We need to
        # re-map the ldi $a,N values in the assembled code.)
        old_to_new = {}
        for new_idx, (old_idx, name, asm_lines, fsize, offset) in enumerate(all_placed_ordered):
            old_to_new[old_idx] = new_idx

        # Fix up overlay indices in assembled code (ldi $a,N before jal _overlay_load)
        for ci in range(len(assembled) - 1):
            s = assembled[ci].strip()
            nxt = assembled[ci + 1].strip()
            if nxt == 'jal _overlay_load' and s.startswith('ldi $a,'):
                try:
                    old_idx = int(s.split(',')[1])
                    if old_idx in old_to_new:
                        assembled[ci] = f'\tldi $a,{old_to_new[old_idx]}'
                except ValueError:
                    pass

        # Manifest: [offset, size] per overlay (2 bytes each)
        for _, name, asm_lines, fsize, offset in all_placed_ordered:
            assembled.append(f'\tbyte {offset}')
            assembled.append(f'\tbyte {fsize}')

        # Page table: [page] per overlay (1 byte each)
        # page values: 3=page3(derefp3), 1=page1(deref), 2=page2(deref2)
        assembled.append('__pages:')
        # Determine which page each overlay is on
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
                assembled.append('\tbyte 0')  # future: EEPROM

        # __ov_entry: marks the start of the overlay region (address OVERLAY_REGION).
        # Emitted after manifest so it resolves to the correct code address.
        # Caller does 'jal __ov_entry' after _overlay_load copies the code.
        assembled.append('\tsection code')
        assembled.append(f'\torg {OVERLAY_REGION}')
        assembled.append('\tsection page3_code')
        assembled.append('__ov_entry:')

        # Phase 3: Overlay code at OVERLAY_REGION addresses into storage pages
        for idx, name, asm_lines, fsize, _ in p3_overlays:
            assembled.append('\tsection code')
            assembled.append(f'\torg {OVERLAY_REGION}')
            assembled.append('\tsection page3_code')
            assembled.extend(asm_lines)

        # Pad data section to data_alloc so overlay data starts at the correct offset.
        # The manifest stores offsets based on data_alloc, but the assembler's
        # data_size starts at 0. Without padding, overlay data at data[0] wouldn't
        # match the manifest offset (data_alloc).
        if p1_overlays and self.data_alloc > 0:
            assembled.append('\tsection data')
            for _ in range(self.data_alloc):
                assembled.append('\tbyte 0')

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
            assembled.append('\tmov $d,$a')
            assembled.append('\tdec')
            assembled.append('\tmov $a,$d')
            assembled.append(f'\tjnz {mc_lbl}')
            for line in runtime_resident_helpers:
                s = line.strip()
                if s.endswith(':') and s.startswith('__'):
                    mini_copied_helpers.add(s[:-1])

        # Auto-insert delay_calibrate if calibrated delays are needed
        # (e.g., lcd_cmd uses __delay_Nms) but user didn't call delay_calibrate()
        if getattr(self, '_needs_delay_calibrate', False):
            has_delay_cal = any('jal __delay_cal' in l for l in init_code_lines)
            if not has_delay_cal:
                # Add delay_calibrate after i2c_init (after the ;!keep markers)
                insert_at = len(init_code_lines)
                for ci, cl in enumerate(init_code_lines):
                    if cl.strip().endswith(';!keep') or cl.strip() == 'jal __delay_cal':
                        insert_at = ci + 1
                init_code_lines = (init_code_lines[:insert_at] +
                                   ['\tjal __delay_cal'] +
                                   init_code_lines[insert_at:])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__delay_cal')

        # Init sequence extracted from main (VIA init, calibration calls, etc.)
        assembled.extend(init_code_lines)
        # Jump past helper bodies to note precomputation / self-copy
        if init_only_funcs:
            assembled.append('\tj __init_done')

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

        for name, start, end, fsize in init_only_funcs:
            assembled.extend(_rename_shared_refs(self.code[start:end]))

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

        # Also include any non-shared, non-init-only helpers needed by init
        if init_code_lines:
            init_needs = set()
            all_init_lines = list(init_code_lines)
            for name, start, end, fsize in init_only_funcs:
                all_init_lines.extend(self.code[start:end])
            for line in all_init_lines:
                s = line.strip()
                if s.startswith('jal '):
                    target = s.split()[1]
                    if target.startswith('__'):
                        init_needs.add(target)

            needed_but_not_init = init_needs - {n for n, _, _, _ in init_only_funcs}
            # Exclude mini-copied helpers (available at kernel addresses)
            needed_but_not_init -= mini_copied_helpers
            # Exclude helpers that are still resident (don't need init copies)
            needed_but_not_init -= {n for n in needed_but_not_init
                                    if f'{n}:' in _NO_OVERLAY and n not in init_rename_set}
            if needed_but_not_init:
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
                        if hname in needed_but_not_init:
                            # Rename if this helper has labels in overlay sections
                            helper_lines = self.code[hstart:i]
                            if hname in init_rename_set:
                                helper_lines = _rename_shared_refs(helper_lines)
                                # Also rename local labels to avoid conflicts
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
                                            # Rename if this local label exists in the helper
                                            if any(ref + ':' in h2.strip() for h2 in self.code[hstart:i]):
                                                l = l.replace(ref, ref + '_i')
                                    renamed.append(l)
                                helper_lines = renamed
                            assembled.extend(helper_lines)
                    else:
                        i += 1

        # Label: init helpers done, continue to precomputation / self-copy
        if init_only_funcs:
            assembled.append('__init_done:')

        # DDRA for tone: now set inside __tone itself (bracket with ddra_imm 0x02
        # before toggle loop, ddra_imm 0x00 after). Persistent DDRA=0x02 breaks
        # delay loops due to VIA bus coupling (confirmed in stopwatch testing).

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
        init_size = measure_lines([l for l in assembled[phase4_start:]
                                   if l.strip() and not l.strip().startswith(';')
                                   and 'section' not in l and 'org' not in l])
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

        # ── Diagnostics ──
        print(f"Two-stage boot overlay system:", file=sys.stderr)
        print(f"  Kernel: {KERNEL_SIZE}B (loader={loader_size}B + main={actual_main_size}B"
              f" + helpers={runtime_helper_size}B)"
              f" + shared={shared_helper_size}B", file=sys.stderr)
        print(f"  Overlay region: 0x{OVERLAY_REGION:02X}-0xFF ({256-OVERLAY_REGION}B)",
              file=sys.stderr)
        print(f"  {len(overlay_slots)} overlay slots, largest={max_slot_size}B", file=sys.stderr)
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
            # Only save A→D if function has locals or complex expressions that
            # will clobber A between uses of param 0. Simple heuristic: save
            # if function body declares any local variables.
            has_locals = any(s[0] == 'local' for s in body[1]) if body[0] == 'block' else False
            save_a_to_d = (len(params) >= 1
                           and params[0] in self.read_vars
                           and self.reg_d_var is None
                           and (has_locals or len(params) > 2
                                or self._has_calls(body)))
            for i, pname in enumerate(params):
                if i == 0:
                    if save_a_to_d:
                        self.locals[pname] = ('reg', 'd')  # will be saved at entry
                    else:
                        self.locals[pname] = ('regparam', 'a')
                elif i == 1:
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

        self.compile_stmt(body)

        for _ in range(self.local_count):
            self.emit('\tpop $d')
        # Don't emit ret if the last instruction is already ret (from a return statement)
        if not self.code or self.code[-1].strip() != 'ret':
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
            elif e[0] == 'assign' and e[1] in ('+=','-=','*=','&=','|=','^=') and e[2][0] == 'var':
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
                        elif op in ('+=','-=','*=','&=','|=','^='):
                            cur = env.get(name, 0)
                            ops = {'+':lambda a,b:a+b, '-':lambda a,b:a-b,
                                   '*':lambda a,b:a*b, '&':lambda a,b:a&b,
                                   '|':lambda a,b:a|b, '^':lambda a,b:a^b}
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
                self.gen_expr(right)
                self.emit('\tpush $a')
                self.local_count += 1
                self.gen_expr(left[2])
                if is_page3:
                    base = self.page3_globals.get(arr_name, 0)
                else:
                    base = self.globals.get(arr_name, 0)
                if base:
                    self.emit(f'\taddi {base},$a')
                self.emit('\tmov $a,$b')
                self.emit('\tpop $a')
                self.local_count -= 1
                self.emit('\tiderefp3' if is_page3 else '\tideref')
                return

            if op != '=':
                self.gen_expr(left)
                self.emit('\tmov $a,$b')
                self.gen_expr(right)
                self.emit('\tswap')
                self._emit_binop(op[0])
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
                    else:
                        pass  # fall through to general multiply
                    if val in (0, 1, 2, 3, 4, 5, 6, 8, 10):
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
                if op not in ('^', '*'):
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
                self.emit('\tddrb_imm 0x00')   # DDRB = 0 (SDA/SCL idle)
                self.emit('\thlt')
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
                if args:
                    self.const_vars.pop('__a_known', None)  # force reload
                    self.gen_expr(args[0])
                # Determine which exw variant
                mode = 0
                enable = 0
                if len(args) >= 2:
                    c = self._const_eval(args[1])
                    if c is not None:
                        mode = c & 3
                if len(args) >= 3:
                    c = self._const_eval(args[2])
                    if c is not None:
                        enable = c & 1
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
                # VIA init for I2C: delay for VIA ~RES settle (RC = 1ms),
                # then set ORB=0, DDRB=0, DDRA=0, init SP.
                self.emit('\tldi $d,0')
                lbl = self.label('via_dly')
                self.emit(f'{lbl}:')
                self.emit('\tdec')
                self.emit(f'\tjnz {lbl}')
                # After delay loop: A=0
                self.emit("\tclr $a")
                self.emit('\texw 0 0')         # ORB = 0 (A=0)
                self.emit('\tddrb_imm 0x00')   # DDRB = 0 (both lines idle/HIGH)
                # NOTE: exw 0 3 (DDRA=0) REMOVED — it breaks EEPROM reads.
                self.emit('\tpush $a ;!keep')  # stack warmup (STK pin settling)
                self.emit('\tpop $a ;!keep')
                return

            if name == 'i2c_bus_reset':
                # Bus recovery STOP to clear stuck I2C state.
                # VIA delay/init already done by i2c_init — just do the STOP.
                self.emit('\tddrb_imm 0x03')   # STOP: both LOW
                self.emit('\tddrb_imm 0x01')   # STOP: SDA LOW, SCL HIGH
                self.emit('\tddrb_imm 0x00')   # STOP: both HIGH (idle)
                return

            if name == 'lcd_init':
                # Complete LCD init sequence as one jal call
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_init')
                # __lcd_init reads from EEPROM via __eeprom_rd
                self._lcd_helpers.add('__eeprom_rd')
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                self.emit('\tjal __lcd_init')
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
                self.emit('\tddrb_imm 0x01')   # SDA LOW, SCL HIGH
                self.emit('\tddrb_imm 0x03')   # both LOW
                return

            if name == 'i2c_stop':
                # Inline STOP
                self.emit('\tddrb_imm 0x03')   # both LOW
                self.emit('\tddrb_imm 0x01')   # SDA LOW, SCL HIGH
                self.emit('\tddrb_imm 0x00')   # both HIGH (idle)
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

            if name == 'i2c_read_byte':
                # Read one I2C byte into A (8 bits MSB first, SDA released)
                # Caller must send ACK or NACK after this
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_rb')
                self.emit('\tjal __i2c_rb')
                self.emit('\tmov $d,$a')  # A = byte read (from D)
                return

            if name == 'eeprom_write_byte':
                # eeprom_write_byte(addr, data) — addr is 16-bit, split at compile time
                # Also accepts eeprom_write_byte(addr_hi, addr_lo, data) for backwards compat
                if not args or len(args) < 2:
                    self.gen_error("eeprom_write_byte(addr, data) or eeprom_write_byte(hi, lo, data)")
                    return
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                if len(args) == 2:
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
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
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
                self.emit('\tddrb_imm 0x03')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
                # ACK poll (wait for write cycle)
                lbl = self.label('ewp')
                self.emit(f'{lbl}:')
                self.emit('\texrw 2')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tldi $a,0xAE')
                self.emit('\tjal __i2c_sb')
                self.emit('\tpush $a')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
                self.emit('\tpop $a')
                self.emit('\ttst 0x01')
                self.emit(f'\tjnz {lbl}')
                return

            if name == 'eeprom_read_byte':
                # eeprom_read_byte(addr) — addr is 16-bit constant, returns byte in A
                # Also accepts eeprom_read_byte(addr_hi, addr_lo)
                if not args:
                    self.gen_error("eeprom_read_byte(addr) or eeprom_read_byte(hi, lo)")
                    return
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                if len(args) == 1:
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
                # Set address: START + 0xAE + addr_hi + addr_lo + STOP
                self.emit('\texrw 2')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
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
                self.emit('\tddrb_imm 0x03')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
                # Read: START + 0xAF + read byte + NACK + STOP
                self.emit('\texrw 2')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tldi $a,0xAF')
                self.emit('\tjal __i2c_sb')
                self.emit('\tjal __i2c_rb')
                self.emit('\tmov $d,$a')       # A = read byte
                # NACK + STOP (merged)
                self.emit('\tddrb_imm 0x02')
                self.emit('\tddrb_imm 0x00')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
                return

            if name == 'rtc_read_seconds':
                # Read DS3231 seconds register (0x00), returns BCD in A
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                # Set register pointer: START + 0xD0 + 0x00 + STOP
                self.emit('\texrw 2')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tldi $a,0xD0')
                self.emit('\tjal __i2c_sb')
                self.emit('\tclr $a')
                self.emit('\tjal __i2c_sb')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
                # Read: START + 0xD1 + read byte + NACK + STOP
                self.emit('\texrw 2')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tldi $a,0xD1')
                self.emit('\tjal __i2c_sb')
                self.emit('\tjal __i2c_rb')
                self.emit('\tmov $d,$a')
                self.emit('\tddrb_imm 0x02')
                self.emit('\tddrb_imm 0x00')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
                return

            if name == 'rtc_read_temp':
                # Read DS3231 temperature MSB (register 0x11), returns signed °C in A
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self._lcd_helpers.add('__i2c_rb')
                # Set register pointer to 0x11
                self.emit('\texrw 2')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tldi $a,0xD0')
                self.emit('\tjal __i2c_sb')
                self.emit('\tldi $a,0x11')
                self.emit('\tjal __i2c_sb')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
                # Read MSB
                self.emit('\texrw 2')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tldi $a,0xD1')
                self.emit('\tjal __i2c_sb')
                self.emit('\tjal __i2c_rb')
                self.emit('\tmov $d,$a')
                self.emit('\tddrb_imm 0x02')
                self.emit('\tddrb_imm 0x00')
                self.emit('\tddrb_imm 0x03')
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
                return

            if name == 'i2c_ack':
                # Send ACK (pull SDA LOW, clock SCL once)
                self.emit('\tddrb_imm 0x03')   # SDA LOW, SCL LOW
                self.emit('\tddrb_imm 0x01')   # SDA LOW, SCL HIGH
                self.emit('\tddrb_imm 0x02')   # SDA released, SCL LOW
                return

            if name == 'i2c_nack':
                # Send NACK (SDA released/HIGH, clock SCL once)
                self.emit('\tddrb_imm 0x02')   # SDA released, SCL LOW
                self.emit('\tddrb_imm 0x00')   # SDA released, SCL HIGH
                self.emit('\tddrb_imm 0x02')   # SDA released, SCL LOW
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
                self.emit('\tddrb_imm 0x01')       # START
                self.emit('\tddrb_imm 0x03')
                self.emit(f'\tldi $a,0xAE')        # EEPROM write
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $a,{(ee_addr >> 8) & 0xFF}')  # addr high
                self.emit('\tjal __i2c_sb')
                self.emit(f'\tldi $a,{ee_addr & 0xFF}')         # addr low
                self.emit('\tjal __i2c_sb')
                self.emit('\tddrb_imm 0x00')       # repeated START
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
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
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x03')
                if c is not None:
                    self.emit(f'\tldi $a,{c & 0xFF}')
                else:
                    self.gen_expr(args[0])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__i2c_sb')
                self.emit('\tjal __i2c_sb')      # send addr, A = ACK bit
                self.emit('\tpush $a')
                self.emit('\tddrb_imm 0x03')     # STOP
                self.emit('\tddrb_imm 0x01')
                self.emit('\tddrb_imm 0x00')
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
                self.emit('\tjal __delay_cal')
                return

            if name == 'delay':
                # delay N ms using calibrated value from data[0].
                # A = ms count. Calls __delay_Nms.
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        self.emit(f'\tldi $a,{c & 0xFF}')
                    else:
                        self.gen_expr(args[0])
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__delay_Nms')
                self.emit('\tmov $a,$b')    # B = ms count
                self.emit('\tjal __delay_Nms')
                return

            if name == 'tone':
                # tone(freq_hz, duration_ms) — play square wave on PA1.
                # Both args MUST be compile-time constants.
                # Stores [ratio, cyc_lo, cyc_hi] in data page note table.
                # Emits: ddra_imm 0x02 (once) + ldi $a, offset; jal __play_note.
                # DDRA bit 1 must be set for PA1 output (i2c_init clears DDRA).
                # DDRA is now set inside __tone itself (bracketed), not here.
                # Persistent DDRA=0x02 breaks delay loops via VIA bus coupling.
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
                self.emit(f'\tldi $a,{p1_off}')
                self.emit('\tjal __play_note')
                return

            if name == 'lcd_cmd' or name == 'lcd_char':
                # LCD command (RS=0) or character (RS=1) via I2C to PCF8574.
                # Emits a jal to a generated helper function.
                # The helper is emitted ONCE at the end of compilation.
                is_char = (name == 'lcd_char')
                flags = 0x09 if is_char else 0x00  # RS+BL or nothing
                flags_en = (flags | 0x04) if is_char else 0x04  # +EN
                if args:
                    c = self._const_eval(args[0])
                    if c is not None:
                        self.emit(f'\tldi $d,{c & 0xFF}')
                    else:
                        self.gen_expr(args[0])
                        self.emit('\tmov $a,$d')
                helper = f'__lcd_{"chr" if is_char else "cmd"}'
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add(helper)
                self.emit(f'\tjal {helper}')
                # HD44780 execution delay after lcd_cmd (not needed for lcd_char).
                # MUST use calibrated __delay_Nms — clock speed varies.
                # Clear display (0x01) / return home (0x02): 1.52ms → delay(2)
                # All other commands: 37µs → delay(1)
                if not is_char:
                    if not hasattr(self, '_lcd_helpers'):
                        self._lcd_helpers = set()
                    self._lcd_helpers.add('__delay_Nms')
                    # Flag that delay calibration is needed in init
                    self._needs_delay_calibrate = True
                    if c is not None and c in (0x01, 0x02):
                        self.emit('\tldi $b,2')    # 2ms
                    else:
                        self.emit('\tldi $b,1')    # 1ms
                    self.emit('\tjal __delay_Nms')
                return

            if name == 'lcd_print':
                # lcd_print("string") — store string in page 3, emit loop to print
                if args and args[0][0] == 'string':
                    s = args[0][1]
                    # Allocate string bytes in page 3 (null-terminated)
                    p3_off = self.page3_alloc
                    if not hasattr(self, '_lcd_print_strings'):
                        self._lcd_print_strings = []
                    self._lcd_print_strings.append((p3_off, s))
                    self.page3_alloc += len(s) + 1  # +1 for null terminator
                    # Emit call to shared __lcd_print helper
                    self.emit(f'\tldi $a,{p3_off}')
                    if not hasattr(self, '_lcd_helpers'):
                        self._lcd_helpers = set()
                    self._lcd_helpers.add('__lcd_print')
                    self._lcd_helpers.add('__lcd_chr')  # __lcd_print calls __lcd_chr
                    self.emit('\tjal __lcd_print')
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
                self.emit(f'\tstsp {off}')
                self.emit('\tmov $d,$a')  # D = original A after stsp
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
    changed = True
    while changed:
        changed = False
        out = []
        i = 0
        while i < len(lines):
            line = lines[i]
            if is_instr(line) and mnemonic(line) in TERMINATORS:
                out.append(line)
                i += 1
                while i < len(lines) and not is_label(lines[i]) and not is_section(lines[i]):
                    changed = True
                    i += 1
                continue
            out.append(line)
            i += 1
        lines = out

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

        # stsp: clobbers A, but D = original A (microcode saves A→D first)
        if mn == 'stsp':
            out.append(line)
            regs['d'] = regs['a']  # D gets the value A had before stsp
            regs['a'] = fresh_unknown()       # A is clobbered (holds offset)
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
    changed = True
    while changed:
        changed = False
        out = []
        i = 0
        while i < len(lines):
            line = lines[i]

            # stsp N / ldsp N → stsp N (when next instr clobbers A)
            if is_instr(line) and mnemonic(line) == 'stsp':
                parts_st = line.strip().split()
                if (i + 1 < len(lines) and is_instr(lines[i+1])):
                    parts_ld = lines[i+1].strip().split()
                    if (len(parts_st) == 2 and len(parts_ld) == 2
                            and parts_ld[0] == 'ldsp' and parts_st[1] == parts_ld[1]
                            and i + 2 < len(lines)):
                        # Check what follows the ldsp
                        nxt = lines[i+2] if i + 2 < len(lines) else None
                        if nxt and is_instr(nxt):
                            nmn = mnemonic(nxt)
                            if nmn in CLOBBERS_A:
                                out.append(line)
                                i += 2
                                changed = True
                                continue
                        # Also safe if followed by label then clobber
                        if nxt and is_label(nxt) and i + 3 < len(lines) and is_instr(lines[i+3]):
                            nmn = mnemonic(lines[i+3])
                            if nmn in CLOBBERS_A:
                                out.append(line)
                                i += 2
                                changed = True
                                continue

            # ldi $a,X / push $a → push_imm X
            if (is_instr(line) and line.strip().startswith('ldi $a,')
                    and i + 1 < len(lines) and lines[i+1].strip() == 'push $a'):
                val = line.strip().split(',')[1]
                out.append(f'\tpush_imm {val}')
                i += 2
                changed = True
                continue

            # push $a / pop $a → eliminate both (no-op)
            # Skip if either line has ;!keep annotation (hardware side-effect)
            if (is_instr(line) and line.strip() == 'push $a'
                    and i + 1 < len(lines) and lines[i+1].strip() == 'pop $a'
                    and ';!keep' not in line and ';!keep' not in lines[i+1]):
                i += 2
                changed = True
                continue

            # push $a / pop $b → mov $a,$b (1 byte instead of 2)
            if (is_instr(line) and line.strip() == 'push $a'
                    and i + 1 < len(lines) and is_instr(lines[i+1])):
                pop_s = lines[i+1].strip()
                if pop_s.startswith('pop ') and pop_s != 'pop $a':
                    dst = pop_s.split()[1]
                    out.append(f'\tmov $a,{dst}')
                    i += 2
                    changed = True
                    continue

            # Branch threading: j/jz/jnz/jc/jnc to a label that is just j elsewhere
            if is_instr(line) and mnemonic(line) in ('j', 'jz', 'jnz', 'jc', 'jnc'):
                parts_j = line.strip().split()
                if len(parts_j) == 2:
                    target = parts_j[1]
                    # Find the target label and check if next instruction is j
                    for k in range(len(lines)):
                        if lines[k].strip() == f'{target}:':
                            if (k + 1 < len(lines) and is_instr(lines[k+1])
                                    and mnemonic(lines[k+1]) == 'j'):
                                final = lines[k+1].strip().split()[1]
                                out.append(f'\t{parts_j[0]} {final}')
                                i += 1
                                changed = True
                                break
                    else:
                        out.append(line)
                        i += 1
                    continue

            out.append(line)
            i += 1
        lines = out

    return lines


def main():
    ap = argparse.ArgumentParser(description='MK1 C Compiler v2')
    ap.add_argument('input', help='C source file')
    ap.add_argument('-o', '--output', help='Output .asm file')
    ap.add_argument('-O', '--optimize', action='store_true', help='Enable optimization')
    ap.add_argument('--eeprom', action='store_true', help='EEPROM overlay mode: partition code for EEPROM-backed execution')
    ap.add_argument('--eeprom-base', type=lambda x: int(x, 0), default=0x0200,
                    help='EEPROM base address for overlay storage (default: 0x0200)')
    args = ap.parse_args()

    with open(args.input) as f: source = f.read()

    gen = MK1CodeGen(optimize=args.optimize)
    if hasattr(args, 'eeprom') and args.eeprom:
        gen.eeprom_mode = True
        gen.eeprom_base = args.eeprom_base
    asm = gen.compile(source)

    # Run peephole optimizer — but NOT for overlay programs where
    # manifest offsets are already computed from pre-peepholed component sizes.
    # A final peephole would shrink the kernel, making offsets wrong.
    lines = asm.split('\n')
    if not getattr(gen, '_overlay_kernel_size', None):
        lines = peephole(lines)
        asm = '\n'.join(lines)

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
    eeprom_bytes = 0
    section = 'code'
    two_byte_set = {'ldsp','stsp','push_imm','jal','jc','jz','jnc','jnz','j','ldi',
                    'cmp','addi','subi','andi','ori','ld','st','ldsp_b','ldp3','stp3',
                    'setjmp','ocall','tst','out_imm','cmpi','ddrb_imm','ddra_imm',
                    'ora_imm','orb_imm'}
    for line in lines:
        s = line.strip()
        if 'section page3_kernel' in s or 'section page3_code' in s:
            section = 'page3'; continue
        elif 'section page3' in s and 'kernel' not in s and 'code' not in s:
            section = 'page3_data'; continue
        elif 'section eeprom' in s: section = 'eeprom'; continue
        elif 'section data_code' in s: section = 'data'; continue
        elif 'section stack_code' in s: section = 'stack'; continue
        elif 'section data' in s: section = 'data'; continue
        elif 'section code' in s: section = 'code'; continue
        if not s or s.endswith(':') or s.startswith(';'): continue
        mn = s.split()[0]
        if mn == 'byte':
            b = 1
        elif mn == 'cmp':
            parts = s.split()
            b = 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
        elif mn in two_byte_set:
            b = 2
        else:
            b = 1
        if section == 'code': code_bytes += b
        elif section == 'data': data_bytes += b
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

    print("\n── MK1 Memory Report ──", file=sys.stderr)
    if overlay_mode:
        ov_kernel = gen._overlay_kernel_size
        ov_region = gen._overlay_region
        ov_shared = gen._overlay_shared_size
        init_only = code_bytes - copy_loop
        print(f"  Mode:       two-stage boot (overlay system)", file=sys.stderr)
        print(f"  Stage 1:    {init_only}B init + {copy_loop}B copy = {code_bytes}B", file=sys.stderr)
        print(f"  Stage 2:    {ov_kernel}B kernel + {ov_shared}B shared = {ov_region}B", file=sys.stderr)
        runtime_free = MAX_CODE - ov_region
        print(f"  Page 0:     {ov_region}B used, {runtime_free}B overlay region", file=sys.stderr)
    elif init_extraction:
        init_only = code_bytes - copy_loop
        total = max(init_only, kernel_bytes) + copy_loop
        print(f"  Mode:       two-stage boot (init extraction)", file=sys.stderr)
        print(f"  Stage 1:    {init_only}B init + {copy_loop}B copy = {code_bytes}B / {MAX_CODE}B", file=sys.stderr)
        print(f"  Stage 2:    {kernel_bytes}B kernel  (runtime code page)", file=sys.stderr)
        runtime_free = MAX_CODE - kernel_bytes
        print(f"  Page 0:     {kernel_bytes}B used, {runtime_free}B free at runtime", file=sys.stderr)
    else:
        print(f"  Mode:       single-stage (flat)", file=sys.stderr)
        print(f"  Page 0:     {code_bytes}B / {MAX_CODE}B code", file=sys.stderr)

    data_runtime = gen.data_alloc
    n_globals = len([n for n in gen.globals if not n.startswith('_')])
    data_temp = data_bytes - gen.data_alloc if data_bytes > gen.data_alloc else 0
    data_desc = f"{n_globals} globals" if n_globals else "allocated"
    print(f"  Page 1:     {data_runtime}B {data_desc}, {256 - data_runtime}B free  (data page)", file=sys.stderr)

    print(f"  Page 2:     stack (grows down from 0xFF)", file=sys.stderr)

    if overlay_mode:
        print(f"  Page 3:     {p3_bytes}B / 256B  (kernel + shared + manifest)", file=sys.stderr)
    elif init_extraction:
        p3_free = 256 - kernel_bytes
        print(f"  Page 3:     {kernel_bytes}B kernel image, {p3_free}B free", file=sys.stderr)
        print(f"              (fully reusable at runtime via derefp3/iderefp3)", file=sys.stderr)
    elif p3_bytes > 0:
        print(f"  Page 3:     {p3_bytes}B / 256B", file=sys.stderr)
    else:
        print(f"  Page 3:     unused (256B available)", file=sys.stderr)

    if eeprom_bytes > 0:
        print(f"  EEPROM:     {eeprom_payload}B used, {4096 - eeprom_payload}B free  (AT24C32)", file=sys.stderr)
    else:
        print(f"  EEPROM:     unused  (4096B available)", file=sys.stderr)

    if init_extraction:
        total_free = (MAX_CODE - kernel_bytes) + (256 - data_runtime) + 256 + (4096 - eeprom_payload)
        print(f"  Total free: {total_free}B  (code {MAX_CODE-kernel_bytes} + data {256-data_runtime} + page3 256 + eeprom {4096-eeprom_payload})", file=sys.stderr)

    # Warnings — init stage can use up to 254B (runs once, self-copies kernel)
    init_limit = 254 if (init_extraction or overlay_mode) else MAX_CODE
    if code_bytes > init_limit:
        print(f"  !! CODE OVERFLOW: {code_bytes}B > {init_limit}B limit", file=sys.stderr)
    elif code_bytes > init_limit - 3:
        print(f"  !! Code page tight: only {init_limit - code_bytes}B free", file=sys.stderr)
    if gen.data_alloc > 256:
        print(f"  !! DATA OVERFLOW: {gen.data_alloc}B > 256B", file=sys.stderr)

    print("───────────────────────", file=sys.stderr)

if __name__ == '__main__':
    main()
