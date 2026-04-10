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
            'void', 'int', 'switch', 'case', 'default', 'break', 'continue'}

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
            typ = self.parse_type()
            name = self.expect('IDENT').value
            if self.match('('):
                fn = self.parse_function(typ, name)
                if fn: functions.append(fn)
            elif self.match('='):
                val = self.expect('NUMBER').value; self.expect(';')
                globals_.append((name, val))
            elif self.match('['):
                size = self.expect('NUMBER').value; self.expect(']'); self.expect(';')
                globals_.append((name, [0]*size))
            elif self.match(';'):
                globals_.append((name, 0))
            else: raise SyntaxError(f'Line {self.peek().line}: unexpected after {name}')
        return globals_, functions

    def parse_type(self):
        if self.match('void'): return 'void'
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
        if t.value in ('unsigned', 'char', 'int'): return self.parse_local_decl()
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
        if (',$b' in s and s.startswith('mov')) or s.startswith('pop $b') or s == 'swap' or s.startswith('jal'):
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
                'i2c_ack', 'i2c_nack', 'i2c_wait_ack',
                'ora_imm', 'orb_imm', 'ddra_imm',
                'write_code', 'call_code', 'eeprom_read_to_code',
                'peek3', 'poke3'}
    # NOTE: i2c_send_byte, i2c_read_byte, eeprom_write_byte, eeprom_read_byte,
    # rtc_read_seconds, rtc_read_temp, lcd_cmd, lcd_char, lcd_init are NOT builtins —
    # they use jal to subroutines that clobber B, C, D. The register
    # allocator must treat them as user function calls.

    def _has_calls(self, node):
        """Check if AST node contains any non-builtin function calls (jal)."""
        if not isinstance(node, tuple) or len(node) == 0:
            return False
        if node[0] == 'call' and node[1] not in self.BUILTINS:
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
        for name in unique:
            if not c_var:
                c_var = name
            elif not d_var:
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

        for name, init in globals_:
            size = len(init) if isinstance(init, list) else 1
            if self.data_alloc + size <= 256:
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

        # Note: I2C helpers for EEPROM overlay tier are only registered if
        # _eeprom_overlay_partition() is actually called (when page 3 overflows).
        # The page 3 overlay tier doesn't need I2C at runtime.

        # Emit I2C/LCD helpers AFTER all functions — so all helpers needed
        # by any function are registered. Protected from overlay by _NO_OVERLAY set.
        self._emit_i2c_helpers()

        # Dead function elimination: remove functions that are never called
        self._eliminate_dead_functions()

        # Overlay partitioning
        if getattr(self, 'eeprom_mode', False):
            # EEPROM mode: use page 3 overlay system first (fast, big overlay region).
            # Force overlaying even if code fits in 256 bytes.
            self._overlay_partition()
            # If page 3 still isn't enough, use EEPROM I2C tier as fallback.
            # TODO: self._eeprom_overlay_partition() for overflow
        else:
            # Standard: move cold functions to page 3 if code > 256 bytes
            self._overlay_partition()

        # Emit page 1 globals
        page1_vars = [(n, i) for n, i in globals_ if n in self.globals]
        if page1_vars:
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
        page3_vars = [(n, i) for n, i in globals_ if n in self.page3_globals]
        if page3_vars:
            self.emit('\tsection page3')
            for name, init in page3_vars:
                self.emit(f'_{name}:')
                if isinstance(init, list):
                    for v in init:
                        self.emit(f'\tbyte {v}')
                else:
                    self.emit(f'\tbyte {init}')
            self.emit('\tsection code')

        # Emit LCD init data table in page 3 (if lcd_init was used)
        if hasattr(self, '_lcd_init_p3_data'):
            p3_base, data = self._lcd_init_p3_data
            self.emit('\tsection page3')
            self.emit(f'; LCD init I2C sequence at page3 offset {p3_base}')
            for b in data:
                self.emit(f'\tbyte {b}')
            self.emit('\tsection code')

        # Emit lcd_print string data in page 3
        if hasattr(self, '_lcd_print_strings'):
            self.emit('\tsection page3')
            for p3_off, s in self._lcd_print_strings:
                self.emit(f'; string at page3 offset {p3_off}')
                for ch in s:
                    self.emit(f'\tbyte {ord(ch)}')
                self.emit('\tbyte 0')  # null terminator
            self.emit('\tsection code')

        # Emit note table data in page 3
        if hasattr(self, '_note_table'):
            self.emit('\tsection page3')
            for p3_off, ratio, cyc_lo, cyc_hi in self._note_table:
                self.emit(f'; note at page3 offset {p3_off}: ratio={ratio} cyc={cyc_hi}:{cyc_lo}')
                self.emit(f'\tbyte {ratio}')
                self.emit(f'\tbyte {cyc_lo}')
                self.emit(f'\tbyte {cyc_hi}')
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
            self.emit('__i2c_rb:')
            self.emit('\tldi $d,0')
            self.emit('\tldi $c,8')
            self.emit(f'{lbl_rb}:')
            self.emit('\tddrb_imm 0x00')   # SCL HIGH
            self.emit('\texrw 0')          # A = port B
            self.emit('\tandi 0x01,$a')    # isolate SDA
            self.emit('\tmov $a,$b')       # B = SDA bit
            self.emit('\tmov $d,$a')       # A = accumulated byte
            self.emit('\tsll')             # shift left
            self.emit('\tor $b,$a')        # OR in SDA bit
            self.emit('\tmov $a,$d')       # D = result
            self.emit('\tddrb_imm 0x02')   # SCL LOW
            self.emit('\tmov $c,$a')
            self.emit('\tdec')
            self.emit('\tmov $a,$c')
            self.emit(f'\tjnz {lbl_rb}')
            self.emit('\tret')

        if '__lcd_init' in helpers:
            # Data-driven LCD init: store I2C byte sequence in page 3,
            # use compact loop to send. Keeps overlay small (~35B vs ~85B).
            # Sentinels: 0xFE=START, 0xFD=STOP, 0xFF=END, else=send byte
            START, STOP, END = 0xFE, 0xFD, 0xFF
            lcd_init_data = []
            # No PCF reset (0x00) — backlight defaults to on, reset turns it off
            BL = 0x08  # backlight bit — keep on throughout init
            for _ in range(3):
                lcd_init_data += [START, 0x34|BL, 0x30|BL, STOP]   # nibble 0x03 x3
            lcd_init_data += [START, 0x24|BL, 0x20|BL, STOP]       # nibble 0x02
            for cmd in [0x28, 0x0C, 0x01, 0x06]:             # HD44780 commands
                hi = cmd & 0xF0
                lo = (cmd & 0x0F) << 4
                lcd_init_data += [START, hi|0x04|BL, hi|BL, lo|0x04|BL, lo|BL, STOP]
            lcd_init_data.append(END)

            p3_base = self.page3_alloc
            self.page3_alloc += len(lcd_init_data)
            self._lcd_init_p3_data = (p3_base, lcd_init_data)

            lbl_loop = self.label('li_lp')
            lbl_ns = self.label('li_ns')
            lbl_send = self.label('li_sd')
            lbl_adv = self.label('li_av')
            lbl_done = self.label('li_dn')

            self.emit('__lcd_init:')
            self.emit(f'\tldi $d,{p3_base}')
            self.emit(f'{lbl_loop}:')
            self.emit('\tmov $d,$a')
            self.emit('\tderefp3')
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
            self.emit(f'\tjnz {lbl_send}')
            self.emit('\tjal __i2c_sp')
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
        if '__lcd_chr' in helpers:
            self.emit('__lcd_chr:')
            self.emit('\tpush $a')        # save caller's A (will be discarded)
            self.emit('\tldi $a,0x09')    # flags: RS + BL
            self.emit('\tj __lcd_send')

        if '__lcd_cmd' in helpers:
            self.emit('__lcd_cmd:')
            self.emit('\tpush $a')
            self.emit('\tldi $a,0x00')    # flags: none

        if '__lcd_cmd' in helpers or '__lcd_chr' in helpers:
            self.emit('__lcd_send:')
            self.emit('\tpush $a')        # push flags to stack
            self.emit('\tjal __i2c_st')   # START + addr
            # High nibble + EN
            self.emit('\tmov $d,$a')
            self.emit('\tandi 0xF0,$a')
            self.emit('\tpop $b')         # B = flags
            self.emit('\tpush $b')
            self.emit('\tor $b,$a')
            self.emit('\tori 0x04,$a')    # + EN
            self.emit('\tjal __i2c_sb')
            # High nibble - EN
            self.emit('\tmov $d,$a')
            self.emit('\tandi 0xF0,$a')
            self.emit('\tpop $b')
            self.emit('\tpush $b')
            self.emit('\tor $b,$a')
            self.emit('\tjal __i2c_sb')
            # Low nibble + EN
            self.emit('\tmov $d,$a')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tandi 0xF0,$a')
            self.emit('\tpop $b')
            self.emit('\tpush $b')
            self.emit('\tor $b,$a')
            self.emit('\tori 0x04,$a')
            self.emit('\tjal __i2c_sb')
            # Low nibble - EN
            self.emit('\tmov $d,$a')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tsll')
            self.emit('\tandi 0xF0,$a')
            self.emit('\tpop $b')
            self.emit('\tor $b,$a')
            self.emit('\tjal __i2c_sb')
            self.emit('\tjal __i2c_sp')
            self.emit('\tpop $a')         # balance the push at entry
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
            self.emit('\tldi $b,0')
            self.emit('\tideref')          # data[0] = ipm
            # Disable SQW to prevent coupling into SDA
            self.emit('\texrw 2')
            self.emit('\tddrb_imm 0x01')
            self.emit('\tddrb_imm 0x03')
            self.emit('\tldi $a,0xD0')
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x0E')
            self.emit('\tjal __i2c_sb')
            self.emit('\tldi $a,0x1C')     # INTCN=1, SQW disabled
            self.emit('\tjal __i2c_sb')
            self.emit('\tjal __i2c_sp')
            self.emit('\tclr $a')
            self.emit('\texw 0 3')         # clear DDRA
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
            # Multiply B × data[0]: result in D:C (hi:lo)
            self.emit('\tclr $a')
            self.emit('\tderef')           # A = ipm from data[0]
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
        # Reads ipm from data[0]. Clobbers A, B, D.
        if '__tone_setup' in helpers:
            lbl_mul = self.label('tsmul')
            lbl_noc = self.label('tsnoc')
            self.emit('__tone_setup:')
            self.emit('\tclr $a')
            self.emit('\tderef')           # A = ipm from data[0]
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
            # Result = (D << 4) | (A >> 4). Non-overlapping nibbles, so add works.
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
            # Store dest in data[6], count in data[7]
            self.emit('\tmov $b,$a')         # A = dest
            self.emit('\tldi $b,6')
            self.emit('\tideref')            # data[6] = dest
            self.emit('\tldsp 2')            # A = count (past ret addr)
            self.emit('\tldi $b,7')
            self.emit('\tideref')            # data[7] = count
            # Read loop
            self.emit(f'{lbl_loop}:')
            self.emit('\tjal __i2c_rb')      # D = byte, A/B/C clobbered
            # Write byte to code page: istc needs A=byte, B=dest
            self.emit('\tmov $d,$a')         # A = byte
            self.emit('\tpush $a')           # save byte
            self.emit('\tldi $a,6')
            self.emit('\tderef')             # A = data[6] = dest
            self.emit('\tmov $a,$b')         # B = dest
            self.emit('\tpop $a')            # A = byte
            self.emit('\tistc')              # code[B] = A
            # Increment dest in data[6]
            self.emit('\tmov $b,$a')         # A = dest
            self.emit('\tinc')
            self.emit('\tldi $b,6')
            self.emit('\tideref')            # data[6] = dest + 1
            # Decrement count in data[7], branch on zero
            self.emit('\tldi $a,7')
            self.emit('\tderef')             # A = data[7] = remaining
            self.emit('\tdec')               # A = remaining - 1, ZF set
            self.emit('\tldi $b,7')
            self.emit('\tideref')            # data[7] = remaining - 1 (ZF preserved: no FI)
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

        # __play_note: A = page 3 offset into note table.
        # Reads [ratio, cyc_lo, cyc_hi] from page 3.
        # If ratio=0 and __delay_Nms available, plays silence (cyc_lo = ms).
        # Otherwise calls __tone_setup + __tone.
        if '__play_note' in helpers:
            has_silence = '__delay_Nms' in helpers
            lbl_sil = self.label('pnsil') if has_silence else None
            self.emit('__play_note:')
            self.emit('\tmov $a,$d')       # D = base offset (preserved)
            self.emit('\tderefp3')         # A = page3[offset] = ratio
            if has_silence:
                self.emit('\ttst 0xFF')
                self.emit(f'\tjz {lbl_sil}')   # ratio=0 → silence
            self.emit('\tmov $a,$b')       # B = ratio
            self.emit('\tpush $d')         # save base offset
            self.emit('\tjal __tone_setup') # C = half_period (clobbers A,B,D)
            self.emit('\tpop $a')          # A = base offset
            self.emit('\tinc')
            self.emit('\tmov $a,$d')       # D = offset+1
            self.emit('\tderefp3')         # A = page3[offset+1] = cyc_lo
            self.emit('\tpush $a')         # save cyc_lo
            self.emit('\tmov $d,$a')       # A = offset+1
            self.emit('\tinc')
            self.emit('\tderefp3')         # A = page3[offset+2] = cyc_hi
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
                self.emit('\tderefp3')         # A = page3[offset+1] = ms
                self.emit('\tmov $a,$b')       # B = ms
                self.emit('\tjal __delay_Nms')
                self.emit('\tret')

    def _overlay_partition(self):
        """If code exceeds page 0 capacity, move cold functions to page 3.
        Generates overlay loader at the start and rewrite calls to use ocall."""
        # Estimate total code size
        two_byte = {'ldsp','stsp','push_imm','jal','jc','jz','jnc','jnz','j','ldi',
                    'cmp','addi','subi','andi','ori','ld','st','ldsp_b','ldp3','stp3',
                    'setjmp','ocall','tst','out_imm'}
        size = 0
        for line in self.code:
            s = line.strip()
            if not s or s.endswith(':'):
                continue
            mn = s.split()[0]
            if mn == 'cmp':
                parts = s.split()
                size += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
            elif mn in two_byte:
                size += 2
            else:
                size += 1

        # Only partition if code exceeds 240 bytes (leaving room for overlay loader)
        # In EEPROM mode, always partition to move functions to overlays
        if size <= 240 and not getattr(self, 'eeprom_mode', False):
            return

        # Find functions and their sizes (exclude _main)
        funcs = []
        i = 0
        while i < len(self.code):
            line = self.code[i]
            s = line.strip()
            # Don't overlay: _main, critical I2C helpers that are called from overlayed code
            _NO_OVERLAY = {'_main:', '__i2c_sb:', '__i2c_st:', '__i2c_st_only:', '__i2c_sp:',
                           '__lcd_chr:', '__lcd_cmd:', '__lcd_send:',
                           '__delay_cal:', '__delay_Nms:', '__i2c_rb:',
                           '__eeprom_r2c_loop:', '__eeprom_dispatch:', '__eeprom_load:',
                           '_overlay_load:'}
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

        # Sort by size descending — move largest functions to overlay first
        funcs.sort(key=lambda f: -f[3])

        # Move functions to page 3 until code fits in 220 bytes (room for loader)
        overlay_funcs = []
        remaining_size = size
        overlay_loader_size = 20  # approximate

        if getattr(self, 'eeprom_mode', False):
            # EEPROM mode: overlay ALL non-main, non-helper functions
            overlay_funcs = list(funcs)
            remaining_size = size - sum(f[3] for f in funcs)
        else:
            for name, start, end, fsize in funcs:
                if remaining_size <= (240 - overlay_loader_size):
                    break
                overlay_funcs.append((name, start, end, fsize))
                remaining_size -= fsize

        if not overlay_funcs:
            return

        # ── Overlay system ──
        # Overlay functions are assembled at OVERLAY_REGION in page 0 (so
        # internal labels resolve correctly). The assembler's page3_code
        # section stores instruction bytes in page 3.
        # At runtime, the overlay loader copies them to OVERLAY_REGION
        # and calls via jal_r.
        #
        # Page 3 layout: [metadata: 3 bytes/overlay] [code bytes]

        # OVERLAY_REGION must satisfy two constraints:
        # 1. OVERLAY_REGION >= resident_code_size (don't overlap resident code)
        # 2. OVERLAY_REGION + max_overlay_size <= 256 (overlay must fit)
        overlay_loader_size_actual = 27
        max_overlay_size = max(fsize for _, _, _, fsize in overlay_funcs)
        non_overlay_size = remaining_size + overlay_loader_size_actual
        OVERLAY_REGION = 256 - max_overlay_size
        if OVERLAY_REGION < non_overlay_size + 2:
            # Can't fit — resident code + largest overlay > 256
            import sys
            print(f"WARNING: overlay won't fit. Resident={non_overlay_size}B, "
                  f"largest overlay={max_overlay_size}B, total={non_overlay_size+max_overlay_size}B > 256",
                  file=sys.stderr)
        META_SIZE = 3

        overlay_meta = []
        overlay_asm_blocks = []

        for idx, (name, start, end, fsize) in enumerate(overlay_funcs):
            overlay_asm_blocks.append((name, self.code[start:end], fsize))
            overlay_meta.append((idx, name, fsize))

        # Remove overlay functions and rewrite calls to ocall
        remove_ranges = {(start, end) for _, start, end, _ in overlay_funcs}
        new_code = []
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
                for idx, name, _ in overlay_meta:
                    if s == f'jal {name}':
                        # Save A and B (function args) to data page before overlay load
                        # Must save B first (ldi $b,N clobbers B)
                        new_code.append(f'\tpush $a')          # save A temporarily
                        new_code.append(f'\tmov $b,$a')        # A = B (arg2)
                        new_code.append(f'\tldi $b,253')
                        new_code.append(f'\tideref')           # data[253] = arg2
                        new_code.append(f'\tpop $a')           # A = arg1 (restored)
                        new_code.append(f'\tldi $b,252')
                        new_code.append(f'\tideref')           # data[252] = arg1
                        new_code.append(f'\tldi $a,{idx}')
                        new_code.append(f'\tjal _overlay_load')
                        rewritten = True
                        break
                if not rewritten:
                    new_code.append(line)
                i += 1

        # ── Inline helpers only used by overlay code ──
        # If a _NO_OVERLAY helper is never called from resident code,
        # inline its body into each overlay that calls it and remove from resident.
        overlay_names = {name for name, _, _, _ in overlay_funcs}
        # Find helper bodies in new_code (resident)
        helper_bodies = {}
        hi = 0
        while hi < len(new_code):
            hs = new_code[hi].strip()
            if hs.endswith(':') and hs.startswith('__') and not hs.startswith('.'):
                hname = hs[:-1]
                hstart = hi
                hi += 1
                hlines = []
                while hi < len(new_code):
                    ns = new_code[hi].strip()
                    if ns.endswith(':') and not ns.startswith('.') and ns.startswith('_'):
                        break
                    hlines.append(new_code[hi])
                    hi += 1
                helper_bodies[hname] = (hstart, hi, hlines)
            else:
                hi += 1

        for hname, (hstart, hend, hlines) in list(helper_bodies.items()):
            # Check if any resident code (non-label, non-helper-body) calls this helper
            called_from_resident = False
            for ci, cline in enumerate(new_code):
                if ci >= hstart and ci < hend:
                    continue  # skip the helper's own body
                cs = cline.strip()
                # Check for any reference: jal, j, or other use of the helper name
                if hname in cs and cs != f'{hname}:':
                    called_from_resident = True
                    break
            if not called_from_resident:
                # Inline into each overlay block and remove from resident
                # Replace 'jal hname' + 'ret' pattern: expand body minus final ret
                inline_body = [l for l in hlines if l.strip() != 'ret']
                for oi, (oname, olines, ofsize) in enumerate(overlay_asm_blocks):
                    new_olines = []
                    for oline in olines:
                        if oline.strip() == f'jal {hname}':
                            new_olines.extend(inline_body)
                        else:
                            new_olines.append(oline)
                    # Recompute size
                    new_fsize = 0
                    for nl in new_olines:
                        ns = nl.strip()
                        if not ns or ns.endswith(':'): continue
                        mn = ns.split()[0]
                        if mn in two_byte: new_fsize += 2
                        elif mn == 'cmp':
                            parts = ns.split()
                            new_fsize += 1 if (len(parts)>1 and parts[1].startswith('$')) else 2
                        else: new_fsize += 1
                    overlay_asm_blocks[oi] = (oname, new_olines, new_fsize)
                # Remove from resident
                for ri in range(hstart, hend):
                    new_code[ri] = ''  # blank out

        # Clean blanked lines
        new_code = [l for l in new_code if l != '']

        # Rebuild overlay_meta with updated sizes
        overlay_meta = [(idx, name, fsize) for idx, (name, _, fsize) in enumerate(overlay_asm_blocks)]

        # ── Optimized overlay loader ──
        # Metadata: 2 bytes per overlay (src_offset, length). Entry is always
        # OVERLAY_REGION. Index*2 instead of index*3 for metadata lookup.
        # Loop uses cmp $c to check dst against precomputed end address,
        # eliminating the separate counter register.
        META_SIZE = 2
        p3_used = self.page3_alloc  # page 3 bytes already used by globals + LCD init

        loader = [
            '; ── Overlay loader ──',
            '_overlay_load:',
            '\tsll',                        # A = index * 2 = metadata offset
        ]
        if p3_used > 0:
            loader.append(f'\taddi {p3_used},$a')  # skip past pre-existing page 3 data
        loader += [
            '\tpush $a',                    # save meta_offset
            '\tderefp3',                    # A = page3[offset] = src_offset
            '\tmov $a,$d',                  # D = src (read pointer)
            '\tpop $a',                     # A = meta_offset
            '\tinc',
            '\tderefp3',                    # A = page3[offset+1] = length
            f'\taddi {OVERLAY_REGION},$a',  # A = OVERLAY_REGION + length = end addr
            '\tmov $a,$c',                  # C = end address
            f'\tldi $b,{OVERLAY_REGION}',   # B = dst (write pointer)
            '.copy:',
            '\tmov $d,$a',                  # A = src
            '\tderefp3',                    # A = page3[src]
            '\tistc',                       # code[B] = A
            '\tmov $d,$a',
            '\tinc',
            '\tmov $a,$d',                  # D++ (src)
            '\tmov $b,$a',
            '\tinc',
            '\tmov $a,$b',                  # B++ (dst)
            '\tcmp $c',                     # dst == end?
            '\tjnz .copy',                  # no: keep copying
            # Restore function args (saved to data page by caller) before jump
            '\tldi $a,253',
            '\tderef',                      # A = data[253] = arg2
            '\tmov $a,$b',                  # B = arg2
            '\tldi $a,252',
            '\tderef',                      # A = data[252] = arg1
            f'\tj {OVERLAY_REGION}',           # jump to overlay with correct A,B
        ]

        # Main must be at address 0 (CPU starts there).
        # Loader goes after main; ocall replaced with ldi+jal.
        self.code = new_code + loader

        # ── Page 3: metadata (2 bytes/overlay) + overlay code ──
        # Account for page 3 space already used by globals + LCD init data
        p3_used = self.page3_alloc
        meta_table_size = len(overlay_meta) * META_SIZE
        p3_offset = p3_used + meta_table_size

        # Pad code section up to OVERLAY_REGION so labels resolve at correct addresses.
        # The assembler's PC must be at OVERLAY_REGION when overlay labels are defined.
        # Emit HLT padding bytes to advance the PC.
        resident_estimate = sum(
            2 if s.strip().split()[0] in two_byte else 1
            for s in (new_code + loader) if s.strip() and not s.strip().endswith(':')
            and not s.strip().startswith(';') and not s.strip().startswith('.')
        )
        pad_needed = OVERLAY_REGION - resident_estimate
        if pad_needed > 0:
            # Use .org to advance PC to OVERLAY_REGION (assembler pads with HLT)
            self.code.append(f'\torg {OVERLAY_REGION}')

        # Overlay code assembled into page 3 via page3_code section.
        # Labels resolve at OVERLAY_REGION+ addresses because the code PC
        # has been advanced to OVERLAY_REGION by the padding above.
        for name, asm_lines, fsize in overlay_asm_blocks:
            self.code.append('\tsection page3_code')
            self.code.extend(asm_lines)

        # Emit page 3 metadata table (AFTER padding, so it's in section page3)
        self.code.append('\tsection page3')
        p3_used = self.page3_alloc
        meta_table_size = len(overlay_meta) * META_SIZE
        p3_offset = p3_used + meta_table_size
        for idx, name, fsize in overlay_meta:
            self.code.append(f'\tbyte {p3_offset}')   # absolute offset in page 3
            self.code.append(f'\tbyte {fsize}')        # length
            p3_offset += fsize

        self.code.append('\tsection code')

    def _eeprom_overlay_partition(self):
        """EEPROM overlay mode: partition code for EEPROM-backed execution.

        Architecture:
        - Resident kernel: VIA init, I2C helpers, overlay dispatcher+loader, main wrapper
        - Page 3 tier (L1): hot overlays pre-loaded from EEPROM at boot, fast copy to code page
        - EEPROM tier (L2): cold overlays loaded via I2C on demand
        - Overlay caching: data[255] tracks current overlay ID, skip reload if cached

        The dispatcher checks cache → page 3 → EEPROM, loads to code page, calls.
        """
        import sys, json

        EEPROM_BASE = getattr(self, 'eeprom_base', 0x0200)
        OV_CACHE_SLOT = 255    # data[255] = current overlay ID
        OV_ENTRY_SLOT = 254    # data[254] = entry offset within overlay

        # ── Step 1: Measure function sizes ──
        two_byte = {'ldsp','stsp','push_imm','jal','jc','jz','jnc','jnz','j','ldi',
                    'cmp','addi','subi','andi','ori','ld','st','ldsp_b','ldp3','stp3',
                    'setjmp','ocall','tst','out_imm','ddrb_imm','ora_imm','ddra_imm',
                    'orb_imm','exw','exrw','ldi_b','ldi_c','ldi_d'}

        def measure_size(lines):
            size = 0
            for line in lines:
                s = line.strip()
                if not s or s.endswith(':'): continue
                mn = s.split()[0]
                if mn == 'cmp':
                    parts = s.split()
                    size += 1 if (len(parts) > 1 and parts[1].startswith('$')) else 2
                elif mn in two_byte:
                    size += 2
                else:
                    size += 1
            return size

        # ── Step 2: Find functions and their code ──
        # _main and I2C helpers are always resident
        _ALWAYS_RESIDENT = {'_main:', '__i2c_sb:', '__i2c_st:', '__i2c_st_only:',
                            '__i2c_sp:', '__i2c_rb:', '__eeprom_r2c_loop:',
                            '__delay_cal:', '__delay_Nms:',
                            '__lcd_chr:', '__lcd_cmd:', '__lcd_send:',
                            '__tone_setup:', '__tone:', '__play_note:'}

        funcs = []
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
                func_lines = self.code[start:i]
                func_size = measure_size(func_lines)
                is_resident = s in _ALWAYS_RESIDENT
                funcs.append({
                    'name': name, 'start': start, 'end': i,
                    'lines': func_lines, 'size': func_size,
                    'resident': is_resident
                })
            else:
                i += 1

        # ── Step 3: Build call graph ──
        call_graph = {}  # name → set of names it calls
        for f in funcs:
            calls = set()
            for line in f['lines']:
                s = line.strip()
                if s.startswith('jal ') or s.startswith('j '):
                    target = s.split()[1]
                    if target.startswith('_') and not target.startswith('.'):
                        calls.add(target)
            call_graph[f['name']] = calls

        # ── Step 4: Count call frequency for each function ──
        call_freq = {}
        for caller, callees in call_graph.items():
            for callee in callees:
                call_freq[callee] = call_freq.get(callee, 0) + 1

        # ── Step 5: Find connected components among non-resident functions ──
        # Functions that call each other MUST be in the same overlay
        overlay_funcs = [f for f in funcs if not f['resident']]
        if not overlay_funcs:
            return  # everything fits in resident

        total_size = sum(f['size'] for f in funcs)
        resident_size = sum(f['size'] for f in funcs if f['resident'])

        # I2C + dispatcher overhead
        # __i2c_sb=40, __i2c_rb=23, __eeprom_r2c_loop=54, VIA init=14,
        # dispatcher=~25, loader=~45, main wrapper=~12
        KERNEL_OVERHEAD = 40 + 23 + 54 + 14 + 25 + 45 + 12  # ~213 bytes
        # But many of these are already in resident_size (counted above)
        # The actual kernel overhead is what we ADD beyond the user's resident code

        # In EEPROM mode, always partition if there are non-resident functions.
        # The point is to move user code to EEPROM even if it would fit.
        if not overlay_funcs:
            return  # nothing to overlay

        # Build adjacency for connected components
        ov_names = {f['name'] for f in overlay_funcs}
        adj = {f['name']: set() for f in overlay_funcs}
        for f in overlay_funcs:
            for callee in call_graph.get(f['name'], set()):
                if callee in ov_names:
                    adj[f['name']].add(callee)
                    adj[callee].add(f['name'])

        # Find connected components (BFS)
        visited = set()
        components = []
        for f in overlay_funcs:
            if f['name'] not in visited:
                comp = []
                queue = [f['name']]
                while queue:
                    n = queue.pop(0)
                    if n in visited: continue
                    visited.add(n)
                    comp.append(n)
                    for nb in adj.get(n, set()):
                        if nb not in visited:
                            queue.append(nb)
                components.append(comp)

        # ── Step 6: Compute overlay region size (two-pass) ──
        # First pass: use a conservative estimate, generate everything,
        # measure actual resident size, then fix up OVERLAY_REGION_START.
        OVERLAY_REGION_START = 0xC0  # conservative first-pass estimate
        OVERLAY_SIZE = 256 - OVERLAY_REGION_START  # will be recomputed in pass 2

        # ── Step 7: Bin-pack components into overlay slots ──
        func_by_name = {f['name']: f for f in funcs}
        overlay_slots = []

        # Sort components by total size descending (largest first = better packing)
        comp_sizes = []
        for comp in components:
            total = sum(func_by_name[n]['size'] for n in comp)
            comp_sizes.append((total, comp))
        comp_sizes.sort(key=lambda x: -x[0])

        for comp_size, comp in comp_sizes:
            if comp_size > OVERLAY_SIZE:
                print(f"WARNING: function group {comp} ({comp_size}B) exceeds overlay size ({OVERLAY_SIZE}B)",
                      file=sys.stderr)
            # Try to fit into an existing slot
            placed = False
            for slot in overlay_slots:
                slot_used = sum(func_by_name[n]['size'] for n in slot)
                if slot_used + comp_size <= OVERLAY_SIZE:
                    slot.extend(comp)
                    placed = True
                    break
            if not placed:
                overlay_slots.append(list(comp))

        # ── Step 8: Assign EEPROM addresses and entry points ──
        eeprom_addr = EEPROM_BASE
        overlay_info = []
        for slot_id, slot_funcs in enumerate(overlay_slots):
            slot_size = 0
            entries = {}
            for fname in slot_funcs:
                entries[fname] = OVERLAY_REGION_START + slot_size
                slot_size += func_by_name[fname]['size']
            overlay_info.append({
                'id': slot_id,
                'functions': slot_funcs,
                'entries': entries,  # fname → code page address
                'size': slot_size,
                'eeprom_addr': eeprom_addr,
            })
            eeprom_addr += slot_size

        # ── Step 9: Generate resident code ──
        # Remove overlay functions from code, rewrite calls to dispatcher

        # Build function→overlay mapping
        func_to_overlay = {}
        for ov in overlay_info:
            for fname in ov['functions']:
                func_to_overlay[fname] = (ov['id'], ov['entries'][fname])

        # Rewrite code: replace jal to overlay functions with dispatch calls
        new_code = []
        i = 0
        ov_func_ranges = {}
        for f in overlay_funcs:
            ov_func_ranges[(f['start'], f['end'])] = f['name']

        ii = 0
        while ii < len(self.code):
            # Skip overlay function bodies
            skip = False
            for (rs, re), fname in ov_func_ranges.items():
                if rs <= ii < re:
                    ii = re
                    skip = True
                    break
            if skip:
                continue

            line = self.code[ii]
            s = line.strip()

            # Rewrite jal to overlay functions
            rewritten = False
            if s.startswith('jal '):
                target = s.split()[1]
                if target in func_to_overlay:
                    ov_id, entry_addr = func_to_overlay[target]
                    new_code.append(f'\tldi $a,{ov_id}')
                    new_code.append(f'\tldi $b,{entry_addr}')
                    new_code.append(f'\tjal __eeprom_dispatch')
                    rewritten = True

            if not rewritten:
                new_code.append(line)
            ii += 1

        # ── Step 10: Generate dispatcher ──
        dispatcher = [
            '; ── EEPROM overlay dispatcher ──',
            '; A = overlay_id, B = entry_addr',
            '__eeprom_dispatch:',
            '\tmov $a,$c',          # C = overlay_id
            '\tmov $b,$a',          # A = entry_addr
            f'\tldi $b,{OV_ENTRY_SLOT}',
            '\tideref',             # data[254] = entry_addr
            # Cache check: compare data[255] with C
            f'\tldi $a,{OV_CACHE_SLOT}',
            '\tderef',              # A = data[255] = cached_id
            '\tcmp $c',             # cached == overlay_id?
            '\tjz .ee_cached',
            # Cache miss: update cache, load overlay
            '\tmov $c,$a',          # A = overlay_id
            f'\tldi $b,{OV_CACHE_SLOT}',
            '\tideref',             # data[255] = overlay_id
            '\tjal __eeprom_load',
            '.ee_cached:',
            f'\tldi $a,{OV_ENTRY_SLOT}',
            '\tderef',              # A = data[254] = entry_addr
            '\tjal_r',              # call overlay at entry_addr
            '\tret',
        ]

        # ── Step 11: Generate EEPROM loader ──
        # A = overlay_id. Reads overlay directory from page 3.
        # Directory: 4 bytes per overlay [size, eeprom_hi, eeprom_lo, pad]
        # Page 3 offset = page3_alloc + id * 4
        p3_dir_base = self.page3_alloc
        DIR_ENTRY_SIZE = 3  # size, hi, lo
        loader = [
            '; ── EEPROM overlay loader ──',
            '; A = overlay_id. Loads overlay from EEPROM to code page.',
            '__eeprom_load:',
            # Compute page 3 directory offset: A * 3 + base
            '\tmov $a,$d',          # D = id
            '\tsll',                # A = id * 2
            '\tadd $d,$a',          # A = id * 3
        ]
        if p3_dir_base > 0:
            loader.append(f'\taddi {p3_dir_base},$a')
        loader += [
            '\tmov $a,$d',          # D = base offset
            # Read size
            '\tderefp3',            # A = page3[offset] = size
            '\tpush $a',            # save size (for r2c_loop count)
            # Read eeprom_hi
            '\tmov $d,$a',          # A = base
            '\tinc',
            '\tderefp3',            # A = eeprom_hi
            '\tmov $a,$c',          # C = hi
            # Read eeprom_lo
            '\tmov $d,$a',          # A = base
            '\tinc',
            '\tinc',
            '\tderefp3',            # A = eeprom_lo
            # Save lo, do I2C setup
            '\tpush $a',            # save lo
            '\tpush $a',            # save lo again (will use for i2c_sb)
            # I2C START + device write address
            '\tddrb_imm 0x01',
            '\tddrb_imm 0x03',
            '\tldi $a,0xAE',
            '\tjal __i2c_sb',
            # Send eeprom_hi (in C)
            '\tmov $c,$a',
            '\tjal __i2c_sb',
            # Send eeprom_lo (from stack)
            '\tpop $a',
            '\tjal __i2c_sb',
            # Repeated START + read address
            '\tddrb_imm 0x00',
            '\tddrb_imm 0x01',
            '\tddrb_imm 0x03',
            '\tldi $a,0xAF',
            '\tjal __i2c_sb',
            # Bulk read to code page
            f'\tldi $b,{OVERLAY_REGION_START}',
            # Count is 2 deep on stack now (lo was popped, size is next)
            '\tjal __eeprom_r2c_loop',
            '\tpop $a',             # clean size from stack
            '\tpop $a',             # clean extra lo from stack
            '\tret',
        ]

        # Wait — stack management is wrong. Let me re-trace:
        # After reading directory: stack has [size]
        # push lo: stack has [lo, size]
        # push lo: stack has [lo, lo, size]
        # pop for i2c_sb(lo): stack has [lo, size]
        # __eeprom_r2c_loop expects count at stack[SP+2]
        # After jal __eeprom_r2c_loop: stack has [ret_r2c, lo, size]
        # r2c_loop reads ldsp 2 = lo? NO! It should read size.
        # This is broken. Let me restructure.

        # The r2c_loop reads count from ldsp 2 (past its own return addr).
        # I need: stack = [r2c_ret, count, ...] when r2c_loop starts
        # Before jal __eeprom_r2c_loop: stack = [count, ...]
        # After jal: stack = [r2c_ret, count, ...]
        # ldsp 2 = count. Correct!
        # So I need size on top of stack just before jal __eeprom_r2c_loop.

        # Revised stack management:
        # After directory read: push size
        # push lo (for later i2c_sb)
        # ... I2C setup (pops lo for sending) ...
        # Before r2c_loop: stack = [size]
        # jal __eeprom_r2c_loop
        # After: pop size to clean

        # But C (eeprom_hi) gets clobbered by __i2c_sb. Need to save it.
        # Actually: send device addr, then hi, then lo. After sending device addr,
        # C still has hi. Send it. After sending hi, C is clobbered.
        # But we only need lo after that, which we saved on stack.

        # Let me rewrite the loader cleanly:
        loader = [
            '; ── EEPROM overlay loader ──',
            '__eeprom_load:',
            # Read directory from page 3: offset = id * 3 + base
            '\tmov $a,$d',          # D = id
            '\tsll',                # A = id * 2
            '\tadd $d,$a',          # A = id * 3
        ]
        if p3_dir_base > 0:
            loader.append(f'\taddi {p3_dir_base},$a')
        loader += [
            '\tmov $a,$d',          # D = directory base
            '\tderefp3',            # A = size
            '\tpush $a',            # stack: [size, ...]
            '\tmov $d,$a',
            '\tinc',
            '\tderefp3',            # A = eeprom_hi
            '\tpush $a',            # stack: [hi, size, ...]
            '\tmov $d,$a',
            '\tinc',
            '\tinc',
            '\tderefp3',            # A = eeprom_lo
            '\tpush $a',            # stack: [lo, hi, size, ...]
            # I2C START
            '\tddrb_imm 0x01',
            '\tddrb_imm 0x03',
            '\tldi $a,0xAE',
            '\tjal __i2c_sb',
            # Send hi (from stack)
            '\tldsp 2',             # A = hi (past: lo, ret_sb... wait)
        ]
        # Problem: jal __i2c_sb pushes its own return address.
        # After jal __i2c_sb returns: stack = [lo, hi, size, ...]
        # ldsp 2 from HERE: stack = [lo, hi, size], ldsp 2 = hi? No.
        # Actually after __i2c_sb returns (ret pops return addr), stack is unchanged
        # from before the jal. Stack = [lo, hi, size, ...]
        # So ldsp 1 = lo (at SP), ldsp 2... wait, ldsp N reads stack[SP+N].
        # SP points to the last pushed item. stack[SP] = lo (top).
        # ldsp 1 = stack[SP+1] = hi.  ldsp 2 = stack[SP+2] = size.
        # BUT ldsp N involves SP + N which can have the carry race.
        # For small N (1, 2, 3) it's fine unless SP is near 0xFF.

        # Actually wait. After the 3 pushes: SP = 0xFF - 3 - (return addrs from jal chain)
        # The __eeprom_load was called via jal from the dispatcher. That pushed a return addr.
        # So: stack = [lo, hi, size, load_ret, dispatch_ret, ...]
        # SP points at lo. ldsp 0 = lo, ldsp 1 = hi, ldsp 2 = size.

        # But after `jal __i2c_sb`, ret is popped. So stack is still [lo, hi, size, ...].
        # The ldsp at this point: SP unchanged. ldsp 0 = lo, ldsp 1 = hi. Good.

        # BUT: i2c_sb itself uses push/pop internally! After it returns, SP is
        # back where it was before the call. So the stack frame is preserved. OK.

        # Rewrite loader without the stack confusion:
        loader = [
            '; ── EEPROM overlay loader ──',
            '; A = overlay_id. Directory in page 3.',
            '__eeprom_load:',
            '\tmov $a,$d',          # D = id
            '\tsll',                # A = id*2
            '\tadd $d,$a',          # A = id*3
        ]
        if p3_dir_base > 0:
            loader.append(f'\taddi {p3_dir_base},$a')
        loader += [
            '\tmov $a,$d',          # D = dir offset
            '\tderefp3',            # A = size
            '\tpush $a',            # stack: [size]
            '\tmov $d,$a',
            '\tinc',
            '\tderefp3',            # A = hi
            '\tpush $a',            # stack: [hi, size]
            '\tmov $d,$a',
            '\tinc',
            '\tinc',
            '\tderefp3',            # A = lo
            '\tpush $a',            # stack: [lo, hi, size]
            # I2C: START + device write addr
            '\tddrb_imm 0x01',
            '\tddrb_imm 0x03',
            '\tldi $a,0xAE',
            '\tjal __i2c_sb',
            # Send hi
            '\tpop $a',             # A = lo (wrong! stack order)
        ]
        # Hmm, stack is LIFO. pop gives lo first, then hi, then size.
        # I need hi first (to send as EEPROM address high byte).
        # Fix: push in reverse order, or use data page.

        # Let me use data page slots instead of stack:
        loader = [
            '; ── EEPROM overlay loader ──',
            '__eeprom_load:',
            '\tmov $a,$d',          # D = id
            '\tsll',
            '\tadd $d,$a',          # A = id*3
        ]
        if p3_dir_base > 0:
            loader.append(f'\taddi {p3_dir_base},$a')
        loader += [
            '\tmov $a,$d',          # D = dir offset
            # Read size → data[8]
            '\tderefp3',            # A = size
            '\tldi $b,8',
            '\tideref',             # data[8] = size
            # Read hi
            '\tmov $d,$a',
            '\tinc',
            '\tderefp3',            # A = hi
            '\tldi $b,9',
            '\tideref',             # data[9] = hi
            # Read lo
            '\tmov $d,$a',
            '\tinc',
            '\tinc',
            '\tderefp3',            # A = lo
            '\tldi $b,10',
            '\tideref',             # data[10] = lo
            # I2C START + device write
            '\tddrb_imm 0x01',
            '\tddrb_imm 0x03',
            '\tldi $a,0xAE',
            '\tjal __i2c_sb',
            # Send hi
            '\tldi $a,9',
            '\tderef',              # A = data[9] = hi
            '\tjal __i2c_sb',
            # Send lo
            '\tldi $a,10',
            '\tderef',              # A = data[10] = lo
            '\tjal __i2c_sb',
            # Repeated START + read addr
            '\tddrb_imm 0x00',
            '\tddrb_imm 0x01',
            '\tddrb_imm 0x03',
            '\tldi $a,0xAF',
            '\tjal __i2c_sb',
            # Bulk read to code page
            f'\tldi $b,{OVERLAY_REGION_START} ;!ov_region',
            # Push count for __eeprom_r2c_loop
            '\tldi $a,8',
            '\tderef',              # A = data[8] = size
            '\tpush $a',
            '\tjal __eeprom_r2c_loop',
            '\tpop $a',             # clean count
            '\tret',
        ]

        # ── Step 12: Assemble everything ──
        # Ensure I2C helpers are emitted
        if not hasattr(self, '_lcd_helpers'):
            self._lcd_helpers = set()
        self._lcd_helpers.add('__i2c_sb')
        self._lcd_helpers.add('__i2c_rb')
        self._lcd_helpers.add('__eeprom_r2c_loop')

        # Check if _main already has i2c_init
        has_init = any('exw 0 0' in l for l in new_code)

        # Add cache initialization to _main (before user code)
        # Insert after VIA init: data[255] = 0xFF (no overlay cached)
        cache_init = [
            f'\tldi $a,0xFF',
            f'\tldi $b,{OV_CACHE_SLOT}',
            '\tideref',             # data[255] = 0xFF (no cache)
        ]
        # Find the end of VIA init in _main (after the push/pop warmup)
        insert_idx = 0
        for ci, cline in enumerate(new_code):
            if 'pop $a ;!keep' in cline:
                insert_idx = ci + 1
                break
            if cline.strip() == 'hlt' and ci < 3:
                insert_idx = ci
                break
        if insert_idx > 0:
            new_code = new_code[:insert_idx] + cache_init + new_code[insert_idx:]

        # Add dispatcher and loader
        new_code.extend(dispatcher)
        new_code.extend(loader)

        self.code = new_code

        # ── Step 12b: Two-pass fixup of OVERLAY_REGION_START ──
        # Now measure actual resident size and adjust if needed.
        actual_resident = measure_size(new_code)
        # Round up to next even address, plus 2 bytes padding
        new_start = actual_resident + (actual_resident % 2) + 2
        if new_start != OVERLAY_REGION_START:
            old_start = OVERLAY_REGION_START
            OVERLAY_REGION_START = new_start
            # Fix up ldi $b,{old_start} in the loader (tagged with ;!ov_region)
            old_ldi = f'\tldi $b,{old_start} ;!ov_region'
            new_ldi = f'\tldi $b,{OVERLAY_REGION_START} ;!ov_region'
            self.code = [l.replace(old_ldi, new_ldi) if old_ldi in l else l for l in self.code]
            # Fix up overlay entry addresses
            for ov in overlay_info:
                slot_offset = 0
                for fname in ov['functions']:
                    ov['entries'][fname] = OVERLAY_REGION_START + slot_offset
                    func_to_overlay[fname] = (ov['id'], ov['entries'][fname])
                    slot_offset += func_by_name[fname]['size']
            # Rewrite dispatch calls in code with updated entry addresses
            for ci, cline in enumerate(self.code):
                if cline.strip().startswith('ldi $b,') and ci > 0:
                    prev = self.code[ci - 1].strip()
                    if prev.startswith('ldi $a,') and ci + 1 < len(self.code):
                        nxt = self.code[ci + 1].strip()
                        if nxt == 'jal __eeprom_dispatch':
                            # Extract overlay_id from prev line
                            try:
                                ov_id = int(prev.split(',')[1])
                            except (IndexError, ValueError):
                                continue
                            # Find the matching overlay and get the entry for the old addr
                            for ov in overlay_info:
                                if ov['id'] == ov_id:
                                    # The old entry_addr encoded in this ldi $b
                                    # needs to be the updated one
                                    old_entry_str = cline.strip().split(',')[1]
                                    try:
                                        old_entry = int(old_entry_str)
                                    except ValueError:
                                        continue
                                    # Compute the function offset: old_entry - old_start
                                    func_offset = old_entry - old_start
                                    new_entry = OVERLAY_REGION_START + func_offset
                                    self.code[ci] = f'\tldi $b,{new_entry}'
                                    break

        OVERLAY_SIZE = 256 - OVERLAY_REGION_START
        if OVERLAY_SIZE < 16:
            print(f"ERROR: overlay region too small ({OVERLAY_SIZE}B). "
                  f"Resident code uses {actual_resident}B, leaving no room for overlays.",
                  file=sys.stderr)
            sys.exit(1)

        # Verify overlays still fit after resize
        for ov in overlay_info:
            if ov['size'] > OVERLAY_SIZE:
                print(f"ERROR: overlay {ov['id']} ({ov['functions']}, {ov['size']}B) "
                      f"exceeds final overlay region ({OVERLAY_SIZE}B).",
                      file=sys.stderr)
                sys.exit(1)

        if actual_resident >= 250:
            print(f"WARNING: resident code is {actual_resident}B, dangerously close to "
                  f"256B limit (HLT fill at 256).", file=sys.stderr)
        elif actual_resident >= 248:
            print(f"WARNING: resident code is {actual_resident}B, approaching 256B limit.",
                  file=sys.stderr)

        # ── Step 13: Emit overlay directory in page 3 ──
        self.code.append('\tsection page3')
        for ov in overlay_info:
            hi = (ov['eeprom_addr'] >> 8) & 0xFF
            lo = ov['eeprom_addr'] & 0xFF
            self.code.append(f'; overlay {ov["id"]}: {ov["functions"]} ({ov["size"]}B) @ EEPROM 0x{ov["eeprom_addr"]:04X}')
            self.code.append(f'\tbyte {ov["size"]}')
            self.code.append(f'\tbyte {hi}')
            self.code.append(f'\tbyte {lo}')
        self.page3_alloc += len(overlay_info) * DIR_ENTRY_SIZE

        # ── Step 14: Emit overlay code as page3_code sections ──
        # Each overlay's functions are emitted into page3_code at OVERLAY_REGION_START
        # The assembler stores them in the page3 buffer; the upload tool
        # extracts and writes to EEPROM.
        for ov in overlay_info:
            self.code.append(f'\tsection page3_code')
            for fname in ov['functions']:
                f = func_by_name[fname]
                self.code.extend(f['lines'])

        self.code.append('\tsection code')

        # ── Step 15: Output metadata for upload tool ──
        self._eeprom_overlays = overlay_info
        # Print summary
        n_overlays = len(overlay_info)
        total_ov_bytes = sum(ov['size'] for ov in overlay_info)
        print(f"EEPROM overlay mode: {n_overlays} overlays, {total_ov_bytes}B in EEPROM, "
              f"overlay region 0x{OVERLAY_REGION_START:02X}-0xFF ({OVERLAY_SIZE}B)",
              file=sys.stderr)
        for ov in overlay_info:
            print(f"  OV{ov['id']}: {ov['functions']} ({ov['size']}B) @ 0x{ov['eeprom_addr']:04X}",
                  file=sys.stderr)

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
                self.emit('\tpop $b')       # B = right_lo
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
                self.emit('\tpop $b')
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
                # then set ORB=0, DDRB=0, DDRA=0.
                self.emit('\tldi $d,0')
                lbl = self.label('via_dly')
                self.emit(f'{lbl}:')
                self.emit('\tdec')
                self.emit(f'\tjnz {lbl}')
                self.emit('\tclr $a')
                self.emit('\texw 0 0')         # ORB = 0
                self.emit('\tddrb_imm 0x00')   # DDRB = 0 (both lines idle/HIGH)
                self.emit('\tclr $a')
                self.emit('\texw 0 3')         # DDRA = 0
                self.emit('\tpush $a ;!keep')  # stack warmup (STK pin settling)
                self.emit('\tpop $a ;!keep')
                return

            if name == 'i2c_bus_reset':
                # Full bus reset: VIA delay + init + STOP to clear stuck I2C state.
                self.emit('\tldi $d,0')
                lbl = self.label('br_dly')
                self.emit(f'{lbl}:')
                self.emit('\tdec')
                self.emit(f'\tjnz {lbl}')
                self.emit('\tclr $a')
                self.emit('\texw 0 0')         # ORB = 0
                self.emit('\tddrb_imm 0x00')   # DDRB = 0 (idle)
                self.emit('\tclr $a')
                self.emit('\texw 0 3')         # DDRA = 0
                self.emit('\tpush $a ;!keep')  # stack warmup
                self.emit('\tpop $a ;!keep')
                self.emit('\tddrb_imm 0x03')   # STOP: both LOW
                self.emit('\tddrb_imm 0x01')   # STOP: SDA LOW, SCL HIGH
                self.emit('\tddrb_imm 0x00')   # STOP: both HIGH (idle)
                return

            if name == 'lcd_init':
                # Complete LCD init sequence as one jal call
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__lcd_init')
                self.emit('\tjal __lcd_init')
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
                # Stores [ratio, cyc_lo, cyc_hi] in page 3 note table.
                # Emits: ldi $a, offset; jal __play_note (4 bytes per call).
                if len(args) < 2:
                    raise Exception("tone() requires 2 arguments: freq_hz, duration_ms")
                freq = self._const_eval(args[0])
                dur = self._const_eval(args[1])
                if freq is None or dur is None:
                    raise Exception("tone() requires constant arguments")
                ratio = round(8000 / freq)
                if ratio > 255 or ratio < 1:
                    raise Exception(f"tone frequency {freq}Hz out of range (~32Hz-8kHz)")
                total_cycles = freq * dur // 1000
                if total_cycles < 1:
                    return  # too short to play
                cyc_hi = min((total_cycles >> 8) & 0xFF, 255)
                cyc_lo = total_cycles & 0xFF
                # Allocate 3 bytes in page 3 note table
                p3_off = self.page3_alloc
                if not hasattr(self, '_note_table'):
                    self._note_table = []
                self._note_table.append((p3_off, ratio, cyc_lo, cyc_hi))
                self.page3_alloc += 3
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__play_note')
                self._lcd_helpers.add('__tone_setup')
                self._lcd_helpers.add('__tone')
                self.emit(f'\tldi $a,{p3_off}')
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
                p3_off = self.page3_alloc
                if not hasattr(self, '_note_table'):
                    self._note_table = []
                self._note_table.append((p3_off, 0, dur_ms & 0xFF, 0))
                self.page3_alloc += 3
                if not hasattr(self, '_lcd_helpers'):
                    self._lcd_helpers = set()
                self._lcd_helpers.add('__play_note')
                self._lcd_helpers.add('__delay_Nms')
                self.emit(f'\tldi $a,{p3_off}')
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
            if base_expr[0] == 'var' and base_expr[1] in self.page3_globals:
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
                  'adc', 'sbc', 'exr', 'exrw'}

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
            s = line.strip()
            if not s.startswith('.'):
                # Global label (function boundary): all unknown but distinct
                invalidate_all()
            else:
                # Local label: A unknown, keep B/C/D
                regs['a'] = fresh_unknown()
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

    # Always run peephole optimizer
    lines = asm.split('\n')
    lines = peephole(lines)
    asm = '\n'.join(lines)

    if args.output:
        with open(args.output, 'w') as f: f.write(asm + '\n')
    else:
        print(asm)

if __name__ == '__main__':
    main()
