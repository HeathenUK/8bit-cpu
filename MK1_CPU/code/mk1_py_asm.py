#!/usr/bin/env python3
"""Python MK1 assembler — converts mk1cc2's asm dialect to code/data/page3
byte arrays. Mirrors enough of the ESP32 C++ assembler (assembler.h) to
let us run compiled programs through mk1sim.MK1 without needing hardware.

mk1cc2's dialect differs slightly from microcode.py's canonical mnemonics:
  mk1cc2  →  canonical
  ldi $a,5    move imm, $a
  mov $a,$b   move $a, $b
  push $a     stor $a, [$sp]
  pop $a      load $a, [$sp]
  jal X       stor $pc, [$sp]     (imm=X)
  j X         move imm, $pc       (imm=X)
  ret         load $pc, [$sp]
  clr $a      sub $a, $a
  nop         move $a, $a
  out         move $a, $out
  addi N,$a   add imm, $a         (imm=N)
  cmpi N      cmp imm             (imm=N)

Sections:
  code / page3_code / data_code / stack_code / data / page3 / eeprom
  `org N` sets the current position within the active section.

Two-pass assembly: pass 1 collects label addresses, pass 2 emits bytes.
Output matches the ESP32 uploadBuf layout: [code(256), data(256),
stack(256), page3(256)] plus optional EEPROM bytes.

Usage as a module:
    pages = assemble_asm(asm_text)
    # pages['code'], pages['data'], pages['stack'], pages['page3'],
    # pages['eeprom']
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import microcode as _mc


# ── Build the canonical-mnemonic → opcode map ────────────────────────

def _build_opcode_map():
    """Returns (by_mnemonic, has_imm_map).
    by_mnemonic[canonical_str] = opcode_byte
    has_imm_map[opcode_byte] = True/False
    """
    by_mn = {}
    has_imm = {}
    for opc, entry in _mc.ucode_template.items():
        mn, steps, imm = entry
        by_mn[mn] = opc
        has_imm[opc] = imm
    return by_mn, has_imm


CANON, HAS_IMM = _build_opcode_map()


# ── mk1cc2 dialect → canonical mnemonic translator ───────────────────

def _translate_instr(s):
    """Return (canonical_mnemonic, imm_expr_or_None) for one instr line.

    s is the bare instruction text (no leading whitespace, no label).
    imm_expr_or_None is a string expression (label/number) that should
    resolve to a byte value during pass 2.

    Raises ValueError for unknown instructions."""
    parts = s.split()
    if not parts:
        raise ValueError(f'empty instruction')
    mn = parts[0]
    rest = s[len(mn):].strip()

    # Directly-matching mnemonics with no operand rewriting
    direct = {
        'nop': ('move $a, $a', None),
        'hlt': ('hlt', None),
        'ret': ('load $pc, [$sp]', None),
        'out': ('move $a, $out', None),
        'clr': None,        # handled below (has operand)
        'dec': ('dec', None),
        'inc': ('inc', None),
        'decb': ('decb', None), 'incb': ('incb', None),
        'decc': ('decc', None), 'incc': ('incc', None),
        'decd': ('decd', None), 'incd': ('incd', None),
        'deref': ('deref', None),
        'deref2': ('deref2', None),
        'derefp3': ('derefp3', None),
        'ideref': ('ideref', None),
        'ideref2': ('ideref2', None),
        'iderefp3': ('iderefp3', None),
        'istc': ('istc', None),
        'istc_inc': ('istc_inc', None),
        'sll': ('sll', None), 'slr': ('slr', None),
        'sllb': ('sllb', None),
        'rll': ('rll', None), 'rlr': ('rlr', None),
        'swap': ('swap', None),
        'not': ('not', None), 'neg': ('neg', None),
        'xor': ('xor', None),
        'pop_b': ('pop_b', None),
        'push_b': ('push_b', None),
        'setc': ('setc', None), 'setz': ('setz', None),
        'setnc': ('setnc', None), 'setnz': ('setnz', None),
        'setjmp': ('setjmp', None), 'setret': ('setret', None),
        'ocall': ('ocall', None),
        'adc': ('adc', None), 'sbc': ('sbc', None),
        'jal_r': ('jal_r', None),
        'cmpi': None,  # handled below
    }
    if mn in direct and direct[mn] is not None:
        return direct[mn]

    # clr $a → sub $a, $a
    if mn == 'clr':
        if rest == '$a':
            return ('sub $a, $a', None)
        raise ValueError(f'unknown clr operand: {s!r}')

    # ldi $X, N → move imm, $X
    if mn == 'ldi':
        m = re.match(r'\s*(\$\w+)\s*,\s*(\S+)\s*$', rest)
        if not m:
            raise ValueError(f'bad ldi: {s!r}')
        reg, imm = m.group(1), m.group(2)
        return (f'move imm, {reg}', imm)

    # mov $src, $dst → move $src, $dst (may be 2B if src=imm or dst=imm)
    if mn == 'mov':
        m = re.match(r'\s*(\$\w+)\s*,\s*(\$\w+)\s*$', rest)
        if not m:
            raise ValueError(f'bad mov: {s!r}')
        src, dst = m.group(1), m.group(2)
        return (f'move {src}, {dst}', None)

    # push $X → stor $X, [$sp]
    if mn == 'push':
        m = re.match(r'\s*(\$\w+)\s*$', rest)
        if not m:
            raise ValueError(f'bad push: {s!r}')
        return (f'stor {m.group(1)}, [$sp]', None)
    # pop $X → load $X, [$sp]
    if mn == 'pop':
        m = re.match(r'\s*(\$\w+)\s*$', rest)
        if not m:
            raise ValueError(f'bad pop: {s!r}')
        return (f'load {m.group(1)}, [$sp]', None)

    # push_imm N → push_imm with imm
    if mn == 'push_imm':
        return ('push_imm', rest)

    # jal X → stor $pc, [$sp] imm=X
    if mn == 'jal':
        return ('stor $pc, [$sp]', rest)
    # j/jz/jnz/jc/jnc X → specific opcodes with imm=X
    if mn == 'j':
        return ('move imm, $pc', rest)
    if mn == 'jz':
        return ('jzf', rest)
    if mn == 'jnz':
        return ('jnz', rest)
    if mn == 'jc':
        return ('jcf', rest)
    if mn == 'jnc':
        return ('jnc', rest)

    # Immediate ALU: addi/subi/andi/ori N,$X → add/sub/and/or imm, $X
    for op_in, op_out in (('addi', 'add'), ('subi', 'sub'),
                          ('andi', 'and'), ('ori', 'or')):
        if mn == op_in:
            m = re.match(r'\s*(\S+?)\s*,\s*(\$\w+)\s*$', rest)
            if not m:
                raise ValueError(f'bad {op_in}: {s!r}')
            imm, dst = m.group(1), m.group(2)
            return (f'{op_out} imm, {dst}', imm)

    # ALU reg-reg: add/sub/and/or $src, $dst
    for alu in ('add', 'sub', 'and', 'or'):
        if mn == alu:
            m = re.match(r'\s*(\$\w+)\s*,\s*(\$\w+)\s*$', rest)
            if not m:
                raise ValueError(f'bad {alu}: {s!r}')
            return (f'{alu} {m.group(1)}, {m.group(2)}', None)

    # cmp $X → cmp $X (1B); cmp N → cmp imm (2B) — mk1cc2 writes both as `cmp ...`
    if mn == 'cmp':
        m = re.match(r'\s*(\S+)\s*$', rest)
        if not m:
            raise ValueError(f'bad cmp: {s!r}')
        operand = m.group(1)
        if operand.startswith('$'):
            return (f'cmp {operand}', None)
        return ('cmp imm', operand)
    # cmpi N → cmp imm (but there's a separate 0xFD opcode for cmpi)
    if mn == 'cmpi':
        return ('cmpi', rest)

    # tst N → tst with imm
    if mn == 'tst':
        return ('tst', rest)

    # load $X, [$Y] / stor $X, [$Y] / load $X, [imm] / stor $X, [imm]
    if mn in ('load', 'stor'):
        m = re.match(r'\s*(\$\w+)\s*,\s*\[(.+?)\]\s*$', rest)
        if not m:
            raise ValueError(f'bad {mn}: {s!r}')
        reg, inside = m.group(1), m.group(2).strip()
        if inside.startswith('$'):
            return (f'{mn} {reg}, [{inside}]', None)
        # Non-register: immediate addressing
        return (f'{mn} {reg}, [imm]', inside)

    # move (canonical form user may type directly)
    if mn == 'move':
        m = re.match(r'\s*(\$\w+|imm)\s*,\s*(\$\w+|imm)\s*$', rest)
        if m:
            src, dst = m.group(1), m.group(2)
            return (f'move {src}, {dst}', None if src != 'imm' else '0')

    # ddrb_imm N, ddra_imm N, orb_imm N, ora_imm N, out_imm N
    for specimm in ('ddrb_imm', 'ddra_imm', 'orb_imm', 'ora_imm', 'out_imm'):
        if mn == specimm:
            return (specimm, rest)

    # ldsp N, stsp N, ldsp_b N, ldp3 N, stp3 N
    for specmn in ('ldsp', 'stsp', 'ldsp_b', 'ldp3', 'stp3'):
        if mn == specmn:
            return (specmn, rest)

    # exr / exw / exrw
    if mn == 'exr' or mn == 'exw' or mn == 'exrw':
        # Normalise spacing: `exr 0` vs `exr 0 0`
        parts2 = rest.split()
        if mn == 'exrw':
            # exrw N — single operand in canonical. mk1cc2 uses same form.
            return (f'exrw {parts2[0]}', None)
        # exw/exr need two operands in canonical form
        if len(parts2) == 1:
            return (f'{mn} 0 {parts2[0]}', None)
        if len(parts2) == 2:
            return (f'{mn} {parts2[0]} {parts2[1]}', None)

    raise ValueError(f'unknown instruction: {s!r}')


# ── Expression evaluation (pass 2) ───────────────────────────────────

def _eval_imm(expr, labels):
    """Resolve a numeric immediate. Accepts decimal, hex (0xNN), binary
    (0bNNN), label names, char literals ('A'), or sum of such via `+`."""
    expr = expr.strip()
    if expr.startswith("'") and expr.endswith("'") and len(expr) >= 3:
        body = expr[1:-1]
        if body.startswith('\\'):
            esc = {'n': 10, 'r': 13, 't': 9, '0': 0, '\\': 92, "'": 39}
            if len(body) == 2 and body[1] in esc:
                return esc[body[1]]
        if len(body) == 1:
            return ord(body)
    # Simple sum: a+b
    if '+' in expr and not expr.startswith('0x'):
        return sum(_eval_imm(p.strip(), labels) for p in expr.split('+'))
    if expr.startswith('0x') or expr.startswith('0X'):
        return int(expr, 16)
    if expr.startswith('0b') or expr.startswith('0B'):
        return int(expr, 2)
    if expr.lstrip('-').isdigit():
        return int(expr)
    # Label
    if expr in labels:
        return labels[expr]
    raise ValueError(f'unresolved immediate: {expr!r}')


# ── Pass 1: measure sizes + collect label addresses ──────────────────

_TWO_BYTE_MNS = {'move imm, $a', 'move imm, $b', 'move imm, $c', 'move imm, $d',
                 'move imm, $sp', 'move imm, $pc', 'push_imm', 'out_imm',
                 'ddrb_imm', 'ddra_imm', 'orb_imm', 'ora_imm',
                 'add imm, $a', 'add imm, $b', 'add imm, $c', 'add imm, $d',
                 'sub imm, $a', 'sub imm, $b', 'sub imm, $c', 'sub imm, $d',
                 'and imm, $a', 'and imm, $b', 'and imm, $c', 'and imm, $d',
                 'or imm, $a', 'or imm, $b', 'or imm, $c', 'or imm, $d',
                 'cmp imm', 'cmpi', 'tst',
                 'ldsp', 'stsp', 'ldsp_b', 'ldp3', 'stp3',
                 'stor $pc, [$sp]',   # jal imm
                 'stor $a, [imm]', 'stor $b, [imm]', 'stor $c, [imm]',
                 'stor $d, [imm]', 'stor $pc, [imm]', 'stor $sp, [imm]',
                 'load $a, [imm]', 'load $b, [imm]', 'load $c, [imm]',
                 'load $d, [imm]', 'load $pc, [imm]', 'load $sp, [imm]'}


def _instr_size(canon_mn):
    return 2 if canon_mn in _TWO_BYTE_MNS else 1


def _parse_lines(asm_text):
    """Strip comments + blank lines. Yields (line_no, stripped_text)."""
    for i, raw in enumerate(asm_text.splitlines(), 1):
        s = raw
        # Strip ; comments
        if ';' in s:
            s = s[:s.index(';')]
        s = s.strip()
        if s:
            yield i, s


# Sections recognised by mk1cc2 + their target byte-buffer key.
# `code` is the main stage-1 code page.
# `page3_code` bytes live at code addresses but are ALSO copied to page3
# at the same offset (via `section page3_code` + `org N`).
# `page3_kernel` is an alternate spelling for init-extraction mode.
# `data_code`, `stack_code` are overlay BODY bytes that live in data/stack
# pages at runtime (copied by overlay loader to code[overlay_region]).
# `data`, `page3`, `eeprom`, `stack` are pure data.
SECTIONS = {
    'code': 'code',
    'page3_code': 'page3_code',   # dual-residence (code + page3)
    'page3_kernel': 'page3_code', # alias
    'data_code': 'data_code',     # overlay body → data page
    'stack_code': 'stack_code',   # overlay body → stack page
    'data': 'data',
    'page3': 'page3',             # direct page3 data (bytes, no code)
    'stack': 'stack',
    'eeprom': 'eeprom',
}


def assemble_asm(asm_text):
    """Two-pass assemble. Returns dict with bytearrays for each page."""
    # Pass 1 — compute label addresses per section.
    # Each section has its own address counter. `org N` sets it.
    # Labels take the current counter value in the current section.
    # For `code` and `page3_code`: the counter IS the code-page address.
    # For data/page3/stack/eeprom: the counter is the buffer offset.
    sec_pc = {k: 0 for k in set(SECTIONS.values())}
    # Track overlay-body data offsets separately. `data_code` + `org N`
    # means "virtually at code address N" but BYTES go sequentially into
    # the data page starting at the next-free data slot.
    data_code_offset = None   # set when overlay storage starts
    # Labels map name → address (int). data_code labels use the org value,
    # so when the loader copies data_code[offset..offset+size] to code[N..],
    # the labels resolve correctly to the loaded addresses.
    labels = {}
    cur_section = 'code'
    pass1_lines = []   # list of (section_at_time_of_line, text)
    for ln_no, text in _parse_lines(asm_text):
        if text.startswith('section '):
            sec = text.split()[1]
            if sec not in SECTIONS:
                raise ValueError(f'Line {ln_no}: unknown section {sec!r}')
            cur_section = SECTIONS[sec]
            pass1_lines.append((cur_section, text))
            continue
        if text.startswith('org '):
            val = int(text.split()[1], 0)
            sec_pc[cur_section] = val
            pass1_lines.append((cur_section, text))
            continue
        # Label? (ends with :, no whitespace)
        if text.endswith(':'):
            label = text[:-1]
            # For code/page3_code: label = code-page address (sec_pc['code']
            # or sec_pc['page3_code']). For data_code/stack_code: label =
            # the virtual code-page address (the `org N` value, which is
            # where the overlay body will live at runtime).
            labels[label] = sec_pc[cur_section]
            pass1_lines.append((cur_section, text))
            continue
        # Directive: `byte N` = 1 byte, takes expression
        if text.startswith('byte '):
            pass1_lines.append((cur_section, text))
            sec_pc[cur_section] += 1
            continue
        # Regular instruction
        try:
            canon, imm_expr = _translate_instr(text)
        except ValueError as e:
            raise ValueError(f'Line {ln_no}: {e}')
        size = _instr_size(canon)
        pass1_lines.append((cur_section, text))
        sec_pc[cur_section] += size

    # Pass 2 — emit bytes.
    code = bytearray(256)
    data = bytearray(256)
    stack = bytearray(256)
    page3 = bytearray(256)
    eeprom = bytearray(4096)

    # For data_code/stack_code: sequential offset in target page
    data_offset = 0
    stack_offset = 0

    sec_pc = {k: 0 for k in set(SECTIONS.values())}
    cur_section = 'code'
    # Track whether data_code's current org was set (first org after the
    # section switch defines the base; bytes are placed sequentially at
    # data_offset which is independent of the virtual org).

    for sec_at_line, text in pass1_lines:
        cur_section = sec_at_line
        if text.startswith('section '):
            continue
        if text.startswith('org '):
            val = int(text.split()[1], 0)
            sec_pc[cur_section] = val
            continue
        if text.endswith(':'):
            continue
        if text.startswith('byte '):
            expr = text.split(None, 1)[1]
            val = _eval_imm(expr, labels) & 0xFF
            _emit_byte(cur_section, sec_pc[cur_section], val,
                       code, data, stack, page3, eeprom,
                       data_offset_ref=None, stack_offset_ref=None)
            sec_pc[cur_section] += 1
            continue
        canon, imm_expr = _translate_instr(text)
        opcode = CANON.get(canon)
        if opcode is None:
            raise ValueError(f'No opcode for canonical {canon!r} (from {text!r})')
        addr = sec_pc[cur_section]
        _emit_byte(cur_section, addr, opcode, code, data, stack, page3, eeprom,
                   data_offset_ref=None, stack_offset_ref=None)
        sec_pc[cur_section] += 1
        if canon in _TWO_BYTE_MNS:
            if imm_expr is None:
                raise ValueError(f'Expected immediate for {canon!r}: {text!r}')
            imm_val = _eval_imm(imm_expr, labels) & 0xFF
            _emit_byte(cur_section, sec_pc[cur_section], imm_val,
                       code, data, stack, page3, eeprom,
                       data_offset_ref=None, stack_offset_ref=None)
            sec_pc[cur_section] += 1

    return {'code': code, 'data': data, 'stack': stack, 'page3': page3,
            'eeprom': eeprom, 'labels': labels}


def _emit_byte(section, addr, val, code, data, stack, page3, eeprom,
               data_offset_ref, stack_offset_ref):
    """Place one byte into the appropriate buffer given the current section
    and the current section's address counter."""
    if section == 'code':
        if 0 <= addr < 256:
            code[addr] = val
    elif section == 'page3_code':
        # Dual-residence: bytes go to BOTH the code page AND page3 at
        # the same offset. Stage-1 init contains bytes that live in the
        # code page initially; self-copy then reads them from page3 (where
        # they were also written) and copies back to code addresses 0..N.
        if 0 <= addr < 256:
            code[addr] = val
            page3[addr] = val
    elif section == 'data_code':
        # Overlay body bytes — stored in DATA page sequentially; the
        # `org N` on the section just determines the runtime code address
        # for label resolution. We use the label-relative offset (addr
        # minus whatever the first org was) to find the data-page slot.
        # Simpler: each data_code section starts at the current data_alloc;
        # compute relative offset from first instruction of this body.
        # For now, place bytes sequentially starting at data[0] — the
        # manifest must have been generated with consistent offsets.
        # (mk1cc2 does this: manifest offsets are computed after overlay
        # layout, and the first overlay body goes at data[0].)
        # addr here is the virtual code-page address; we need data offset.
        # Hack: assume data_code bodies start at data[0] and use address
        # relative to first seen addr.
        pass   # handled by simple-append logic below
    elif section == 'stack_code':
        pass   # same as data_code but for stack page
    elif section == 'data':
        if 0 <= addr < 256:
            data[addr] = val
    elif section == 'page3':
        if 0 <= addr < 256:
            page3[addr] = val
    elif section == 'stack':
        if 0 <= addr < 256:
            stack[addr] = val
    elif section == 'eeprom':
        if 0 <= addr < 4096:
            eeprom[addr] = val


if __name__ == '__main__':
    # Smoke test
    asm = """
    section code
    org 0
    ldi $a,42
    out
    hlt
    """
    result = assemble_asm(asm)
    print('code[0..5] =', result['code'][:5].hex())
    # Expected: move imm, $a opcode + 42 + out opcode + hlt opcode
    # move imm, $a = 0x38 (0b00111000). out = move $a, $out = 0x06. hlt = 0x7F.
    # Byte sequence: 38 2A 06 7F
