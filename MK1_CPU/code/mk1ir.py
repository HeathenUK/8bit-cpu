"""T1.1 — whole-program IR.

Bridge between the existing flat-asm representation that every pass in
mk1cc2 consumes today and a structured (section → labels → instrs)
model that cross-section passes can operate on directly.

Design principles:
  - ROUND-TRIP FIDELITY: parse(serialize(ir)) == ir for every legal
    asm produced by mk1cc2. Verify by diffing before/after every
    corpus program; any behavior change is a parser bug.
  - NO SEMANTICS CHANGE YET: this module is a structured VIEW of the
    same bytes. Passes can be migrated to operate on IR one at a
    time; each migration is verifiable by byte-equivalent output.
  - MINIMAL: no attempt to decompose operands to a typed form; the
    operand string is stored verbatim. Future passes can parse
    further when they need to.

Structure:
  Program
    ├── sections: dict[str, Section]   # keyed by section name
    └── labels: dict[str, (section, index)]  # global + scoped labels

  Section
    ├── name: str                      # 'code', 'page3_code', 'data', …
    ├── origin: int | None             # `org N` directive (None = 0)
    └── items: list[Item]              # instructions + labels + directives + comments

  Item is one of:
    Label(name: str, is_local: bool)   # '.foo:' vs 'global:'
    Instr(mnemonic: str, operands: list[str], size_bytes: int, comment: str)
    Directive(name: str, args: str)    # 'byte N', 'org N', 'section X'
    Comment(text: str)                 # '; …' standalone comments
    Blank()                            # preserve blank lines for readability

Round-trip policy: serialization preserves source text EXACTLY
(including indentation, semicolons, comment spacing) wherever the
original line is stored in an Item's `raw` field. Semantic edits
go through `replace` helpers that regenerate raw from structured
fields.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


# ── Item hierarchy ─────────────────────────────────────────────────

@dataclass
class Item:
    raw: str                # original source line (including leading tab/spaces)
    # Subclasses set these where relevant — keep here for simple serialization.


@dataclass
class Label(Item):
    name: str = ''
    is_local: bool = False  # True for '.foo:', False for 'foo:'


@dataclass
class Instr(Item):
    mnemonic: str = ''
    operands: list[str] = field(default_factory=list)
    size_bytes: int = 0
    comment: str = ''       # inline '; …' after operands, stripped of leading ';'


@dataclass
class Directive(Item):
    name: str = ''          # 'section', 'org', 'byte', …
    args: str = ''          # everything after the directive name


@dataclass
class Comment(Item):
    text: str = ''          # comment body, stripped of leading ';'


@dataclass
class Blank(Item):
    pass


# ── Section + Program ──────────────────────────────────────────────

@dataclass
class Section:
    name: str
    origin: Optional[int] = None
    items: list[Item] = field(default_factory=list)


@dataclass
class Program:
    """Asm files re-enter sections multiple times (`section code` …
    `section page3_code` … `section code` again, etc.). We model this
    as an ORDERED LIST of section-chunks rather than a dict, so
    serialize() can reproduce the original interleaving byte-for-byte.

    Passes that need all items in a logical section can use
    `items_in(section_name)` which concatenates chunks by name."""
    chunks: list[Section] = field(default_factory=list)
    # Cross-section label index: label_name → (chunk_index, item_index).
    # Last-wins on duplicates (same as the assembler).
    labels: dict[str, tuple[int, int]] = field(default_factory=dict)

    def items_in(self, name: str) -> list[Item]:
        """Concatenated items from every chunk whose section name matches."""
        out = []
        for ch in self.chunks:
            if ch.name == name:
                out.extend(ch.items)
        return out

    def section_names(self) -> list[str]:
        """Unique section names in first-appearance order."""
        seen = set()
        out = []
        for ch in self.chunks:
            if ch.name not in seen:
                seen.add(ch.name)
                out.append(ch.name)
        return out


# ── Byte-size table (mirrors mk1cc2's `two_byte` set) ──────────────

# Instructions that encode to 2 bytes (1-byte opcode + 1-byte operand).
# Everything else is 1 byte. Labels, directives, comments, and blanks
# count as 0 bytes.
_TWO_BYTE_MNEMONICS = {
    'ldi', 'addi', 'subi', 'ori', 'andi',
    'j', 'jal', 'jz', 'jnz', 'jc', 'jnc',
    'push_imm', 'ldsp', 'stsp', 'ldsp_b',
    'ldp3', 'stp3',
    'ddrb_imm', 'ddra_imm', 'orb_imm', 'ora_imm',
    'ocall', 'setjmp',
    'tst', 'cmpi', 'out_imm',
    'je0', 'je1',
}


def instr_size(mnemonic: str, operand_str: str) -> int:
    """Return byte size of a single instruction. Mirrors measure_lines()
    in mk1cc2. `cmp` is special-cased: `cmp $X` is 1 byte (register
    cmp opcode), `cmp N` is 2 bytes (cmpi)."""
    if mnemonic == 'cmp':
        return 1 if operand_str.strip().startswith('$') else 2
    if mnemonic in _TWO_BYTE_MNEMONICS:
        return 2
    return 1


# ── Parser ─────────────────────────────────────────────────────────

def _strip_comment(line: str) -> tuple[str, str]:
    """Split a line into (instruction-part, comment-part).
    Comment starts at the first unquoted ';'."""
    idx = line.find(';')
    if idx < 0:
        return line, ''
    return line[:idx].rstrip(), line[idx + 1:]


def parse_line(raw: str) -> Item:
    """Classify one source line into an Item subclass. The raw string
    is preserved verbatim on the returned Item for round-trip."""
    s = raw.strip()
    if not s:
        return Blank(raw=raw)
    if s.startswith(';'):
        return Comment(raw=raw, text=s[1:])

    body, comment_text = _strip_comment(raw)
    body_s = body.strip()
    if not body_s:
        # Line is just a comment after whitespace
        return Comment(raw=raw, text=comment_text)

    # Label?
    if body_s.endswith(':'):
        name = body_s[:-1]
        return Label(raw=raw, name=name, is_local=name.startswith('.'))

    # Directive (section, org, byte)
    toks = body_s.split(None, 1)
    mn = toks[0]
    args = toks[1] if len(toks) > 1 else ''
    if mn in ('section', 'org', 'byte'):
        return Directive(raw=raw, name=mn, args=args)

    # Instruction. Operands split by comma at the top level; whitespace
    # trimmed. For simplicity we store operand tokens verbatim.
    ops = []
    if args:
        # Many MK1 instructions use comma separation, but some (exw X Y,
        # exrw N) use whitespace. Preserve whatever was there by just
        # storing the full operand string as a single element plus
        # keeping raw for serialization. Passes that need to inspect
        # operands can re-split.
        ops = [args]
    return Instr(
        raw=raw,
        mnemonic=mn,
        operands=ops,
        size_bytes=instr_size(mn, args),
        comment=comment_text,
    )


def parse_program(lines: list[str]) -> Program:
    """Turn a list of asm lines into a Program. Each `section X`
    directive opens a new chunk — we do NOT merge chunks by name so
    the interleaving found in mk1cc2 output round-trips exactly."""
    prog = Program()
    current_chunk: Optional[Section] = None

    for raw in lines:
        item = parse_line(raw)

        if isinstance(item, Directive) and item.name == 'section':
            sec_name = item.args.strip()
            current_chunk = Section(name=sec_name)
            prog.chunks.append(current_chunk)
            current_chunk.items.append(item)
            continue

        if isinstance(item, Directive) and item.name == 'org':
            if current_chunk is None:
                current_chunk = Section(name='code')
                prog.chunks.append(current_chunk)
            try:
                current_chunk.origin = int(item.args.strip(), 0)
            except (ValueError, AttributeError):
                pass
            current_chunk.items.append(item)
            continue

        # All other items go into the current chunk. If we haven't seen
        # a section directive yet, synthesize a default 'code' chunk so
        # leading comments/blanks aren't lost.
        if current_chunk is None:
            current_chunk = Section(name='code')
            prog.chunks.append(current_chunk)

        if isinstance(item, Label) and not item.is_local:
            chunk_idx = len(prog.chunks) - 1
            prog.labels[item.name] = (chunk_idx, len(current_chunk.items))
        current_chunk.items.append(item)

    return prog


# ── Serializer ────────────────────────────────────────────────────

def serialize_program(prog: Program) -> list[str]:
    """Emit the program as a list of asm lines by concatenating chunks
    in insertion order. Items mutated by passes update their `raw`
    field (see `replace_instr`), so the output reflects edits."""
    out = []
    for chunk in prog.chunks:
        for item in chunk.items:
            out.append(item.raw.rstrip('\n'))
    return out


# ── Mutation helpers ──────────────────────────────────────────────
#
# Passes that rewrite instructions should use these — they keep raw
# and the structured fields in sync so serialization reflects edits.

def replace_instr(section: Section, idx: int, new_mnemonic: str,
                  new_operands: Optional[list[str]] = None,
                  comment: Optional[str] = None) -> None:
    """Replace the instruction at `section.items[idx]` with a new
    mnemonic + operands. Preserves indentation from the original raw
    line."""
    old = section.items[idx]
    assert isinstance(old, Instr), f'expected Instr at {idx}, got {type(old).__name__}'
    indent = old.raw[:len(old.raw) - len(old.raw.lstrip())]
    ops = new_operands if new_operands is not None else old.operands
    new_comment = comment if comment is not None else old.comment
    op_str = ','.join(ops) if ops else ''
    parts = [new_mnemonic]
    if op_str:
        parts.append(op_str)
    line = indent + ' '.join(parts)
    if new_comment:
        line += ' ;' + new_comment
    section.items[idx] = Instr(
        raw=line,
        mnemonic=new_mnemonic,
        operands=ops,
        size_bytes=instr_size(new_mnemonic, op_str),
        comment=new_comment,
    )


def delete_items(section: Section, start: int, end: int) -> None:
    """Delete items[start:end] from a section. Cross-section labels
    pointing into this range become stale — callers are expected to
    rebuild the Program's label index if they rely on it."""
    del section.items[start:end]


def section_size_bytes(section: Section) -> int:
    """Total instruction bytes in a section. Labels, directives, and
    comments don't contribute. `byte N` directives contribute 1 byte
    per line."""
    total = 0
    for item in section.items:
        if isinstance(item, Instr):
            total += item.size_bytes
        elif isinstance(item, Directive) and item.name == 'byte':
            total += 1
    return total


# ── Round-trip sanity check ───────────────────────────────────────

def round_trip_identical(lines: list[str]) -> tuple[bool, str]:
    """Parse then serialize the given lines. Returns (identical, diff)
    where identical is True iff the serialization exactly matches the
    input (modulo trailing whitespace on each line). Used as a
    regression guard: if this returns False for any corpus program's
    asm, the parser is lossy for some construct we haven't covered."""
    prog = parse_program(lines)
    out = serialize_program(prog)
    in_trimmed = [ln.rstrip('\n').rstrip() for ln in lines]
    out_trimmed = [ln.rstrip('\n').rstrip() for ln in out]
    if in_trimmed == out_trimmed:
        return True, ''
    # Minimal diff: show first mismatch
    for i, (a, b) in enumerate(zip(in_trimmed, out_trimmed)):
        if a != b:
            return False, f'line {i}: {a!r} != {b!r}'
    if len(in_trimmed) != len(out_trimmed):
        return False, f'length diff: {len(in_trimmed)} vs {len(out_trimmed)}'
    return False, 'unknown mismatch'
