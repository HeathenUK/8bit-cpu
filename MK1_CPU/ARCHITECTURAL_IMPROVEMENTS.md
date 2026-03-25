# MK1 CPU — Recommended Enhancements

All proposals assume the existing PCB. No chip swaps, no new ICs — only bodge wires, trace cuts, and EEPROM reflashes.

---

## 1. CURRENT ARCHITECTURE SUMMARY

- **CPU:** 8-bit data bus, 8-bit MAR, 8-bit program counter
- **SRAM:** CY62256 (U47), 32K×8. Only 1K addressable (4 pages × 256 bytes). SRAM A10–A14 are unrouted.
- **Microcode:** 4× AM29F040 EEPROMs (U73–U76), 32-bit control word, 15-bit addressing (8-bit opcode + 3-bit step + 2 flags + 2 IRQ lines)
- **ALU:** 74HCT283 adder, XOR inversion for subtract, 8 modes (1 unused)
- **Registers:** A, B, C, D (general), SP (stack pointer), PC (program counter), E (ALU input), OUT (display)
- **Flags:** CF, ZF, OF, SF in U11 (74HCT173, 4-bit). Only CF and ZF wired to microcode address bus.
- **Step counter:** U72 (74HCT161), 3 bits used (8 steps max), Q3 unconnected
- **Opcodes:** All 256 used. New instructions require reclaiming existing encodings.

### Memory Map

| STK | HL | Page | Address Range | Usage |
|-----|-----|------|--------------|-------|
| 0 | 0 | 0 | 0x000–0x0FF | Program memory (instruction fetch) |
| 0 | 1 | 1 | 0x100–0x1FF | Data memory (load/store) |
| 1 | 0 | 2 | 0x200–0x2FF | Stack |
| 1 | 1 | 3 | 0x300–0x3FF | **Unmapped — never accessed** |

Page selection is hardcoded per microcode step (STK and HL are control signals, not address bits derived from the MAR). The MAR is 8 bits wide. There is no overflow/carry from the MAR into the page select lines. This means **contiguous addressing across pages is not possible** in the current hardware — each page is an independent 256-byte window, and the microcode step determines which page is active.

---

## 2. UNUSED RESOURCES

### 2.1 Spare Microcode Output Bits (U76)

3 unused output bits on U76 (bits 3, 2, 1). Traces exist on the PCB but terminate at NoConn markers. Bodge wires can route these to any destination.

### 2.2 Spare Logic Gates

| IC | Type | Spare Units | Pins | Best Use |
|----|------|-------------|------|----------|
| **U24** | **74HCT86 (XOR)** | **Units 3, 4** | **9,10→8 / 12,13→11** | **INC/DEC carry modification** |
| U28 | 74HCT32 (OR) | Multiple | — | General glue logic |
| U58 | 74HCT08 (AND) | Multiple | — | General glue logic |
| U55 | 74HCT00 (NAND) | Multiple | — | General glue logic |
| U13 | 74HCT02 (NOR) | Multiple | — | General glue logic |
| U60 | 74HCT04 (INV) | Multiple | — | Signal inversion |
| U46 | 74HCT00 (NAND) | Multiple | — | General glue logic |
| U38 | 74HCT139 (2:4 DEC) | Unit 2 | — | Address decode |

### 2.3 Spare Microcode EEPROM Address Lines

A15–A18 on all four EEPROMs are tied low. 4 spare lines = up to 16× expansion of microcode address space.

### 2.4 Spare Step Counter Bit

U72 Q3 is unconnected. Wiring it to EEPROM A15 doubles instruction length from 8 to 16 steps.

### 2.5 Unused ALU Mode

Mode 110 (OR|SHF asserted together) is never used. Its actual hardware behaviour is untested.

### 2.6 Unmapped SRAM (Page 3 + 31K inaccessible)

Page 3 (256 bytes) exists but no microcode step ever asserts STK|HL. Additionally, SRAM A10–A14 are unrouted, leaving 31K of the 32K chip permanently inaccessible.

### 2.7 Wasted Flags

OF and SF are latched in U11 but not wired to the microcode EEPROM address bus. They are computed and stored every ALU operation but never influence instruction behaviour.

### 2.8 Reclaimable Opcodes

**Definitely reclaimable (no functionality lost):**

| Opcode | Binary | Current | Why Expendable |
|--------|--------|---------|----------------|
| `or $a, $a` | 0xE0 | A OR A → A, sets flags | Identical to `and $a, $a` |
| `or $b, $b` | 0xE5 | B OR B → B, sets flags | Identical to `and $b, $b` |
| `or $c, $c` | 0xEA | C OR C → C, sets flags | Identical to `and $c, $c` |
| `or $d, $d` | 0xEF | D OR D → D, sets flags | Identical to `and $d, $d` |

**Probably reclaimable (exotic register combos never used in practice):**

| Pattern | Count |
|---------|-------|
| Self-ops on $c, $d (`add $c,$c`, `sub $d,$d`, etc.) | 4 |
| OR cross-ops between $c and $d | 8 |
| AND cross-ops between $c and $d | 8 |

Estimated total: **4 definite + up to 20 more** depending on how aggressively you sacrifice.

---

## 3. ENHANCEMENTS

Each enhancement is tagged by what it requires:

- **[HW]** — Hardware change: bodge wires and/or trace cuts on the PCB
- **[UC]** — Microcode change: reflash all 4 EEPROMs
- **[ASM]** — Assembler/programmer change: new mnemonics, reclaimed opcodes, or new programming patterns

Every enhancement that adds an instruction requires **[UC] + [ASM]** at minimum (new microcode + new assembler mnemonic). Some also require **[HW]**.

---

### 3.1 INC / DEC

> Add `inc` and `dec` instructions that increment/decrement register A by 1 in a single byte.

**What changes:**

**[HW]** Bodge wire from U76 bit 1 output to U24 unit 3 pin 10. Bodge wire from existing SUB signal to U24 unit 3 pin 9. Bodge wire from U24 pin 8 (XOR output) to the ALU adder carry-in (C0). Cut the existing direct SUB→C0 trace. **Total: 3 wires, 1 trace cut, consumes 1 spare U76 bit and 1 spare XOR gate.**

**[UC]** Define new control signal CINV (U76 bit 1). Microcode for `inc`: assert CINV with no B-operand on bus (A + 0 + 1 = A+1). Microcode for `dec`: assert CINV|SUB (A + NOT(0) + 0 = A + 0xFF = A−1). The existing SUB and ADD paths are unchanged (CINV=0 preserves current behaviour).

| CINV | SUB | Carry-In | B Inverted | Effect |
|------|-----|----------|------------|--------|
| 0 | 0 | 0 | No | ADD (unchanged) |
| 0 | 1 | 1 | Yes | SUB (unchanged) |
| 1 | 0 | 1 | No | INC: A + 0 + 1 |
| 1 | 1 | 0 | Yes | DEC: A + 0xFF |

**[ASM]** Reclaim 2 opcodes (e.g., `or $a, $a` and `or $b, $b`). New mnemonics: `inc` (1 byte), `dec` (1 byte). Replaces 2-byte `addi 1` / `subi 1` sequences. Saves ~36 bytes across existing programs.

---

### 3.2 SWAP

> Swap two registers in a single instruction.

**What changes:**

**[UC]** Multi-step microcode using the E register as a temporary: RegA→EI, RegB→RegA, EO→RegB. 5 steps total including fetch, fits in current 8-step counter. No hardware changes.

**[ASM]** Reclaim 2–4 opcodes. New mnemonics: `swap $a, $b`, etc. Replaces the current 3-instruction (6-byte) push/mov/pop sequence with 1 instruction (1 byte).

---

### 3.3 CLR

> Clear a register to zero in a single instruction.

**What changes:**

**[UC]** Microcode performs SUB of the register with itself (e.g., Reg→EI, SUB|EO|FI→Reg). Sets ZF. 5 steps, fits current counter. No hardware changes.

**[ASM]** Reclaim 1 opcode. New mnemonic: `clr $a` (1 byte). Convenience; replaces `mov $a, 0` (2 bytes with immediate).

---

### 3.4 Page 3 Data Access

> Make the unmapped 4th SRAM page available as 256 bytes of additional data storage.

Page 3 cannot be presented as a contiguous extension of the existing data page. The MAR is 8 bits wide and drives SRAM A0–A7 only. The page select signals (STK, HL) are hardcoded in each microcode step — they are not derived from the address value and there is no carry/overflow from the MAR into the page select. A contiguous 512-byte data space would require either a 9th address bit routed from the MAR to the SRAM (hardware change to the address path, not just a bodge wire) or conditional page-switching within a single microcode sequence (the microcode has no branch capability). Neither is feasible on this PCB. **The separate opcode is a hardware constraint, not an assembler limitation.**

**What changes:**

**[UC]** New microcode sequences identical to existing `load $a, [imm]` / `stor $a, [imm]` but asserting `STK|HL` instead of `HL` alone on the memory-access step:

```
ldp3:  PO|MI, RO|II|PE, PO|MI, PE|RO|MI, STK|HL|RO|AI, RST     (6 steps)
stp3:  PO|MI, RO|II|PE, PO|MI, PE|RO|MI, STK|HL|AO|RI, RST     (6 steps)
```

Fits current 8-step counter. No hardware changes.

**[ASM]** Reclaim 2 opcodes minimum ($a only), up to 8 for all four GP registers. New mnemonics: `ldp3 $a, [imm]` / `stp3 $a, [imm]` (2 bytes each). The programmer must use these explicitly — existing `load`/`stor` instructions continue to access page 1.

**Use cases:** Lookup tables (sin/cos, font data, character maps), extra variable storage, double-buffering. Doubles usable data space from 256 to 512 bytes.

---

### 3.5 Extend Step Counter to 16 Steps

> Allow instructions to use up to 16 microcode steps instead of 8.

**What changes:**

**[HW]** Bodge wire from U72 Q3 (currently NoConn) to A15 on each of U73–U76. Cut/lift A15 GND ties on all 4 EEPROMs. Possibly adjust reset logic if RST doesn't already clear Q3. **Total: 5 bodge wires + 4 GND lifts.**

Note: this consumes EEPROM address line A15. If enhancement 3.7 (expose OF/SF) also needs A15, these two compete for the same resource. **Only one of {3.5, 3.7} can use A15 unless A16 is used for the other.** With both, A15 = Q3 (step bit 3) and A16 = OF (or SF). This is feasible — there are 4 spare address lines (A15–A18).

**[UC]** Update `microcode.py` to use 4-bit step addressing. All existing instructions work unchanged (steps 8–15 just assert RST). New complex instructions can use up to 16 steps.

**[ASM]** No change to existing programs. Enables future instructions that need more than 8 steps (prerequisite for 3.6).

---

### 3.6 Stack-Relative Load ($a only)

> `ld $a, [SP+n]` — read a value from a stack frame by offset.

**Restricted to $a as destination.** Computing SP+offset requires the ALU: SP goes into E, the immediate offset goes into A, and EO (the ALU result) goes to the MAR. This works when A is the destination because it's being overwritten anyway. For other registers, the programmer writes two instructions: `ld $a, [SP+n]` then `mov $a, $b`.

Stack-relative **store** (`st $a, [SP+n]`) has a fundamental register pressure problem: computing the address requires A for the ALU addition, but A also holds the value to be stored. There is no invisible scratch location to save it. Recommendation: **load only.** Stack writes use `push` or manual SP adjustment.

**What changes:**

**[HW]** Requires 3.5 (16-step counter) for the store variant if it were implemented, but the load variant fits in 8 steps with the current counter:

```
PO|MI, RO|II|PE, SO|EI, PO|MI, PE|RO|AI, EO|MI, STK|RO|AI, RST
```

If only implementing the load, **no hardware change beyond what 3.5 already provides** — and the load actually fits in 8 steps even without 3.5.

**[UC]** New microcode sequence (see above). The ALU is in ADD mode (default). EO outputs SP + offset. The final step reads from the stack page at the computed address.

**[ASM]** Reclaim 1–2 opcodes. New mnemonic: `ld $a, [SP+n]` (2 bytes: opcode + offset). Enables compiler-generated code to access local variables and function parameters by stack frame offset.

---

### 3.7 Expose OF/SF to Microcode

> Use the existing overflow and sign flags for conditional branching.

**What changes:**

**[HW]** Bodge wire from U11 OF output to A15 (or A16) on all 4 EEPROMs. Bodge wire from U11 SF output to A16 (or A17) on all 4 EEPROMs. Cut/lift the corresponding GND ties. **Total: 8 bodge wires + 8 GND lifts.** Must coordinate with 3.5 if both are implemented (see 3.5 note on address line sharing).

**[UC]** Expand microcode generator addressing to include OF and SF. Add conditional branch variants that activate on flag combinations:

| Condition | Flags Tested | Use |
|-----------|-------------|-----|
| BLT (branch less than, signed) | SF ≠ OF | Signed < |
| BGE (branch greater/equal, signed) | SF = OF | Signed ≥ |
| BLE (signed ≤) | ZF=1 or SF≠OF | Signed ≤ |
| BGT (signed >) | ZF=0 and SF=OF | Signed > |

**[ASM]** Reclaim 2–4 opcodes for new branch mnemonics. Enables signed arithmetic comparisons — currently the CPU can only branch on zero/carry (unsigned).

---

## 4. IMPLEMENTATION ORDER

| # | Enhancement | Tags | Opcodes | Hardware | Dependencies |
|---|-------------|------|---------|----------|-------------|
| 1 | INC / DEC | HW, UC, ASM | 2 | 3 wires, 1 cut, 1 XOR gate, 1 U76 bit | None |
| 2 | SWAP | UC, ASM | 2–4 | None | None |
| 3 | CLR | UC, ASM | 1 | None | None |
| 4 | Page 3 data (ldp3/stp3) | UC, ASM | 2–8 | None | None |
| 5 | 16-step counter | HW, UC | 0 | 5 wires, 4 GND lifts | None |
| 6 | Stack-relative LD | UC, ASM | 1–2 | None | None (fits 8 steps) |
| 7 | Expose OF/SF | HW, UC, ASM | 2–4 | 8 wires, 8 GND lifts | Coordinate A15–A18 with #5 |

**Total opcode budget:** 10–21 opcodes. Available from reclamation: 4 definite + up to 20 probable = 24 max.

**Total spare U76 bits consumed:** 1 (CINV for INC/DEC). 2 remaining for future use.

**EEPROM address line allocation** (if all enhancements implemented):

```
A15:  Step counter Q3 (enhancement #5)
A16:  Overflow Flag (enhancement #7)
A17:  Sign Flag (enhancement #7)
A18:  Spare
```

---

## 5. REFERENCE

### Control Signal Map

**U73 — Bits 31–24:**

| Bit | Signal | Function |
|-----|--------|----------|
| 31 | HLT | Halt clock |
| 30 | STK | Stack page select (SRAM A9) |
| 29 | PE | Program counter increment |
| 28–26 | RegIn[2:0] | Register input select (decoded by U77 74HCT138) |
| 25 | MI | Memory address register in |
| 24 | RI | RAM data in |

**U74 — Bits 23–16:**

| Bit | Signal | Function |
|-----|--------|----------|
| 23 | II | Instruction register in |
| 22 | OI | Output register in |
| 21 | XI | External interface in |
| 20–18 | RegOut[2:0] | Register output select (decoded by U78/U79 74HCT238) |
| 17 | RO | RAM data out |
| 16 | IO | Instruction register out (immediate) |

**U75 — Bits 15–8:**

| Bit | Signal | Function |
|-----|--------|----------|
| 15 | SUB | ALU mode bit 2 |
| 14 | OR | ALU mode bit 1 |
| 13 | SHF | ALU mode bit 0 |
| 12 | FI | Flags register in |
| 11 | SU | Stack pointer count up |
| 10 | SD | Stack pointer count down |
| 9 | U0 | External user signal 0 |
| 8 | U1 | External user signal 1 |

**U76 — Bits 7–0:**

| Bit | Signal | Function |
|-----|--------|----------|
| 7 | E0 | External enable 0 |
| 6 | E1 | External enable 1 |
| 5 | HL | Data page select (SRAM A8) |
| 4 | RGT | Right shift/rotate direction |
| 3 | — | **AVAILABLE** |
| 2 | — | **AVAILABLE** |
| 1 | — | **AVAILABLE** (proposed: CINV) |
| 0 | RST | Reset step counter |

### Register Select Encoding

| Code | Register In | Register Out |
|------|------------|-------------|
| 000 | (none) | (none) |
| 001 | AI | AO |
| 010 | BI | BO |
| 011 | CI | CO |
| 100 | DI | DO |
| 101 | SI | PO |
| 110 | EI | SO |
| 111 | PI | EO |

### Microcode EEPROM Addressing (current)

```
A14:      IRQ1
A13:      IRQ0
A12:      Zero Flag
A11:      Carry Flag
A10–A3:   Instruction opcode (IR_7–IR_0)
A2–A0:    Step counter (Q0–Q2)
A15–A18:  Tied low (available)
```

### ALU Carry-In Path

SUB drives both the XOR inversion gates (U21/U22, B-operand inversion) and the adder carry-in (C0) directly. The INC/DEC modification (enhancement 3.1) interposes a XOR gate on the C0 path only, leaving B-inversion intact.
