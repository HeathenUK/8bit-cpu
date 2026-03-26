# MK1 CPU Enhancement Plan

Comprehensive plan for improving the MK1 8-bit CPU via bodge wires, trace cuts, and microcode changes only. No new ICs.

---

## Current Architecture Summary

- **CPU:** 8-bit data bus, 8-bit MAR, 8-bit program counter
- **SRAM:** CY62256 (U47), 32K×8. Only 1K addressable (4 pages × 256 bytes). SRAM A10–A14 unrouted.
- **Microcode:** 4× AM29F040 EEPROMs (U73–U76), 32-bit control word, 15-bit addressing (8-bit opcode + 3-bit step + 2 flags + 2 IRQ lines)
- **ALU:** 74HCT283 dual adder, XOR inversion for subtract, 8 modes (1 unused)
- **Registers:** A, B, C, D (general), SP (stack pointer), PC (program counter), E (ALU input), OUT (display)
- **Flags:** CF, ZF, OF, SF in U11 (74HCT173, 4-bit). Only CF and ZF wired to microcode address bus.
- **Step counter:** U72 (74HCT161), 3 bits used (8 steps max), Q3 unconnected
- **Opcodes:** All 256 used. New instructions require reclaiming existing encodings.
- **Bus:** 10K pull-down resistor network (RN1, SIP-9) on all 8 data lines. Undriven bus reads 0x00.
- **Output:** 4× seven-segment display, multiplexed via NE555 (U33) + 74HCT107 counter (U35) + 74HCT139 decoder (U38). Display decode EEPROM is 28C64 (8K×8). Mode currently set by DIP switch SW6.
- **External bus:** 2 device slots (E0/E1), 2 user signals (U0/U1), 2 IRQ lines, bidirectional via 74HCT245 (U70). IRQ latched by U71 (74HCT107 dual JK flip-flop).
- **Program loading:** Arduino Nano USB uploader + Start9 flash-based loader (16 program banks).

### Memory Map

| STK | HL | Page | Address Range | Usage |
|-----|-----|------|--------------|-------|
| 0 | 0 | 0 | 0x000–0x0FF | Program memory (instruction fetch) |
| 0 | 1 | 1 | 0x100–0x1FF | Data memory (load/store) |
| 1 | 0 | 2 | 0x200–0x2FF | Stack |
| 1 | 1 | 3 | 0x300–0x3FF | **Unmapped — never accessed** |

---

## Available Resources

### U76 Output Bits (Bits 7–0)

| Bit | Signal | Status |
|-----|--------|--------|
| 7 | E0 (ext enable 0) | Occupied — external bus |
| 6 | E1 (ext enable 1) | Occupied — external bus |
| 5 | HL (data page select) | Occupied |
| 4 | RGT (shift/rotate dir) | Occupied |
| 3 | Unconnected | **FREE** |
| 2 | Unconnected | **FREE** |
| 1 | Unconnected | **FREE** |
| 0 | RST (reset step ctr) | Occupied |

### Spare EEPROM Address Lines

A15–A18 on all four microcode EEPROMs (AM29F040, 512KB) are tied to GND. 4 spare lines available by lifting GND and bodge-wiring signals.

### Reclaimable Opcodes

**Definite (no functionality lost):** `or $a,$a` (0xE0), `or $b,$b` (0xE5), `or $c,$c` (0xEA), `or $d,$d` (0xEF) — identical to their AND equivalents.

**Probable (exotic register combos unused in practice):** ~20 additional self-ops and cross-ops on $c/$d.

**From sacrificing external device slot 1:** `exw 1 0`–`exw 1 3`, `exr 1`, `je1` = 6 additional opcodes.

**Total budget: ~30 reclaimable opcodes.**

### Spare Hardware

| IC | Type | Available | Notes |
|----|------|-----------|-------|
| U24 | 74HCT86 (XOR) | Units 3, 4 | 2 spare gates |
| U28 | 74HCT32 (OR) | Multiple | Spare gates |
| U58 | 74HCT08 (AND) | Multiple | Spare gates |
| U55 | 74HCT00 (NAND) | Multiple | Spare gates |
| U13 | 74HCT02 (NOR) | Multiple | Spare gates |
| U60 | 74HCT04 (INV) | Multiple | Spare inverters |
| U71 FF2 | 74HCT107 (JK FF) | 1 flip-flop (if device slot 1 sacrificed) | For display mode latch |

---

## Enhancement 1: INC / DEC

**What it achieves:** Single-byte increment/decrement of register A. Replaces the 2-byte `addi 1 $a` / `subi 1 $a` pattern. Saves ~36 bytes across existing programs.

**Hardware (3 bodge wires, 1 trace cut):**
- Route new CINV signal (U76 bit 1 output) to spare XOR gate U24 unit 3 pin 10
- Route existing SUB signal to U24 unit 3 pin 9
- Route U24 pin 8 (XOR output) to ALU adder carry-in (C0 of U15)
- Cut existing direct SUB → C0 trace

**Truth table:**

| CINV | SUB | Carry-In (CINV XOR SUB) | B Inverted | Effect |
|------|-----|-------------------------|------------|--------|
| 0 | 0 | 0 | No | ADD (unchanged) |
| 0 | 1 | 1 | Yes | SUB (unchanged) |
| 1 | 0 | 1 | No | INC: A + 0 + 1 |
| 1 | 1 | 0 | Yes | DEC: A + 0xFF |

**Microcode:**

INC requires E = 0. The bus has 10K pull-downs (RN1) so asserting `EI` with no bus driver loads 0x00 into E. This is the same mechanism the existing `not` instruction relies on (its step 3 asserts `AI` with no bus driver).

```
inc:  [MI|PO, RO|II|PE, EI, CINV|EO|AI|FI, RST, RST, RST, RST]
dec:  [MI|PO, RO|II|PE, EI, CINV|SUB|EO|AI|FI, RST, RST, RST, RST]
```

**Opcodes consumed:** 2 (reclaim `or $a,$a` and `or $b,$b`).

---

## Enhancement 2: Page 3 Data Access

**What it achieves:** Doubles usable data memory from 256 to 512 bytes. The 4th SRAM page (STK=1, HL=1) exists in hardware but no microcode ever addresses it. Ideal for lookup tables, extra arrays, double-buffering.

**Hardware:** None.

**Microcode:** New sequences identical to existing `ld`/`st` with immediate addressing, but asserting `STK|HL` instead of `HL` on the memory-access step:

```
ldp3 $a, [imm]:  [MI|PO, RO|II|PE, PO|MI, PE|RO|MI, STK|HL|RO|AI, RST, RST, RST]
stp3 $a, [imm]:  [MI|PO, RO|II|PE, PO|MI, PE|RO|MI, STK|HL|AO|RI, RST, RST, RST]
```

**Opcodes consumed:** 2–8 (2 minimum for $a, up to 8 for all GP registers).

---

## Enhancement 3: CLR

**What it achieves:** Clears register A to zero in 1 byte. Replaces `ldi $a 0` (2 bytes).

**Hardware:** None.

**Microcode:** Self-subtract (A + NOT(A) + 1 = 0). FI deliberately NOT asserted to avoid setting CF=1 as a side effect:

```
clr:  [MI|PO, RO|II|PE, AO|EI, EO|AI|SUB, RST, RST, RST, RST]
```

**Opcodes consumed:** 1.

---

## Enhancement 4: SWAP

**What it achieves:** Swaps two registers in a single 1-byte instruction. Replaces the 3-instruction / 6-byte push/mov/pop sequence.

**Hardware:** None.

**Important note:** The E register CANNOT be used as a temporary for SWAP because EO is the ALU output, not a register passthrough. EO always computes A ⊕ E through the active ALU mode. The corrected approach uses the stack as scratch:

**Microcode (swap $a, $b):**

```
swap $a,$b:  [MI|PO, RO|II|PE, SO|MI, STK|AO|RI|SD, BO|AI|SU, SO|MI, STK|RO|BI, RST]
```

Step-by-step:
- Step 2: Read SP to MAR
- Step 3: Write A to stack at SP, decrement SP (standard push pattern)
- Step 4: Move B → A AND increment SP back (net SP change = 0)
- Step 5: Re-read SP to MAR (same address as step 2)
- Step 6: Read original A from stack into B

SP is temporarily modified (decremented then incremented) but net change is zero. No mid-instruction interrupts exist, so this is safe.

**Opcodes consumed:** 2–6 (depending on how many register pair combinations).

---

## Enhancement 5: Display Mode Control

**What it achieves:** Software-selectable output display mode at runtime — unsigned decimal, signed decimal, hexadecimal, or ASCII. Currently requires physical DIP switch change.

**Prerequisite:** Sacrifice external device slot 1 (from 2 down to 1) to free one U71 JK flip-flop for use as a mode latch.

**Hardware (~11 bodge wires, ~3 trace cuts):**

1. **U76 bit 2** → DM (display mode latch clock) signal
2. **Freed U71 flip-flop** (from IRQ1) wired as D-latch for mode bit 0:
   - BUS_0 → J input
   - BUS_0 → spare U60 inverter → K input
   - DM → CLK input
   - System reset → CLR (defaults to mode 0 on power-up)
   - Q output → display EEPROM (28C64) A11
3. **SR latch** from 2 spare NAND gates (U55) for mode bit 1:
   - DM AND BUS_1 (spare U58 AND gate) → Set input
   - DM AND NOT(BUS_1) (spare U60 inverter + spare U58 AND gate) → Reset input
   - Q output → display EEPROM (28C64) A12
4. **Cut traces** from DIP switch SW6 to display EEPROM A11/A12.

**Gate consumption:** 2 inverters (U60) + 2 AND gates (U58) + 2 NAND gates (U55) = 6 spare gates.

**Display EEPROM (28C64) reprogramming required** — reorganise layout so all 4 modes sit at clean 1024-byte offsets:

| A12 | A11 | Mode | EEPROM offset |
|-----|-----|------|--------------|
| 0 | 0 | Unsigned decimal | 0x000 |
| 0 | 1 | Signed (2's complement) | 0x400 |
| 1 | 0 | Hexadecimal | 0x800 |
| 1 | 1 | ASCII | 0xC00 |

Note: the existing `output_display_extended.py` places ASCII at offset 0x1000 which doesn't match a clean 2-bit scheme. Must regenerate with the layout above.

**Microcode:**

```
sdm:  [MI|PO, RO|II|PE, AO|DM, RST, RST, RST, RST, RST]
```

**Usage:**
```asm
ldi $a 0      ; unsigned decimal
sdm
ldi $a 42
out $a        ; displays "42"

ldi $a 2      ; hex mode
sdm
ldi $a 0xFF
out $a        ; displays "FF"
```

**Opcodes consumed:** 1. **U76 bits consumed:** 1 (bit 2).

---

## Enhancement 6: Expose OF/SF Flags

**What it achieves:** Signed arithmetic becomes a first-class capability. The overflow and sign flags are already computed and latched in U11 every ALU operation — they're just not wired to the microcode EEPROMs. Enables proper signed conditional branches.

**Pre-implementation check:** Verify on the physical board that U11 D2 and D3 carry distinct OF and SF signals (not duplicates of CF/ZF or tied to ground). Probe during ALU operations before cutting any traces.

**Hardware (8 bodge wires, 8 GND lifts):**
- Wire U11 Q2 (OF) to EEPROM address line A16 on all 4 EEPROMs (U73–U76)
- Wire U11 Q3 (SF) to EEPROM address line A17 on all 4 EEPROMs
- Cut/lift A16 and A17 GND ties on each EEPROM

**Microcode:** Expand `microcode.py` flag addressing from 4 bits to 6 bits. Add conditional branch variants:

| Condition | Flags Tested | Use |
|-----------|-------------|-----|
| BLT (branch less than, signed) | SF ≠ OF | Signed < |
| BGE (branch greater/equal, signed) | SF = OF | Signed ≥ |

**Opcodes consumed:** 2–4.

---

## Enhancement 7: Stack-Relative Load

**What it achieves:** `ld $a, [SP+n]` — read a value from the stack by offset without popping. Enables local variables and function parameter access.

**Hardware:** None. Fits in 8 steps.

**Microcode:**

```
ld $a,[SP+n]:  [MI|PO, RO|II|PE, SO|EI, PO|MI, PE|RO|AI, EO|MI, STK|RO|AI, RST]
```

Step-by-step:
- Step 2: SP → E register (ALU input)
- Step 3–4: Fetch immediate operand (offset n) into A
- Step 5: EO = A + E = n + SP → MAR (address computation)
- Step 6: Read from stack page at computed address → A

Restricted to $a as destination (ALU constraint). FI deliberately NOT asserted — the flags from the address addition (SP + offset) would be meaningless.

**Opcodes consumed:** 1–2.

---

## Enhancement 8: 16-Step Counter

**What it achieves:** Doubles maximum instruction complexity from 8 to 16 micro-steps. Pure future-proofing — all proposed enhancements fit in 8 steps.

**Hardware (5 bodge wires, 4 GND lifts):**
- Wire U72 Q3 (currently unconnected) to EEPROM address line A15 on all 4 EEPROMs
- Cut/lift A15 GND ties on each EEPROM
- Verify system reset clears Q3

**Microcode:** Update `microcode.py` to 4-bit step addressing. Existing instructions unchanged (steps 8–15 assert RST).

**Opcodes consumed:** 0.

---

## Resource Budget After All Enhancements

| Resource | Total | Used | Remaining |
|----------|-------|------|-----------|
| U76 spare bits (1, 2, 3) | 3 | 2 (CINV + DM) | **1 (bit 3)** |
| EEPROM address lines (A15–A18) | 4 | 3 (Q3 + OF + SF) | **1 (A18)** |
| Reclaimable opcodes | ~30 | 11–24 | **6–19** |
| External device slots | 2 | 1 sacrificed for display mode | **1** |
| U71 flip-flops | 2 | 1 for IRQ0, 1 for display mode | **0** |

---

## What You Get

| Capability | Before | After |
|-----------|--------|-------|
| Data memory | 256 bytes | **512 bytes** |
| Code density | `addi 1` = 2 bytes, `ldi $a 0` = 2 bytes | `inc` = 1 byte, `clr` = 1 byte (~15% savings) |
| Register swap | 3 instructions / 6 bytes | **1 instruction / 1 byte** |
| Signed arithmetic | Software hack via `andi 128` | **Native signed branches (BLT, BGE)** |
| Display mode | Physical DIP switch | **Software-controlled at runtime** |
| Stack access | Pop-only | **Direct offset read: `ld $a, [SP+n]`** |
| Instruction complexity ceiling | 8 micro-steps | **16 micro-steps** |

---

## Implementation Order

| # | Enhancement | Hardware | Microcode | Dependencies |
|---|------------|----------|-----------|-------------|
| 1 | INC / DEC | 3 wires, 1 cut | Yes | None |
| 2 | Page 3 data | None | Yes | None |
| 3 | CLR | None | Yes | None |
| 4 | SWAP | None | Yes | None |
| 5 | Display Mode | ~11 wires, ~3 cuts | Yes + display EEPROM | Sacrifice device slot 1 |
| 6 | OF/SF flags | 8 wires, 8 lifts | Yes | Verify U11 D2/D3 on hardware |
| 7 | Stack-relative LD | None | Yes | None |
| 8 | 16-step counter | 5 wires, 4 lifts | Yes | None |

Enhancements 1–4 and 7 are microcode-only or near-microcode-only and can be done first. Enhancement 5 requires the most bodge wiring but is independent of the others. Enhancement 6 requires physical verification before committing. Enhancement 8 is optional future-proofing.

All 4 microcode EEPROMs should be reflashed once at the end after all microcode changes are made in `microcode.py`.

---

## Known Risks and Mitigations

| Enhancement | Risk | Severity | Mitigation |
|------------|------|----------|-----------|
| INC/DEC | Bus must read 0x00 when undriven for E=0 | **Resolved** | Confirmed: RN1 (10K SIP-9) pulls all bus lines to GND |
| SWAP | EO is ALU output, not E passthrough | **Resolved** | Use stack as scratch (corrected microcode above) |
| CLR | Self-subtract sets CF=1 | Low | FI not asserted — flags untouched |
| Display Mode | 28C64 EEPROM layout mismatch | Low | Regenerate `output_display_extended.py` with 1K offsets |
| Stack-rel LD | Address computation would set wrong flags | Low | FI not asserted |
| OF/SF | U11 D2/D3 wiring unconfirmed from schematics | Low | Probe on physical hardware before committing |
| Opcodes | Self-OR reclamation could break user programs | Low | Document; no sample programs use self-OR |

---

## Cross-Enhancement Compatibility

All 8 enhancements have been checked for mutual compatibility:

- **INC/DEC vs all ALU enhancements:** CINV only affects carry-in path. When CINV=0, existing ADD/SUB behaviour is unchanged.
- **OF/SF vs 16-step counter:** Use different EEPROM address lines (A15 for Q3, A16 for OF, A17 for SF). No conflict.
- **Page 3 vs stack-relative LD:** Different page select signals (STK|HL vs STK). Different SRAM pages.
- **Display mode vs remaining external device:** Device slot 0 kept intact. E0, U0, U1, IRQ0 all unaffected.
- **All enhancements share the same 4 microcode EEPROMs:** No conflict — reflash once with all changes.
