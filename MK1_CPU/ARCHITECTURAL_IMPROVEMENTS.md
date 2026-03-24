# MK1 CPU — Architectural Improvements Reference

Comprehensive audit of unused resources and actionable improvement proposals.

---

## 1. UNUSED RESOURCES INVENTORY

### 1.1 Spare Microcode Output Bits (U76 EEPROM)

The 32-bit microcode word is spread across 4 AM29F040 EEPROMs (U73–U76). U76 carries bits 7–0:

| Bit | Signal | Status |
|-----|--------|--------|
| 7 | E0 | Used — External enable 0 |
| 6 | E1 | Used — External enable 1 |
| 5 | HL | Used — High/Low address mode |
| 4 | RGT | Used — Right shift/rotate |
| **3** | **—** | **UNUSED** — NoConn on PCB trace |
| **2** | **—** | **UNUSED** — NoConn on PCB trace |
| **1** | **—** | **UNUSED** — NoConn on PCB trace |
| 0 | RST | Used — Reset step counter |

**3 spare control signal bits available.** The EEPROM outputs already have traces routed to a location on the PCB (terminating at NoConn markers near coordinates 9200,9850–10050 in control.sch). Bodge wires can tap these traces and route them to new destinations.

### 1.2 Spare Logic Gates

**Confirmed spare XOR gates (critical for INC/DEC):**

| IC | Type | Location | Spare Units | Pins |
|----|------|----------|-------------|------|
| U24 | 74HCT86 (Quad XOR) | alu.sch / control.sch | **Units 3, 4** | Unit 3: pins 9,10→8 / Unit 4: pins 12,13→11 |

Note: U21 and U22 (also 74HCT86) have ALL 4 gates used for two's complement inversion. U24 is the only source of spare XOR gates.

**Other confirmed spare gates (selected, most useful):**

| IC | Type | Location | Spare Units | Gate Function |
|----|------|----------|-------------|---------------|
| U28 | 74HCT32 | a-register.sch / alu.sch | Multiple | OR |
| U58 | 74HCT08 | a-register.sch | Multiple | AND |
| U55 | 74HCT00 | stack-register.sch | Multiple | NAND |
| U13 | 74HCT02 | alu.sch | Multiple | NOR |
| U60 | 74HCT04 | control.sch | Multiple | Inverter |
| U46 | 74HCT00 | ram.sch | Multiple | NAND |
| U38 | 74HCT139 | output.sch | Unit 2 | 2-to-4 Decoder |

### 1.3 Spare EEPROM Address Lines

Each AM29F040 has 19 address lines (A0–A18). Only 15 are used:

| Address Bits | Width | Function |
|-------------|-------|----------|
| A0–A2 | 3 | Step counter (U72 Q0–Q2) |
| A3–A10 | 8 | Instruction opcode (IR_0–IR_7) |
| A11 | 1 | Carry Flag (CF) |
| A12 | 1 | Zero Flag (ZF) |
| A13 | 1 | IRQ0 |
| A14 | 1 | IRQ1 |
| **A15–A18** | **4** | **UNUSED — tied low** |

**4 spare address lines** = potential 16× expansion of microcode space.

### 1.4 Spare Step Counter Bit

U72 (74HCT161, 4-bit counter) only uses Q0–Q2 (3 bits → 8 steps). **Q3 is unconnected (NoConn).** Connecting Q3 to A15 on all EEPROMs would extend instructions from 8 to 16 steps.

### 1.5 Unused ALU Mode

ALU operation is encoded in 3 bits (SUB, OR, SHF):

| Mode | Bits (SUB,OR,SHF) | Function |
|------|-------------------|----------|
| 000 | 0,0,0 | ADD |
| 001 | 1,0,0 | SUB |
| 010 | 0,1,0 | OR |
| 011 | 1,1,0 | AND |
| 100 | 0,0,1 | SHF (shift) |
| 101 | 1,0,1 | ROT (rotate) |
| **110** | **0,1,1** | **UNUSED** |
| 111 | 1,1,1 | NOT |

Mode 110 (OR|SHF) is never asserted. Its hardware behaviour depends on how the ALU decodes these bits — it would need investigation to determine what it actually does on the current hardware before it could be repurposed.

### 1.6 Opcode Space

**All 256 opcodes are used.** The encoding:

| Bits 7–6 | Range | Category | Count |
|----------|-------|----------|-------|
| 00 | 0x00–0x3F | MOV (register-register, specials) | 64 |
| 01 | 0x40–0x7F | LOAD (memory read, bitwise ops) | 64 |
| 10 | 0x80–0xBF | STORE (memory write, ALU-imm) | 64 |
| 11 | 0xC0–0xFF | ALU (register-register) | 64 |

To add new instructions, opcodes must be **reclaimed** from existing encodings. Candidates for sacrifice are listed in Section 3.

### 1.7 Flags Register

U11 (74HCT173, 4-bit latch) stores CF, ZF, OF, SF — **all 4 bits used, no spare flag positions** without replacing the chip (e.g., with a 74HCT273 for 8 flag bits).

---

## 2. IMPROVEMENT PROPOSALS

### TIER 1: Microcode-Only (zero hardware changes)

These require only reflashing the microcode EEPROMs.

#### 2.1a — Reclaim Expendable Opcodes

Several existing opcodes encode register combinations that are useless or redundant:

**MOV range (0x00–0x3F) — candidates for reclamation:**
- `mov $out, $X` for most X: $out has no output path, so reading from it is meaningless. The slots where $out is the SOURCE (bits `00_110_XXX`) are mostly wasted. That's up to 7 opcodes, though `mov $out, $a` is actually `out` (writes A to display), and similar for other destinations.
- `mov $pc, $pc`: jumps to current PC+1, functionally a NOP. Could be reclaimed.
- `mov $sp, $sp`: NOP equivalent. Could be reclaimed.

**LOAD range special slots:**
- `load $out, [imm]` — loads from memory into the output register. Rarely useful since `out` is typically written from registers. Could be reclaimed.

**STORE range:**
- Several `stor $X, [$sp]` combinations overlap with push/pop semantics but with different behaviour. Some may be reclaimable.

**ALU range (0xC0–0xFF):**
- Self-operations like `add $a, $a` (doubles $a), `and $a, $a` (NOP with flags), `or $a, $a` (NOP with flags) are technically useful but `and $X, $X` and `or $X, $X` are redundant with each other — one set could be reclaimed. That's up to **16 opcodes** (4 self-ops × 4 registers for either AND or OR).

**Estimated reclaimable: 10–25 opcodes** depending on how aggressively you sacrifice.

#### 2.1b — Microcode Timing Optimisation

Most instructions use 3–5 of the 8 available steps, with remaining steps asserting RST. No speed improvement possible within the current step counter, but if steps are extended to 16 (see Tier 3), complex multi-step instructions become feasible.

---

### TIER 2: Microcode + Bodge Wires (using spare U76 bits and spare gates)

#### 2.2a — INC / DEC Instructions

**Goal:** Single-byte `inc` and `dec` that operate on register A (or any register), replacing 2-byte `addi 1` / `subi 1`.

**Mechanism:**
- Use **1 spare U76 bit** (e.g., bit 1) as a new control signal: `CINV` (carry invert)
- Bodge wire from U76 output pin (bit 1) to **U24 unit 3** (spare XOR gate, pins 9,10→8)
- Connect: `SUB` signal → U24 pin 9, `CINV` → U24 pin 10
- XOR output (U24 pin 8) replaces the direct SUB→C0 connection on the ALU adder carry-in
- Cut/lift the existing SUB→C0 trace

**Truth table:**

| CINV | SUB | Carry-In (XOR) | B inversion | Effect |
|------|-----|----------------|-------------|--------|
| 0 | 0 | 0 | No | Normal ADD: A + B |
| 0 | 1 | 1 | Yes | Normal SUB: A − B |
| 1 | 0 | 1 | No | A + B + 1 (INC when B=0) |
| 1 | 1 | 0 | Yes | A + NOT(B) (DEC when B=0: A + 0xFF = A−1) |

**Hardware cost:** 1 bodge wire from U76 to U24, 1 bodge wire for SUB to U24, 1 wire from U24 output to adder C0, cut 1 trace. Total: **3 wires + 1 cut + 1 spare XOR gate.**

**Software cost:** Reclaim 2+ opcodes for `inc` and `dec`. Microcode asserts CINV (and not SUB for INC, or SUB for DEC) with no register on the bus (bus floats to 0x00 or is explicitly zeroed).

**Impact:** Saves 25 bytes across existing programs (INC) + 11 bytes (DEC) = **36 bytes total**. Every loop counter, pointer bump, and stack adjustment becomes 1 byte instead of 2.

#### 2.2b — Register Swap

**Goal:** Swap two registers in a single instruction instead of the current 3-instruction push/mov/pop sequence.

**Mechanism:**
- Use **1 spare U76 bit** (e.g., bit 2) as `SWAP` control signal
- This enables a temporary latch or uses a multi-step microcode sequence that reads one register into a temporary hold, reads the other onto the bus, writes the first destination, then outputs the held value
- Actually: **this can be done with microcode alone** using 4 steps: RegA→EI, RegB→RegA, EO→RegB (using the ALU E register as a temp). No extra hardware needed, just a reclaimed opcode.

**Impact:** Common in sort algorithms (bubble sort, quicksort). Currently requires 3 instructions (6 bytes); swap reduces to 1 instruction (1 byte).

#### 2.2c — Zero Register / Immediate Zero on Bus

**Goal:** Put 0x00 on the bus without reading from RAM.

**Mechanism:**
- Use **1 spare U76 bit** (e.g., bit 3) to enable a pull-down network on the bus
- OR: assert ALU output (EO) with ADD mode and no input — if the ALU outputs 0 when E register contains 0 and no operand is loaded, this may already work

**Simpler alternative:** Just define a microcode-only `clr $X` instruction that does `XOR $a, $a` (which is `SUB $a, $a` followed by move). This needs no hardware, just a reclaimed opcode.

---

### TIER 3: Hardware Modifications (bigger bodges or new chips)

#### 2.3a — Extend Step Counter to 16 Steps

**Goal:** Allow instructions to use up to 16 microcode steps instead of 8.

**Mechanism:**
- Connect U72 Q3 (currently NoConn) to A15 on all 4 EEPROMs
- Update microcode generator to use 4-bit step addressing
- Regenerate and reflash all EEPROMs

**Hardware cost:** 5 bodge wires (Q3 to A15 on each of U73–U76, plus possibly one for the step counter reset logic).

**Impact:** Enables complex multi-step instructions like stack-relative load/store, multiply, block copy. Essential prerequisite for stack-relative addressing.

#### 2.3b — Stack-Relative Addressing

**Goal:** `ld $a, [SP+n]` and `st $a, [SP+n]` — access stack frame variables by offset.

**Mechanism — Option A (ALU-assisted, microcode-heavy):**
1. Extend step counter to 16 steps (Tier 3a above)
2. Microcode sequence: load SP into ALU input, load immediate offset into ALU, add them, output result to MAR, then read/write RAM
3. Requires ~8–10 microcode steps (hence the need for 16-step counter)
4. No new hardware beyond the step counter extension

**Mechanism — Option B (dedicated adder):**
1. Add a 74HCT283 (4-bit adder) or two, wired between SP output and MAR input
2. The immediate offset feeds the adder's B input from the instruction register or bus
3. Faster (fewer microcode steps) but more hardware

**Impact:** Transformative for compiler-generated code. Enables proper stack frames, local variables, function parameters by offset. Makes the CPU a viable compiler target.

#### 2.3c — Additional Flags

**Goal:** Parity flag, half-carry, or other flags for BCD arithmetic or string operations.

**Mechanism:**
- Replace U11 (74HCT173, 4-bit) with 74HCT273 (8-bit latch)
- Connect additional ALU output bits to the new flag inputs
- Use spare EEPROM address lines (A15–A16) to incorporate new flags into conditional microcode

**Hardware cost:** 1 chip swap + bodge wires for new flag sources + EEPROM address rewiring.

**Impact:** Enables signed comparison (overflow flag already exists but isn't used for branching), BCD support, etc.

---

## 3. OPCODE RECLAMATION MAP

Opcodes that could be sacrificed for new instructions, ordered by expendability:

### Definitely Reclaimable (no real functionality lost)

| Opcode | Binary | Current Function | Why Expendable |
|--------|--------|-----------------|----------------|
| `or $a, $a` | 11100000 | A = A OR A (= A, sets flags) | Identical effect to `and $a, $a` |
| `or $b, $b` | 11100101 | B = B OR B | Identical effect to `and $b, $b` |
| `or $c, $c` | 11101010 | C = C OR C | Identical effect to `and $c, $c` |
| `or $d, $d` | 11101111 | D = D OR D | Identical effect to `and $d, $d` |

These 4 opcodes set flags identically to the AND self-ops. Freeing them gives us 4 opcodes for INC/DEC/SWAP/CLR.

### Probably Reclaimable (exotic register combos)

Many ALU operations between unusual register pairs are never used in any existing program and unlikely to be emitted by a compiler (which would use $a as accumulator):

| Pattern | Count | Examples |
|---------|-------|---------|
| `add $c, $c` / `sub $c, $c` etc. | 4 | Self-ops on $c, $d |
| `or $c, $d` / `or $d, $c` etc. | 8 | Cross-ops between $c and $d through OR |
| `and $c, $d` / `and $d, $c` etc. | 8 | Cross-ops between $c and $d through AND |

Up to **20 more opcodes** reclaimable from the ALU register-register block if the compiler only targets $a as accumulator.

---

## 4. RECOMMENDED IMPLEMENTATION ORDER

| Priority | Change | Type | Opcodes Needed | Hardware | Impact |
|----------|--------|------|---------------|----------|--------|
| **1** | INC / DEC | Tier 2 | 2 (from `or $x,$x`) | 1 XOR gate (U24), 3 wires, 1 cut, 1 U76 bit | 36 bytes saved, compiler essential |
| **2** | SWAP $a,$b etc. | Tier 1 | 2–4 (from `or $x,$x` or ALU cross-ops) | Microcode only | Simplifies sorting, register shuffling |
| **3** | CLR $x | Tier 1 | 1 | Microcode only (SUB $x,$x → $x) | Convenience, compiler codegen |
| **4** | 16-step counter | Tier 3 | 0 | 5 wires (Q3→A15 on all EEPROMs) | Prerequisite for stack-relative |
| **5** | Stack-relative LD/ST | Tier 3 | 2–4 | Depends on step counter | Compiler-essential, enables stack frames |
| **6** | Additional flags/branches | Tier 3 | 2–4 | Chip swap + wiring | Signed comparisons, richer branching |

---

## 5. COMPLETE CONTROL SIGNAL MAP (for reference)

### U73 — Bits 31–24

| Bit | Signal | Function |
|-----|--------|----------|
| 31 | HLT | Halt clock |
| 30 | STK | Stack memory address space |
| 29 | PE | Program counter enable (increment) |
| 28 | RegIn[2] | Register input select (3-bit muxed with 27,26) |
| 27 | RegIn[1] | AI=001, BI=010, CI=011, DI=100, SI=101, EI=110, PI=111 |
| 26 | RegIn[0] | Decoded by 74HCT138 (U77) |
| 25 | MI | Memory address register in |
| 24 | RI | RAM data in |

### U74 — Bits 23–16

| Bit | Signal | Function |
|-----|--------|----------|
| 23 | II | Instruction register in |
| 22 | OI | Output register in |
| 21 | XI | External interface in |
| 20 | RegOut[2] | Register output select (3-bit muxed with 19,18) |
| 19 | RegOut[1] | AO=001, BO=010, CO=011, DO=100, PO=101, SO=110, EO=111 |
| 18 | RegOut[0] | Decoded by 74HCT238 (U78/U79) |
| 17 | RO | RAM data out |
| 16 | IO | Instruction register out (immediate data) |

### U75 — Bits 15–8

| Bit | Signal | Function |
|-----|--------|----------|
| 15 | SUB | ALU mode bit 2 |
| 14 | OR | ALU mode bit 1 |
| 13 | SHF | ALU mode bit 0 |
| 12 | FI | Flags register in (latch ALU flags) |
| 11 | SU | Stack pointer count UP |
| 10 | SD | Stack pointer count DOWN |
| 9 | U0 | External user signal 0 |
| 8 | U1 | External user signal 1 |

### U76 — Bits 7–0

| Bit | Signal | Function |
|-----|--------|----------|
| 7 | E0 | External enable 0 |
| 6 | E1 | External enable 1 |
| 5 | HL | High/Low address mode |
| 4 | RGT | Right shift/rotate direction |
| 3 | — | **AVAILABLE** |
| 2 | — | **AVAILABLE** |
| 1 | — | **AVAILABLE** |
| 0 | RST | Reset step counter |

---

## 6. KEY HARDWARE REFERENCE

### Microcode EEPROM Addressing

```
A18–A15:  unused (tied low)
A14:      IRQ1
A13:      IRQ0
A12:      Zero Flag
A11:      Carry Flag
A10–A3:   Instruction opcode (IR_7–IR_0)
A2–A0:    Step counter (0–7)
```

Total effective address: 15 bits → 32K locations per EEPROM.
4 EEPROMs × 8 bits each = 32-bit microcode word.

### Step Counter

- U72 (74HCT161): Q0–Q2 used (8 steps), Q3 unconnected
- RST (microcode bit 0) resets counter to 0
- Every instruction starts with 2 fetch steps: `PO|MI` then `PE|II|RO`

### ALU Carry-In Path

- SUB signal currently drives carry-in (C0) on the 74HCT283 adder(s) directly
- SUB also drives the XOR gates (U21/U22) for B-operand inversion
- INC/DEC modification intercepts only the C0 path, leaving B-inversion intact

### Register Select Decoders

- U77 (74HCT138): 3-to-8 decoder for register INPUT select (active-low outputs)
- U78/U79 (74HCT238): 3-to-8 decoders for register OUTPUT select
- Bits 28–26 (input) and 20–18 (output) encode register number 0–7
