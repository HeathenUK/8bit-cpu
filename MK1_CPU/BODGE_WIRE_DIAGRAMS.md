# MK1 CPU — Bodge Wire Diagrams

Physical pin locations for all hardware enhancements. Pin numbers from standard manufacturer datasheets.

**Verify against YOUR specific chip markings before soldering.** PLCC-32 pins count counterclockwise from the notch.

---

## Enhancement 1: INC/DEC — Components Involved

### U24 — 74HCT86 (Quad 2-Input XOR) — DIP-14

Gate 3 is spare. **Inputs: pins 9, 10. Output: pin 8.**

```
                 ┌────U────┐
       1A  [1]  ─┤         ├─  [14] VCC
       1B  [2]  ─┤         ├─  [13] 4B
       1Y  [3]  ─┤         ├─  [12] 4A
       2A  [4]  ─┤  74HC   ├─  [11] 4Y
       2B  [5]  ─┤  T86    ├─  [10] 3B ◄── WIRE 1: CINV from U76 DQ1
       2Y  [6]  ─┤         ├─  [ 9] 3A ◄── WIRE 2: SUB signal (tap before cut)
      GND  [7]  ─┤         ├─  [ 8] 3Y ◄── WIRE 3: Output → U15 pin 7
                 └─────────┘
```

### U15 — 74HCT283 (4-Bit Adder, Low Nibble) — DIP-16

**Pin 7 = C0 (Carry-In).** Cut existing SUB trace to this pin. Reconnect via U24 gate 3 output.

```
                 ┌────U────┐
       Σ2  [1]  ─┤         ├─  [16] VCC
       B2  [2]  ─┤         ├─  [15] B3
       A2  [3]  ─┤         ├─  [14] A3
       Σ1  [4]  ─┤  74HC   ├─  [13] Σ3
       A1  [5]  ─┤  T283   ├─  [12] Σ4
       B1  [6]  ─┤         ├─  [11] C4 (carry out)
   ►►  C0  [7]  ─┤         ├─  [10] A4
      GND  [8]  ─┤         ├─  [ 9] B4
                 └─────────┘

       ►► = CUT existing SUB → pin 7 trace
            WIRE 3 from U24 pin 8 connects here
```

### U76 — AM29F040 (Microcode EEPROM, Byte 0) — PLCC-32

**DQ1 (pin 14) = control word bit 1 = CINV signal (currently NoConn).**

```
              ┌─── notch ───┐
              │  9  8  7  6 │ 5
           10─┤             ├─4  A12
        A2 11─┤             ├─3  A15 (spare)
        A1 12─┤             ├─2  A16 ◄── Enhancement 6 (OF)
        A0 13─┤   AM29F040 ├─1  A18 (spare)
   ►►  DQ0 14─┤   (U76)    ├─32 VCC
       DQ1 15─┤             ├─31 ~WE
      GND  16─┤             ├─30 A17 ◄── Enhancement 6 (SF)
       DQ2 17─┤             ├─29 A14
              │ 18 19 20 21 │ 22 ~CE
              └─────────────┘
                         23 A10

       ►► DQ1 (pin 15) = CINV signal → Wire 1 to U24 pin 10
          DQ2 (pin 17) = DM signal  → Enhancement 5, Wire 1 to U71 pin 9
```

**IMPORTANT:** The DQ-to-bit mapping (DQ0=bit 0, DQ1=bit 1, etc.) assumes standard wiring. Verify on your PCB by probing the existing RST signal (bit 0) — it should appear on DQ0 (pin 14). If the bit ordering is swapped, trace the actual DQ pin for bits 1 and 2.

### Wiring Summary — Enhancement 1

```
  U76 pin 15 (DQ1) ────────────────────── U24 pin 10 (3B)   WIRE 1: CINV
  SUB net (tap before cut) ────────────── U24 pin  9 (3A)   WIRE 2: SUB
  U24 pin 8 (3Y) ──────────────────────── U15 pin  7 (C0)   WIRE 3: modified carry-in

  TRACE CUT: existing SUB → U15 pin 7 (C0 direct connection)
  DO NOT CUT: SUB → U21/U22 XOR gates (B-operand inversion path)
```

---

## Enhancement 5: Display Mode Control — Components Involved

### U71 — 74HCT107 (Dual JK Flip-Flop, Neg-Edge) — DIP-14

**FF2 is repurposed. FF1 (IRQ0) is untouched.**

```
                 ┌────U────┐
    ░░  1J  [1]  ─┤         ├─  [14] VCC        ░░ = FF1, do NOT
    ░░ 1~Q  [2]  ─┤         ├─  [13] 1~CLR  ░░      disturb
    ░░  1Q  [3]  ─┤         ├─  [12] 1K     ░░
    ░░ 1CLK [4]  ─┤  74HC   ├─  [11] 2K  ◄── WIRE 3: NOT(BUS_0) via U60
       2~Q  [5]  ─┤  T107   ├─  [10] 2~CLR◄── WIRE 4: system ~CLR
   ►►  2Q   [6]  ─┤         ├─  [ 9] 2CLK ◄── WIRE 1: DM from U76 DQ2
      GND   [7]  ─┤         ├─  [ 8] 2J   ◄── WIRE 2: BUS_0
                 └─────────┘

       ►► pin 6 (2Q) = mode bit 0 output → Wire 5 to U36 pin 23 (A11)

  TRACE CUTS: existing IRQ1 logic → pins 8, 9, 10, 11
              (disconnect FF2 from interrupt circuitry)
```

### U36 — 28C64 (Display Decode EEPROM) — DIP-28

**A11 (pin 23) and A12 (pin 2) receive the mode bits. Cut SW6 traces to both.**

```
                 ┌────U────┐
       NC   [1]  ─┤         ├─  [28] VCC
   ►►  A12  [2]  ─┤         ├─  [27] ~WE
       A7   [3]  ─┤         ├─  [26] NC
       A6   [4]  ─┤         ├─  [25] A8
       A5   [5]  ─┤         ├─  [24] A9
       A4   [6]  ─┤  28C64  ├─  [23] A11 ◄── WIRE 5: U71 pin 6 (2Q)
       A3   [7]  ─┤  (U36)  ├─  [22] ~OE
       A2   [8]  ─┤         ├─  [21] A10
       A1   [9]  ─┤         ├─  [20] ~CE
       A0  [10]  ─┤         ├─  [19] I/O7
      O0   [11]  ─┤         ├─  [18] I/O6
      O1   [12]  ─┤         ├─  [17] I/O5
      O2   [13]  ─┤         ├─  [16] I/O4
      GND  [14]  ─┤         ├─  [15] I/O3
                 └─────────┘

       ►► pin 2  (A12) = mode bit 1 ← Wire 8 from SR latch Q output
          pin 23 (A11) = mode bit 0 ← Wire 5 from U71 pin 6

  TRACE CUTS: SW6 → pin 2  (disconnect DIP switch from A12)
              SW6 → pin 23 (disconnect DIP switch from A11)
```

### SR Latch for Mode Bit 1 — Built from U55/U58/U60

```
  BUS_1 ──────┬────────────── U58 gate X input A ─┐
              │                                     ├── U58 gate X out ── U55 NAND latch S
  DM signal ──┼────────────── U58 gate X input B ─┘
              │
              └── U60 inv ── U58 gate Y input A ─┐
                                                   ├── U58 gate Y out ── U55 NAND latch R
  DM signal ──────────────── U58 gate Y input B ─┘

  U55 NAND cross-coupled latch Q output ──────────── U36 pin 2 (A12)
```

**U58 (74HCT08 Quad AND) and U55 (74HCT00 Quad NAND) share the same DIP-14 pin pattern:**

```
                 ┌────U────┐
       1A  [1]  ─┤         ├─  [14] VCC
       1B  [2]  ─┤  74HC   ├─  [13] 4B
       1Y  [3]  ─┤  T08    ├─  [12] 4A
       2A  [4]  ─┤  or     ├─  [11] 4Y
       2B  [5]  ─┤  T00    ├─  [10] 3B
       2Y  [6]  ─┤         ├─  [ 9] 3A
      GND  [7]  ─┤         ├─  [ 8] 3Y
                 └─────────┘

  Which specific gates are spare depends on your board.
  Probe each gate's pins to confirm they're unconnected before using.
```

**U60 (74HCT04 Hex Inverter) — DIP-14:**

```
                 ┌────U────┐
       1A  [1]  ─┤         ├─  [14] VCC
       1Y  [2]  ─┤         ├─  [13] 6A
       2A  [3]  ─┤  74HC   ├─  [12] 6Y
       2Y  [4]  ─┤  T04    ├─  [11] 5A
       3A  [5]  ─┤         ├─  [10] 5Y
       3Y  [6]  ─┤         ├─  [ 9] 4A
      GND  [7]  ─┤         ├─  [ 8] 4Y
                 └─────────┘

  Need 2 spare inverters: one for NOT(BUS_0), one for NOT(BUS_1).
  Probe to find which are unconnected.
```

---

## Enhancement 6: OF/SF Flags — Components Involved

### U11 — 74HCT173 (4-Bit D Register, Flags) — DIP-16

**Pins 5 and 6 are the flag outputs we need. Currently NoConn.**

```
                 ┌────U────┐
      OE1  [1]  ─┤         ├─  [16] VCC
      OE2  [2]  ─┤         ├─  [15] CLR
  ░░   Q0  [3]  ─┤         ├─  [14] D0        ░░ = CF, ZF already
  ░░   Q1  [4]  ─┤  74HC   ├─  [13] D1   ░░       wired to EEPROMs
   ►►  Q2  [5]  ─┤  T173   ├─  [12] D2
   ►►  Q3  [6]  ─┤  (U11)  ├─  [11] D3
      CLK  [7]  ─┤         ├─  [10] E2
      GND  [8]  ─┤         ├─  [ 9] E1
                 └─────────┘

       ►► pin 5 (Q2) = Overflow Flag → Wires 1–4 to U73–U76 pin 2 (A16)
       ►► pin 6 (Q3) = Sign Flag    → Wires 5–8 to U73–U76 pin 30 (A17)

  PRE-CHECK: Probe pins 5 and 6 during ALU operations before committing.
```

### U73, U74, U75, U76 — AM29F040 (PLCC-32) — All 4 EEPROMs

**A16 = PLCC pin 2, A17 = PLCC pin 30. Lift GND from both on all 4 chips.**

```
        pin 1
          ↓
              ┌─── notch ───┐
              │  9  8  7  6 │ 5
           10─┤             ├─4  A12
        A2 11─┤             ├─3  A15 (spare)
        A1 12─┤             ├─2  A16 ◄◄ LIFT GND, wire to U11 pin 5 (OF)
        A0 13─┤   AM29F040 ├─1  A18 (spare)
       DQ0 14─┤             ├─32 VCC
       DQ1 15─┤             ├─31 ~WE
      GND  16─┤             ├─30 A17 ◄◄ LIFT GND, wire to U11 pin 6 (SF)
       DQ2 17─┤             ├─29 A14
              │ 18 19 20 21 │ 22 ~CE
              └─────────────┘
                   ↑       ↑
                DQ4-DQ7   23 A10

  ◄◄ = Lift GND trace FIRST, then solder bodge wire.

  Repeat on all 4 EEPROMs: U73, U74, U75, U76.
  Total: 8 GND lifts + 8 bodge wires.

  Pin 2  (A16) and pin 30 (A17) are on ADJACENT EDGES
  of the PLCC package — A16 is top edge, A17 is left edge,
  both near pin 1 (top-left corner).
```

---

## Complete Wiring Checklist

### Enhancement 1: INC/DEC (3 wires, 1 cut)

```
  [ ] WIRE 1: U76 pin 15 (DQ1) ───── U24 pin 10 (gate 3 input B)
  [ ] WIRE 2: SUB net (tap) ───────── U24 pin  9 (gate 3 input A)
  [ ] WIRE 3: U24 pin  8 (gate 3 Y)── U15 pin  7 (C0)
  [ ] CUT:    SUB trace → U15 pin 7   (keep SUB → U21/U22 intact)
```

### Enhancement 5: Display Mode (~11 wires, ~3 cuts)

```
  FF2 repurpose:
  [ ] CUT:    IRQ1 clock → U71 pin 9
  [ ] CUT:    IRQ1 J/K → U71 pins 8, 11
  [ ] WIRE 1: U76 pin 17 (DQ2) ───── U71 pin  9 (2CLK)
  [ ] WIRE 2: BUS_0 ──────────────── U71 pin  8 (2J)
  [ ] WIRE 3: U60 spare inv out ──── U71 pin 11 (2K)
              (U60 spare inv in ← BUS_0)
  [ ] WIRE 4: System ~CLR ────────── U71 pin 10 (2~CLR)
  [ ] WIRE 5: U71 pin 6 (2Q) ─────── U36 pin 23 (A11)

  SR latch:
  [ ] WIRE 6: U58 spare AND out ──── U55 NAND latch S input
              (U58 inputs ← DM signal + BUS_1)
  [ ] WIRE 7: U58 spare AND out ──── U55 NAND latch R input
              (U58 inputs ← DM signal + NOT(BUS_1) via U60)
  [ ] WIRE 8: U55 latch Q out ────── U36 pin  2 (A12)

  Display EEPROM:
  [ ] CUT:    SW6 → U36 pin 23 (A11)
  [ ] CUT:    SW6 → U36 pin  2 (A12)
```

### Enhancement 6: OF/SF Flags (8 wires, 8 GND lifts)

```
  Overflow Flag (OF):
  [ ] LIFT:   U73 pin  2 (A16) from GND
  [ ] LIFT:   U74 pin  2 (A16) from GND
  [ ] LIFT:   U75 pin  2 (A16) from GND
  [ ] LIFT:   U76 pin  2 (A16) from GND
  [ ] WIRE 1: U11 pin 5 (Q2) ─── U73 pin  2 (A16)
  [ ] WIRE 2: U11 pin 5 (Q2) ─── U74 pin  2 (A16)
  [ ] WIRE 3: U11 pin 5 (Q2) ─── U75 pin  2 (A16)
  [ ] WIRE 4: U11 pin 5 (Q2) ─── U76 pin  2 (A16)

  Sign Flag (SF):
  [ ] LIFT:   U73 pin 30 (A17) from GND
  [ ] LIFT:   U74 pin 30 (A17) from GND
  [ ] LIFT:   U75 pin 30 (A17) from GND
  [ ] LIFT:   U76 pin 30 (A17) from GND
  [ ] WIRE 5: U11 pin 6 (Q3) ─── U73 pin 30 (A17)
  [ ] WIRE 6: U11 pin 6 (Q3) ─── U74 pin 30 (A17)
  [ ] WIRE 7: U11 pin 6 (Q3) ─── U75 pin 30 (A17)
  [ ] WIRE 8: U11 pin 6 (Q3) ─── U76 pin 30 (A17)

  PRE-CHECK:
  [ ] Probe U11 pin 5 during ALU ops — confirm OF signal
  [ ] Probe U11 pin 6 during ALU ops — confirm SF signal
```

---

## PLCC-32 Pin Location Guide

For the 4 microcode EEPROMs (U73–U76). View from above, notch at top-left.

```
                        notch
                        ↓
            ┌───────────█───────────┐
            │ 1   2   3   4   5    │
            │A18 A16 A15 A12  A7   │
         32─┤                      ├─6  A6
     VCC    │                      │    A5
         31─┤                      ├─7
     ~WE    │                      │    A4
     ►►A17──┤                      ├─8
         30 │      AM29F040        │
         29─┤      (top view)      ├─9  A3
     A14    │                      │    A2
         28─┤                      ├─10
     A13    │                      │    A1
         27─┤                      ├─11
      A8    │                      │    A0
         26─┤                      ├─12
      A9    │                      │
            │ 25  24  23  22  21   │
            │A11  ~OE A10 ~CE DQ7  │
            └──────────────────────┘
                  17  18  19  20
                 DQ3  DQ4 DQ5 DQ6

         13=DQ0  14=DQ1  15=DQ2  16=GND

  ►► A16 (pin 2)  = OF target — top edge, 2nd from left
  ►► A17 (pin 30) = SF target — left edge, 3rd from top
  Both near the pin 1 corner. Adjacent edges.
```
