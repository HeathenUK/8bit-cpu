# MK1 CPU — Bodge Wire Diagrams

Pin-level wiring diagrams for all hardware enhancements. Verify physical pin numbers against manufacturer datasheets before soldering.

---

## Enhancement 1: INC/DEC — ALU Carry-In Modification

```mermaid
graph LR
    subgraph U76["U76 (AM29F040 EEPROM) — PLCC-32"]
        U76_DQ1["DQ1 output<br/>(bit 1 — currently NoConn)"]
    end

    subgraph U24["U24 (74HCT86 Quad XOR) — DIP-14"]
        U24_9["Pin 9 (Gate 3 Input A)"]
        U24_10["Pin 10 (Gate 3 Input B)"]
        U24_8["Pin 8 (Gate 3 Output)"]
    end

    subgraph U15["U15 (74HCT283 Adder) — DIP-16"]
        U15_7["Pin 7 (C0 Carry-In)"]
    end

    SUB_NET["Existing SUB net<br/>(tap BEFORE trace cut)"]

    U76_DQ1 -- "Wire 1: CINV signal" --> U24_9
    SUB_NET -- "Wire 2: SUB signal" --> U24_10
    U24_8 -- "Wire 3: CINV XOR SUB" --> U15_7

    style U76 fill:#4a6,color:#fff
    style U24 fill:#46a,color:#fff
    style U15 fill:#a64,color:#fff
```

### Trace Cut
```mermaid
graph LR
    SUB_SOURCE["SUB signal source"] -. "CUT HERE" .-> U15_C0["U15 Pin 7 (C0)"]
    SUB_SOURCE -- "Keep intact" --> XOR_GATES["U21/U22 XOR gates<br/>(B-operand inversion)"]

    style SUB_SOURCE fill:#666,color:#fff
    style U15_C0 fill:#a33,color:#fff
    style XOR_GATES fill:#363,color:#fff
```

> **Critical:** The SUB net splits to (a) XOR gates U21/U22 for B-operand inversion and (b) U15 C0 carry-in. Cut ONLY the branch to C0. Verify the split point on your PCB before cutting.

---

## Enhancement 5: Display Mode Control

### U71 Flip-Flop (Mode Bit 0)

```mermaid
graph LR
    subgraph U76_2["U76 (AM29F040)"]
        DQ2["DQ2 output<br/>(bit 2 — currently NoConn)"]
    end

    subgraph U60["U60 (74HCT04 Inverter) — DIP-14<br/>Use any spare inverter"]
        U60_IN["Input (spare)"]
        U60_OUT["Output (spare)"]
    end

    subgraph U71["U71 (74HCT107 Dual JK FF) — DIP-14<br/>Flip-Flop 2 only"]
        U71_8["Pin 8 (2J)"]
        U71_11["Pin 11 (2K)"]
        U71_9["Pin 9 (2CLK)"]
        U71_10["Pin 10 (2~CLR)"]
        U71_6["Pin 6 (2Q) — OUTPUT"]
    end

    subgraph U36["U36 (28C64 Display EEPROM) — DIP-28"]
        U36_23["Pin 23 (A11)"]
    end

    BUS_0["BUS_0<br/>(data bus bit 0)"]
    RESET["System ~CLR net"]

    DQ2 -- "Wire 1: DM clock" --> U71_9
    BUS_0 -- "Wire 2" --> U71_8
    BUS_0 --> U60_IN
    U60_OUT -- "Wire 3: NOT BUS_0" --> U71_11
    RESET -- "Wire 4" --> U71_10
    U71_6 -- "Wire 5" --> U36_23

    style U76_2 fill:#4a6,color:#fff
    style U71 fill:#a64,color:#fff
    style U36 fill:#64a,color:#fff
    style U60 fill:#46a,color:#fff
```

### SR Latch (Mode Bit 1)

```mermaid
graph LR
    subgraph U58["U58 (74HCT08 AND) — DIP-14<br/>2 spare gates"]
        U58_G1_OUT["Gate X Output:<br/>DM AND BUS_1"]
        U58_G2_OUT["Gate Y Output:<br/>DM AND NOT(BUS_1)"]
    end

    subgraph U60b["U60 (74HCT04 Inverter)"]
        U60b_OUT["NOT(BUS_1)"]
    end

    subgraph U55["U55 (74HCT00 NAND) — DIP-14<br/>2 spare gates forming SR latch"]
        U55_S["S input (NAND gate 1)"]
        U55_R["R input (NAND gate 2)"]
        U55_Q["Q output"]
    end

    subgraph U36b["U36 (28C64)"]
        U36b_2["Pin 2 (A12)"]
    end

    DM_SIGNAL["DM signal<br/>(from U76 bit 2)"]
    BUS_1["BUS_1<br/>(data bus bit 1)"]

    DM_SIGNAL --> U58_G1_OUT
    BUS_1 --> U58_G1_OUT
    DM_SIGNAL --> U58_G2_OUT
    BUS_1 --> U60b_OUT
    U60b_OUT --> U58_G2_OUT
    U58_G1_OUT -- "Wire 6: Set" --> U55_S
    U58_G2_OUT -- "Wire 7: Reset" --> U55_R
    U55_Q -- "Wire 8" --> U36b_2

    style U58 fill:#46a,color:#fff
    style U55 fill:#a64,color:#fff
    style U36b fill:#64a,color:#fff
    style U60b fill:#46a,color:#fff
```

### Trace Cuts for Enhancement 5
```mermaid
graph TD
    CUT1["CUT 1: IRQ1 clock source → U71 Pin 9"]
    CUT2["CUT 2: IRQ1 J/K inputs → U71 Pins 8, 11"]
    CUT3["CUT 3: SW6 DIP switch → U36 Pin 23 (A11)"]
    CUT4["CUT 4: SW6 DIP switch → U36 Pin 2 (A12)"]

    style CUT1 fill:#a33,color:#fff
    style CUT2 fill:#a33,color:#fff
    style CUT3 fill:#a33,color:#fff
    style CUT4 fill:#a33,color:#fff
```

---

## Enhancement 6: OF/SF Flags — EEPROM Address Wiring

```mermaid
graph LR
    subgraph U11["U11 (74HCT173 Flag Register) — DIP-16"]
        U11_5["Pin 5 (Q2 = OF)<br/>Currently NoConn"]
        U11_6["Pin 6 (Q3 = SF)<br/>Currently NoConn"]
    end

    subgraph EEPROMS["Microcode EEPROMs (AM29F040) — PLCC-32 × 4"]
        subgraph U73["U73"]
            U73_A16["A16 — lift GND first"]
            U73_A17["A17 — lift GND first"]
        end
        subgraph U74["U74"]
            U74_A16["A16 — lift GND first"]
            U74_A17["A17 — lift GND first"]
        end
        subgraph U75["U75"]
            U75_A16["A16 — lift GND first"]
            U75_A17["A17 — lift GND first"]
        end
        subgraph U76e["U76"]
            U76_A16["A16 — lift GND first"]
            U76_A17["A17 — lift GND first"]
        end
    end

    U11_5 -- "Wire 1" --> U73_A16
    U11_5 -- "Wire 2" --> U74_A16
    U11_5 -- "Wire 3" --> U75_A16
    U11_5 -- "Wire 4" --> U76_A16

    U11_6 -- "Wire 5" --> U73_A17
    U11_6 -- "Wire 6" --> U74_A17
    U11_6 -- "Wire 7" --> U75_A17
    U11_6 -- "Wire 8" --> U76_A17

    style U11 fill:#a64,color:#fff
    style U73 fill:#4a6,color:#fff
    style U74 fill:#4a6,color:#fff
    style U75 fill:#4a6,color:#fff
    style U76e fill:#4a6,color:#fff
```

### GND Lifts for Enhancement 6
```mermaid
graph TD
    subgraph LIFTS["Lift/cut GND traces BEFORE wiring"]
        L1["U73 A16 — lift from GND"]
        L2["U73 A17 — lift from GND"]
        L3["U74 A16 — lift from GND"]
        L4["U74 A17 — lift from GND"]
        L5["U75 A16 — lift from GND"]
        L6["U75 A17 — lift from GND"]
        L7["U76 A16 — lift from GND"]
        L8["U76 A17 — lift from GND"]
    end

    style L1 fill:#a33,color:#fff
    style L2 fill:#a33,color:#fff
    style L3 fill:#a33,color:#fff
    style L4 fill:#a33,color:#fff
    style L5 fill:#a33,color:#fff
    style L6 fill:#a33,color:#fff
    style L7 fill:#a33,color:#fff
    style L8 fill:#a33,color:#fff
```

> **PLCC-32 pin numbers for A16 and A17:** Consult the AM29F040B datasheet for your specific chip revision. These pins are typically on the top edge of the PLCC-32 package near pin 1. All four EEPROMs (U73–U76) use the same pinout.

> **Pre-check:** Probe U11 pins 5 and 6 with a scope during ALU operations to confirm they carry distinct OF and SF signals before committing to any GND lifts.

---

## All Hardware Changes — Summary

```mermaid
graph TD
    subgraph HW_ENH1["Enhancement 1: INC/DEC<br/>3 wires, 1 trace cut"]
        E1W1["U76 DQ1 → U24 Pin 9"]
        E1W2["SUB net → U24 Pin 10"]
        E1W3["U24 Pin 8 → U15 Pin 7"]
        E1C1["CUT: SUB → U15 Pin 7"]
    end

    subgraph HW_ENH5["Enhancement 5: Display Mode<br/>~8 wires, ~3 trace cuts"]
        E5W1["U76 DQ2 → U71 Pin 9"]
        E5W2["BUS_0 → U71 Pin 8"]
        E5W3["NOT BUS_0 → U71 Pin 11"]
        E5W4["~CLR → U71 Pin 10"]
        E5W5["U71 Pin 6 → U36 Pin 23"]
        E5W6["DM+BUS_1 → U55 SR Set"]
        E5W7["DM+NOT BUS_1 → U55 SR Reset"]
        E5W8["U55 Q → U36 Pin 2"]
        E5C1["CUT: IRQ1 → U71"]
        E5C2["CUT: SW6 → U36 A11"]
        E5C3["CUT: SW6 → U36 A12"]
    end

    subgraph HW_ENH6["Enhancement 6: OF/SF Flags<br/>8 wires, 8 GND lifts"]
        E6W1["U11 Pin 5 → U73–U76 A16 (×4)"]
        E6W2["U11 Pin 6 → U73–U76 A17 (×4)"]
        E6L1["Lift GND: A16 on U73–U76 (×4)"]
        E6L2["Lift GND: A17 on U73–U76 (×4)"]
    end

    subgraph NO_HW["Enhancements 2,3,4,7,8,9,10,11,12<br/>MICROCODE ONLY — no hardware"]
        NONE["Page 3 · CLR · SWAP · Stack LD<br/>XOR · NEG · Cond Set · Nswap · ABS"]
    end

    style HW_ENH1 fill:#264,color:#fff
    style HW_ENH5 fill:#246,color:#fff
    style HW_ENH6 fill:#642,color:#fff
    style NO_HW fill:#333,color:#aaa
    style E1C1 fill:#a33,color:#fff
    style E5C1 fill:#a33,color:#fff
    style E5C2 fill:#a33,color:#fff
    style E5C3 fill:#a33,color:#fff
```

> **Total physical work:** ~19 bodge wires + 4 trace cuts + 8 GND lifts. Nine of the twelve enhancements require zero hardware changes.
