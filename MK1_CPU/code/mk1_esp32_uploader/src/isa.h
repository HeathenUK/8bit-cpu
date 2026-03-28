// MK1 Instruction Set Architecture definition
//
// This file is the SINGLE SOURCE OF TRUTH for the MK1 instruction set
// on the ESP32 assembler. When the microcode changes, update this file.
//
// Instruction encoding reference:
//   MOV:   00 | src[2:0] | dst[2:0]       (1 byte, or 2 if src/dst = imm)
//   LOAD:  01 | dst[2:0] | addr[2:0]      (1 byte, or 2 if addr = imm)
//   STORE: 10 | src[2:0] | addr[2:0]      (1 byte, or 2 if addr = imm)
//   ALU:   11 | op[1:0]  | rs[1:0] | rd[1:0]  (1 byte)
//   ALUi:  1011 | op[1:0] | rd[1:0] | imm     (2 bytes)

#pragma once
#include <Arduino.h>

// ── Register encoding ────────────────────────────────────────────────

struct RegDef {
    const char* name;
    uint8_t code;  // 3-bit register code
};

static const RegDef REGISTERS[] = {
    { "$a",   0 },
    { "$b",   1 },
    { "$c",   2 },
    { "$d",   3 },
    { "$sp",  4 },
    { "$pc",  5 },
    { "$out", 6 },
    { nullptr, 0 }
};

// ── ALU operation encoding ───────────────────────────────────────────

struct AluOpDef {
    const char* name;
    uint8_t code;  // 2-bit ALU op code
};

static const AluOpDef ALU_OPS[] = {
    { "add", 0 },
    { "sub", 1 },
    { "or",  2 },
    { "and", 3 },
    { nullptr, 0 }
};

// ── Fixed-opcode instructions ────────────────────────────────────────
//
// Instructions with fixed binary encodings. These don't follow the
// regular MOV/LOAD/STORE/ALU patterns and must be listed explicitly.
//
// When the microcode adds new instructions, add them here.

enum InstrArgs {
    ARGS_NONE,   // no arguments
    ARGS_IMM,    // one immediate byte (label or number)
    ARGS_REG,    // one register
};

struct FixedInstr {
    const char* mnemonic;
    uint8_t opcode;
    InstrArgs args;
};

static const FixedInstr FIXED_INSTRUCTIONS[] = {
    // ── Core ──
    { "nop",     0x00,  ARGS_NONE },
    { "hlt",     0x7F,  ARGS_NONE },
    { "ret",     0x6C,  ARGS_NONE },  // load $pc, [$sp] = 01_101_100
    { "out",     0x06,  ARGS_NONE },  // mov $a, $out = 00_000_110

    // ── ALU single-operand ──
    { "not",     0x7A,  ARGS_NONE },
    { "sll",     0x7B,  ARGS_NONE },
    { "slr",     0x7C,  ARGS_NONE },
    { "rll",     0x7D,  ARGS_NONE },
    { "rlr",     0x7E,  ARGS_NONE },

    // ── Jumps ──
    { "j",       0x3D,  ARGS_IMM },   // mov imm, $pc = 00_111_101
    { "jc",      0x37,  ARGS_IMM },   // jcf
    { "jz",      0x3F,  ARGS_IMM },   // jzf
    { "jal",     0xAC,  ARGS_IMM },   // stor $pc, [$sp] + imm = 10_101_100
    { "je0",     0x27,  ARGS_IMM },
    { "je1",     0x2F,  ARGS_IMM },

    // ── Phase 1: new instructions ──
    { "xor",     0xE0,  ARGS_NONE },  // A = A XOR B (clobbers D)
    { "neg",     0xE5,  ARGS_NONE },  // A = -A (two's complement)
    { "ldp3",    0xEA,  ARGS_IMM  },  // A = page3[imm]
    { "stp3",    0xEF,  ARGS_IMM  },  // page3[imm] = A
    { "ldsp",    0xEB,  ARGS_IMM  },  // A = stack[SP + imm]
    { "swap",    0xEE,  ARGS_NONE },  // swap A and B
    { "inc",     0xFB,  ARGS_NONE },  // A = A + 1
    { "dec",     0xFE,  ARGS_NONE },  // A = A - 1
    { "jnc",     0xCA,  ARGS_IMM  },  // jump if not carry
    { "jnz",     0xCE,  ARGS_IMM  },  // jump if not zero
    { "setjmp",  0xCB,  ARGS_IMM  },  // cross-page jump (future)
    { "setret",  0xCF,  ARGS_NONE },  // cross-page return (future)

    // ── Aliases for existing instructions (CLR = self-subtract) ──
    { "clr $a",  0xD0,  ARGS_NONE },  // A = 0
    { "clr $b",  0xD5,  ARGS_NONE },  // B = 0
    { "clr $c",  0xDA,  ARGS_NONE },  // C = 0
    { "clr $d",  0xDF,  ARGS_NONE },  // D = 0

    // ── External interface ──
    { "exr 0",   0x78,  ARGS_NONE },
    { "exr 1",   0x79,  ARGS_NONE },
    { "exw 0 0", 0x07,  ARGS_NONE },
    { "exw 0 1", 0x0F,  ARGS_NONE },
    { "exw 0 2", 0x8E,  ARGS_NONE },
    { "exw 0 3", 0x96,  ARGS_NONE },
    { "exw 1 0", 0x17,  ARGS_NONE },
    { "exw 1 1", 0x1F,  ARGS_NONE },
    { "exw 1 2", 0x9E,  ARGS_NONE },
    { "exw 1 3", 0xA6,  ARGS_NONE },

    // ── Sentinel ──
    { nullptr, 0, ARGS_NONE }
};

// ── Compare variants ─────────────────────────────────────────────────
// cmp has register and immediate forms that don't fit the regular
// ALU pattern, so they're listed separately.

struct CmpDef {
    const char* operand;  // "$b", "$c", "$d", or nullptr for immediate
    uint8_t opcode;
    bool has_imm;
};

static const CmpDef CMP_VARIANTS[] = {
    { "$b",    0xAE, false },
    { "$c",    0x46, false },
    { "$d",    0x4E, false },
    { nullptr, 0x86, true  },  // cmp <imm>
};
