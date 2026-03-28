// MK1 Assembler — runs on ESP32
//
// Two-pass assembler: pass 1 collects labels, pass 2 emits bytes.
// Supports the customasm-style syntax used by the MK1 programs:
//   - Labels: "name:" or ".name:" (local labels, scoped to last global label)
//   - Instructions with register and immediate operands
//   - Constants: "NAME = value"
//   - Data bank: #bank ".data" / #bank ".instr"
//   - Data directives: #d8, #res, #str (in data bank)
//   - Comments: ; to end of line
//   - #include directives are ignored (libraries are built-in)

#pragma once
#include <Arduino.h>
#include "isa.h"

static const int CODE_SIZE = 256;
static const int DATA_SIZE = 256;
static const int MAX_LABELS = 128;
static const int MAX_CONSTANTS = 32;
static const int MAX_LINES = 512;
static const int MAX_ERRORS = 16;

struct Label {
    char name[48];
    uint16_t address;
    bool is_data;  // true if in data bank
};

struct Constant {
    char name[32];
    int value;
};

struct AsmError {
    int line;
    char message[80];
};

struct AsmResult {
    uint8_t code[CODE_SIZE];
    uint8_t data[DATA_SIZE];
    int code_size;
    int data_size;
    AsmError errors[MAX_ERRORS];
    int error_count;
};

// ── Helper functions ─────────────────────────────────────────────────

static bool isWhitespace(char c) { return c == ' ' || c == '\t'; }

static void skipWs(const char*& p) {
    while (*p && isWhitespace(*p)) p++;
}

static bool startsWith(const char* str, const char* prefix) {
    while (*prefix) {
        if (tolower(*str) != tolower(*prefix)) return false;
        str++; prefix++;
    }
    return true;
}

static bool isIdentChar(char c) {
    return isalnum(c) || c == '_' || c == '$' || c == '.';
}

static int parseToken(const char* p, char* out, int maxLen) {
    int i = 0;
    while (*p && isIdentChar(*p) && i < maxLen - 1) {
        out[i++] = *p++;
    }
    out[i] = 0;
    return i;
}

static int findRegister(const char* name) {
    for (int i = 0; REGISTERS[i].name; i++) {
        if (strcasecmp(name, REGISTERS[i].name) == 0) return REGISTERS[i].code;
    }
    return -1;
}

static int findAluOp(const char* name) {
    for (int i = 0; ALU_OPS[i].name; i++) {
        if (strcasecmp(name, ALU_OPS[i].name) == 0) return ALU_OPS[i].code;
    }
    return -1;
}

static int parseNumber(const char* p, bool& ok) {
    ok = false;
    skipWs(p);
    if (!*p) return 0;

    bool negative = false;
    if (*p == '-') { negative = true; p++; }

    int base = 10;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) { base = 16; p += 2; }
    else if (p[0] == '0' && (p[1] == 'b' || p[1] == 'B')) { base = 2; p += 2; }

    if (!*p || (!isdigit(*p) && !isxdigit(*p))) return 0;

    char* end;
    long val = strtol(p, &end, base);
    if (end != p) {
        ok = true;
        return negative ? -(int)val : (int)val;
    }
    return 0;
}

// ── Main assembler class ─────────────────────────────────────────────

class MK1Assembler {
public:
    AsmResult result;

    void assemble(const char* source) {
        memset(&result, 0, sizeof(result));
        labelCount = 0;
        constCount = 0;
        lastGlobalLabel[0] = 0;

        // Pass 1: collect labels and constants
        pass(source, true);
        if (result.error_count > 0) return;

        // Pass 2: emit code
        lastGlobalLabel[0] = 0;
        pass(source, false);
    }

private:
    Label labels[MAX_LABELS];
    int labelCount;
    Constant constants[MAX_CONSTANTS];
    int constCount;
    char lastGlobalLabel[48];

    void addError(int line, const char* msg) {
        if (result.error_count < MAX_ERRORS) {
            result.errors[result.error_count].line = line;
            strncpy(result.errors[result.error_count].message, msg, 79);
            result.errors[result.error_count].message[79] = 0;
            result.error_count++;
        }
    }

    void addLabel(const char* name, uint16_t addr, bool is_data) {
        if (labelCount >= MAX_LABELS) return;
        // Expand local labels: ".foo" becomes "lastGlobal.foo"
        if (name[0] == '.') {
            snprintf(labels[labelCount].name, sizeof(labels[0].name),
                     "%s%s", lastGlobalLabel, name);
        } else {
            strncpy(labels[labelCount].name, name, sizeof(labels[0].name) - 1);
        }
        labels[labelCount].address = addr;
        labels[labelCount].is_data = is_data;
        labelCount++;
    }

    int findLabel(const char* name) {
        char expanded[48];
        if (name[0] == '.') {
            snprintf(expanded, sizeof(expanded), "%s%s", lastGlobalLabel, name);
        } else {
            strncpy(expanded, name, sizeof(expanded) - 1);
            expanded[sizeof(expanded) - 1] = 0;
        }
        for (int i = 0; i < labelCount; i++) {
            if (strcmp(labels[i].name, expanded) == 0) return labels[i].address;
        }
        return -1;
    }

    void addConstant(const char* name, int value) {
        if (constCount >= MAX_CONSTANTS) return;
        strncpy(constants[constCount].name, name, sizeof(constants[0].name) - 1);
        constants[constCount].value = value;
        constCount++;
    }

    int findConstant(const char* name) {
        for (int i = 0; i < constCount; i++) {
            if (strcmp(constants[i].name, name) == 0) return constants[i].value;
        }
        return -1;
    }

    // Resolve an operand that could be a number, label, or constant
    int resolveValue(const char* tok, int line, bool pass1) {
        bool ok;
        int v = parseNumber(tok, ok);
        if (ok) return v;

        // Try constant
        int c = findConstant(tok);
        if (c >= 0) return c;

        // Try label
        int l = findLabel(tok);
        if (l >= 0) return l;

        if (!pass1) {
            char msg[80];
            snprintf(msg, sizeof(msg), "Undefined symbol: %.40s", tok);
            addError(line, msg);
        }
        return 0;
    }

    void emitCode(uint8_t byte, bool pass1) {
        if (!pass1 && result.code_size < CODE_SIZE)
            result.code[result.code_size] = byte;
        result.code_size++;
    }

    void emitData(uint8_t byte, bool pass1) {
        if (!pass1 && result.data_size < DATA_SIZE)
            result.data[result.data_size] = byte;
        result.data_size++;
    }

    void pass(const char* source, bool pass1) {
        if (!pass1) {
            result.code_size = 0;
            result.data_size = 0;
        }

        bool in_data_bank = false;
        int lineNum = 0;

        const char* p = source;
        while (*p) {
            lineNum++;

            // Extract line
            const char* lineStart = p;
            while (*p && *p != '\n') p++;
            int lineLen = p - lineStart;
            if (*p == '\n') p++;

            char line[256];
            int len = lineLen < 255 ? lineLen : 255;
            memcpy(line, lineStart, len);
            line[len] = 0;

            // Strip comment
            char* semi = strchr(line, ';');
            if (semi) *semi = 0;

            // Trim trailing whitespace
            len = strlen(line);
            while (len > 0 && isWhitespace(line[len - 1])) line[--len] = 0;

            const char* lp = line;
            skipWs(lp);
            if (!*lp) continue;  // empty line

            // Handle directives
            if (*lp == '#') {
                lp++;
                if (startsWith(lp, "include")) continue;  // skip includes
                if (startsWith(lp, "bank")) {
                    in_data_bank = strstr(lp, ".data") != nullptr;
                    continue;
                }
                if (startsWith(lp, "d8") || startsWith(lp, "res") || startsWith(lp, "str")) {
                    // Data directives
                    if (startsWith(lp, "d8")) {
                        lp += 2; skipWs(lp);
                        char tok[32];
                        parseToken(lp, tok, sizeof(tok));
                        int v = resolveValue(tok, lineNum, pass1);
                        emitData(v & 0xFF, pass1);
                    } else if (startsWith(lp, "res")) {
                        lp += 3; skipWs(lp);
                        bool ok; int count = parseNumber(lp, ok);
                        if (ok) for (int i = 0; i < count; i++) emitData(0, pass1);
                    } else if (startsWith(lp, "str")) {
                        lp += 3; skipWs(lp);
                        if (*lp == '"') {
                            lp++;
                            while (*lp && *lp != '"') {
                                if (*lp == '\\' && lp[1] == '0') {
                                    emitData(0, pass1); lp += 2;
                                } else {
                                    emitData(*lp, pass1); lp++;
                                }
                            }
                        }
                    }
                    continue;
                }
                continue;  // unknown directive
            }

            // Check for constant definition: NAME = value
            {
                const char* eq = strchr(lp, '=');
                if (eq) {
                    // Check it's not inside a label definition
                    const char* colon = strchr(lp, ':');
                    if (!colon || eq < colon) {
                        char name[32];
                        const char* np = lp;
                        int ni = 0;
                        while (np < eq && ni < 31) {
                            if (!isWhitespace(*np)) name[ni++] = *np;
                            np++;
                        }
                        name[ni] = 0;
                        const char* vp = eq + 1;
                        skipWs(vp);
                        bool ok;
                        int val = parseNumber(vp, ok);
                        if (ok && pass1) addConstant(name, val);
                        continue;
                    }
                }
            }

            // Check for label: "name:" at start of line
            {
                const char* colon = strchr(lp, ':');
                if (colon) {
                    char labelName[48];
                    int li = 0;
                    const char* lp2 = lp;
                    while (lp2 < colon && li < 47) {
                        if (!isWhitespace(*lp2)) labelName[li++] = *lp2;
                        lp2++;
                    }
                    labelName[li] = 0;

                    uint16_t addr = in_data_bank ? result.data_size : result.code_size;
                    if (pass1) addLabel(labelName, addr, in_data_bank);
                    if (labelName[0] != '.') {
                        strncpy(lastGlobalLabel, labelName, sizeof(lastGlobalLabel) - 1);
                    }

                    lp = colon + 1;
                    skipWs(lp);
                    if (!*lp) continue;  // label-only line
                }
            }

            // Parse instruction
            if (in_data_bank) {
                // In data bank, bare lines are data bytes
                if (*lp == '#') continue;  // handled above
                char tok[32];
                parseToken(lp, tok, sizeof(tok));
                int v = resolveValue(tok, lineNum, pass1);
                emitData(v & 0xFF, pass1);
                continue;
            }

            parseInstruction(lp, lineNum, pass1);
        }
    }

    void parseInstruction(const char* lp, int lineNum, bool pass1) {
        char mnemonic[16];
        int mlen = parseToken(lp, mnemonic, sizeof(mnemonic));
        if (mlen == 0) return;
        const char* afterMnem = lp + mlen;  // pointer into SOURCE after mnemonic
        skipWs(afterMnem);

        // Build extended mnemonic for multi-word fixed instructions (exr, exw)
        // Try progressively longer matches
        char extMnem[32];
        strncpy(extMnem, mnemonic, sizeof(extMnem) - 1);
        extMnem[sizeof(extMnem) - 1] = 0;
        const char* afterExt = afterMnem;  // tracks position after extended mnemonic

        // ── Check fixed instructions (longest match first) ──
        for (int pass = 0; pass < 3; pass++) {
            for (int i = 0; FIXED_INSTRUCTIONS[i].mnemonic; i++) {
                if (strcasecmp(extMnem, FIXED_INSTRUCTIONS[i].mnemonic) == 0) {
                    emitCode(FIXED_INSTRUCTIONS[i].opcode, pass1);
                    if (FIXED_INSTRUCTIONS[i].args == ARGS_IMM) {
                        char tok[32];
                        parseToken(afterExt, tok, sizeof(tok));
                        int v = resolveValue(tok, lineNum, pass1);
                        emitCode(v & 0xFF, pass1);
                    }
                    return;
                }
            }
            // Try extending mnemonic with next token
            if (*afterExt && pass < 2) {
                char nextTok[16];
                int nlen = parseToken(afterExt, nextTok, sizeof(nextTok));
                if (nlen > 0) {
                    int elen = strlen(extMnem);
                    extMnem[elen] = ' ';
                    strncpy(extMnem + elen + 1, nextTok, sizeof(extMnem) - elen - 2);
                    afterExt += nlen;
                    skipWs(afterExt);
                } else break;
            } else break;
        }

        // ── "out" with register or immediate ──
        if (strcasecmp(mnemonic, "out") == 0) {
            const char* ap = afterMnem;
            if (!*ap) {
                emitCode(0x06, pass1);  // out = mov $a, $out
                return;
            }
            char tok[32];
            parseToken(ap, tok, sizeof(tok));
            int reg = findRegister(tok);
            if (reg >= 0) {
                emitCode((0b00 << 6) | (reg << 3) | 6, pass1);  // mov reg, $out
                return;
            }
            // Immediate
            int v = resolveValue(tok, lineNum, pass1);
            emitCode(0x3E, pass1);  // mov imm, $out = 00_111_110
            emitCode(v & 0xFF, pass1);
            return;
        }

        const char* args = afterMnem;

        // ── "cmp" ──
        if (strcasecmp(mnemonic, "cmp") == 0) {
            char tok[32];
            parseToken(args, tok, sizeof(tok));
            for (int i = 0; i < 4; i++) {
                if (CMP_VARIANTS[i].operand && strcasecmp(tok, CMP_VARIANTS[i].operand) == 0) {
                    emitCode(CMP_VARIANTS[i].opcode, pass1);
                    return;
                }
            }
            // Immediate form
            emitCode(0x86, pass1);
            int v = resolveValue(tok, lineNum, pass1);
            emitCode(v & 0xFF, pass1);
            return;
        }

        // ── "push" / "pop" ──
        if (strcasecmp(mnemonic, "push") == 0) {
            char tok[32];
            parseToken(args, tok, sizeof(tok));
            int reg = findRegister(tok);
            if (reg >= 0 && reg < 6) {
                emitCode((0b10 << 6) | (reg << 3) | 4, pass1);  // stor reg, [$sp]
                return;
            }
            addError(lineNum, "push: invalid register");
            return;
        }
        if (strcasecmp(mnemonic, "pop") == 0) {
            char tok[32];
            parseToken(args, tok, sizeof(tok));
            int reg = findRegister(tok);
            if (reg >= 0 && reg < 7) {
                emitCode((0b01 << 6) | (reg << 3) | 4, pass1);  // load reg, [$sp]
                return;
            }
            addError(lineNum, "pop: invalid register");
            return;
        }

        // ── "mov" — register to register ──
        if (strcasecmp(mnemonic, "mov") == 0) {
            char tok1[32], tok2[32];
            int n1 = parseToken(args, tok1, sizeof(tok1));
            args += n1; skipWs(args);
            parseToken(args, tok2, sizeof(tok2));
            int src = findRegister(tok1);
            int dst = findRegister(tok2);
            if (src >= 0 && dst >= 0) {
                emitCode((0b00 << 6) | (src << 3) | dst, pass1);
                return;
            }
            addError(lineNum, "mov: expected two registers");
            return;
        }

        // ── "ldi" — load immediate ──
        if (strcasecmp(mnemonic, "ldi") == 0) {
            char tok1[32], tok2[32];
            int n1 = parseToken(args, tok1, sizeof(tok1));
            args += n1; skipWs(args);
            parseToken(args, tok2, sizeof(tok2));
            int reg = findRegister(tok1);
            if (reg >= 0) {
                emitCode((0b00 << 6) | (7 << 3) | reg, pass1);  // mov imm, reg
                int v = resolveValue(tok2, lineNum, pass1);
                emitCode(v & 0xFF, pass1);
                return;
            }
            addError(lineNum, "ldi: expected register and value");
            return;
        }

        // ── "ld" — load from memory ──
        if (strcasecmp(mnemonic, "ld") == 0) {
            char tok1[32], tok2[32];
            int n1 = parseToken(args, tok1, sizeof(tok1));
            args += n1; skipWs(args);

            int dst = findRegister(tok1);
            if (dst < 0) { addError(lineNum, "ld: expected destination register"); return; }

            // Check for [reg] syntax
            if (*args == '[') {
                args++; skipWs(args);
                parseToken(args, tok2, sizeof(tok2));
                int addr_reg = findRegister(tok2);
                if (addr_reg >= 0) {
                    emitCode((0b01 << 6) | (dst << 3) | addr_reg, pass1);
                    return;
                }
            }
            // Immediate address
            parseToken(args, tok2, sizeof(tok2));
            int v = resolveValue(tok2, lineNum, pass1);
            emitCode((0b01 << 6) | (dst << 3) | 7, pass1);
            emitCode(v & 0xFF, pass1);
            return;
        }

        // ── "st" — store to memory ──
        if (strcasecmp(mnemonic, "st") == 0) {
            char tok1[32], tok2[32];
            int n1 = parseToken(args, tok1, sizeof(tok1));
            args += n1; skipWs(args);

            int src = findRegister(tok1);
            if (src < 0) { addError(lineNum, "st: expected source register"); return; }

            if (*args == '[') {
                args++; skipWs(args);
                parseToken(args, tok2, sizeof(tok2));
                int addr_reg = findRegister(tok2);
                if (addr_reg >= 0) {
                    emitCode((0b10 << 6) | (src << 3) | addr_reg, pass1);
                    return;
                }
            }
            parseToken(args, tok2, sizeof(tok2));
            int v = resolveValue(tok2, lineNum, pass1);
            emitCode((0b10 << 6) | (src << 3) | 7, pass1);
            emitCode(v & 0xFF, pass1);
            return;
        }

        // ── ALU immediate: "addi", "subi", "ori", "andi" ──
        {
            int alen = strlen(mnemonic);
            if (alen >= 4 && mnemonic[alen - 1] == 'i') {
                char base[16];
                strncpy(base, mnemonic, alen - 1);
                base[alen - 1] = 0;
                int op = findAluOp(base);
                if (op >= 0) {
                    char tok1[32], tok2[32];
                    int n1 = parseToken(args, tok1, sizeof(tok1));
                    args += n1; skipWs(args);
                    parseToken(args, tok2, sizeof(tok2));
                    int imm = resolveValue(tok1, lineNum, pass1);
                    int dst = findRegister(tok2);
                    if (dst >= 0 && dst < 4) {
                        emitCode((0b1011 << 4) | (op << 2) | dst, pass1);
                        emitCode(imm & 0xFF, pass1);
                        return;
                    }
                    addError(lineNum, "ALU immediate: invalid destination (must be $a-$d)");
                    return;
                }
            }
        }

        // ── ALU register: "add", "sub", "or", "and" ──
        {
            int op = findAluOp(mnemonic);
            if (op >= 0) {
                char tok1[32], tok2[32];
                int n1 = parseToken(args, tok1, sizeof(tok1));
                args += n1; skipWs(args);
                parseToken(args, tok2, sizeof(tok2));
                int rs = findRegister(tok1);
                int rd = findRegister(tok2);
                if (rs >= 0 && rs < 4 && rd >= 0 && rd < 4) {
                    uint8_t opc = (0b11 << 6) | (op << 4) | (rs << 2) | rd;
                    // Check for reclaimed opcodes that are now special instructions
                    static const uint8_t reclaimed[] = {0xC3,0xC7,0xD3,0xD7,0xDB,0xDE,0xE3,0xE7,0xF3,0xF7,0};
                    bool collision = false;
                    for (int i = 0; reclaimed[i]; i++) {
                        if (opc == reclaimed[i]) { collision = true; break; }
                    }
                    if (collision) {
                        addError(lineNum, "ALU combo reclaimed for special instruction");
                        return;
                    }
                    emitCode(opc, pass1);
                    return;
                }
                addError(lineNum, "ALU: operands must be $a-$d");
                return;
            }
        }

        // ── Unknown instruction ──
        char msg[80];
        snprintf(msg, sizeof(msg), "Unknown instruction: %.40s", mnemonic);
        addError(lineNum, msg);
    }
};
