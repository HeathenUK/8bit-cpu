// MK1 CPU — ESP32 Web IDE + Program Uploader
//
// Runs a WiFi access point with a web-based assembly editor.
// Enter MK1 assembly -> Assemble & Run -> program is written to SRAM.
//
// Connect to WiFi "MK1-CPU" (no password) -> open http://192.168.4.1
//
// Also supports serial upload with "MK" magic header for uploader.py.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <FFat.h>
#include "assembler.h"
#include "web_ui.h"

// ── Pin mapping (Arduino Nano ESP32 header) ──────────────────────────

static const int PIN_MI    = D10;
static const int PIN_HL    = D11;
static const int PIN_RI    = D12;
static const int PIN_EN    = D13;
static const int PIN_CLK   = A0;
static const int PIN_RST   = A1;
static const int PIN_CU_EN = A2;
static const int PIN_CLK_SENSE = A3;  // bodge wire to TP2 — reads actual system clock
static const int PIN_STK   = A4;     // bodge wire to STK — page 2/3 select
static const int PIN_DIR   = A5;     // bodge wire to U59 pin 1 (DIR) — bus transceiver direction
static const int PIN_RO    = A6;     // bodge wire to TP20 (~RO) — RAM output enable (active low)
static const int PIN_OI    = A7;     // bodge wire to OI test point — output register latch
static const int PIN_HLT   = D1;     // bodge wire to HLT LED — detect program halt

static const int BUS_PINS[8] = { D2, D3, D4, D5, D6, D7, D8, D9 };

// ── WiFi / Web ───────────────────────────────────────────────────────

static const char* AP_SSID = "MK1-CPU";
static const char* MDNS_HOST = "mk1";
static const char* WIFI_CONFIG_PATH = "/wifi.cfg";

// WiFi config: first line = SSID, second line = password
static char staSSID[64] = "";
static char staPSK[64] = "";
static bool staMode = false;  // true if connected to existing network

WebServer server(80);
MK1Assembler assembler;

// Last assembled binary, ready for upload
static uint8_t uploadBuf[1024];  // 256 code + 256 data + 256 stack + 256 page3
static int uploadSize = 0;

// Forward declarations
static void autoSaveSource(const String& source);
static void startOIMonitor();
static void stopOIMonitor();
static int readDS3231Temp();
static int readDS3231Reg(uint8_t reg);
static int readAt24c512(int offset, int count, uint8_t* out);
static bool updateBootLCD(int tempC, bool rtcSet, uint8_t hourBcd, uint8_t minBcd);
static void loadAsmResultForUpload(const AsmResult& r);
static int runUploadedUntilFirstOut(int maxCycles);
static bool runUploadedUntilHalt(int maxCycles);
static void writeEepromData(const uint8_t* data, int size);
static void writeEe64Data(const uint8_t* data, int size);
static inline uint8_t IRAM_ATTR decodeBus(uint32_t gpio);

// ── Clock measurement ────────────────────────────────────────────────

static volatile uint32_t clkEdgeCount = 0;
static uint32_t lastClkCount = 0;
static unsigned long lastClkTime = 0;
static float measuredClkHz = 0;
static bool clkMonitorActive = false;

static void IRAM_ATTR onClkRising() {
    clkEdgeCount++;
}

static void startClkMonitor() {
    if (!clkMonitorActive) {
        clkEdgeCount = 0;
        lastClkCount = 0;
        lastClkTime = millis();
        measuredClkHz = 0;
        attachInterrupt(digitalPinToInterrupt(PIN_CLK_SENSE), onClkRising, RISING);
        clkMonitorActive = true;
    }
}

static void stopClkMonitor() {
    if (clkMonitorActive) {
        detachInterrupt(digitalPinToInterrupt(PIN_CLK_SENSE));
        clkMonitorActive = false;
    }
}

static void updateClkMeasurement() {
    if (!clkMonitorActive) return;
    unsigned long now = millis();
    unsigned long dt = now - lastClkTime;
    if (dt >= 500) {  // update every 500ms
        uint32_t count = clkEdgeCount;
        uint32_t edges = count - lastClkCount;
        measuredClkHz = (float)edges * 1000.0f / (float)dt;
        lastClkCount = count;
        lastClkTime = now;

    }
}

// ── CPU state ────────────────────────────────────────────────────────

enum CpuState { CPU_RUNNING, CPU_HALTED, CPU_STEPPING };
static CpuState cpuState = CPU_RUNNING;
static uint32_t totalCycles = 0;
static uint8_t stepNum = 0;     // microcode step within instruction (0-7)
static uint8_t stepPC = 0;      // PC captured at step 0
static uint8_t stepOpcode = 0;  // opcode captured at step 1

// ── Bus reading ──────────────────────────────────────────────────────

static void busSetInput() {
    for (int i = 0; i < 8; i++)
        pinMode(BUS_PINS[i], INPUT);
}

static uint8_t busRead() {
    uint8_t val = 0;
    for (int i = 0; i < 8; i++)
        if (digitalRead(BUS_PINS[i])) val |= (1 << i);
    return val;
}

static inline bool fastPinHigh(int pin) {
    int gpio = digitalPinToGPIONumber(pin);
    if (gpio < 0) gpio = pin;
    if (gpio < 32) return (GPIO.in & (1UL << gpio)) != 0;
    return (GPIO.in1.val & (1UL << (gpio - 32))) != 0;
}

static inline bool hltOpcodeObserved(uint32_t gpioSample) {
    return fastPinHigh(PIN_HLT) && decodeBus(gpioSample) == 0x7F;
}

// ── Hardware control ─────────────────────────────────────────────────

static void busSetOutput() {
    for (int i = 0; i < 8; i++)
        pinMode(BUS_PINS[i], OUTPUT);
}

static void putOut(uint8_t data) {
    for (int i = 0; i < 8; i++)
        digitalWrite(BUS_PINS[i], (data >> i) & 1);
    delayMicroseconds(2);
}

static void resetPulse() {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_RST, LOW);
    pinMode(PIN_RST, INPUT);
}

static int customClkHz = 0;
static const int CLK_LEDC_CHANNEL = 0;
static void stopCustomClock();  // forward declaration

static void enableClock() {
    if (customClkHz > 0) {
        ledcWrite(CLK_LEDC_CHANNEL, 0);
        ledcDetachPin(PIN_CLK);
        customClkHz = 0;
    }
    stopClkMonitor();
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);   // ensure clean state
    delayMicroseconds(10);
    digitalWrite(PIN_CLK, HIGH);
}

static void disableClock() {
    if (customClkHz > 0) stopCustomClock();
    stopClkMonitor();
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);
    pinMode(PIN_CLK, INPUT);
    startClkMonitor();
}

static void disableCU() { digitalWrite(PIN_CU_EN, HIGH); }
static void enableCU()  { digitalWrite(PIN_CU_EN, LOW);  }

// ── Programmable clock ──────────────────────────────────────────────

static hw_timer_t* clkTimer = NULL;
static volatile bool clkTimerState = false;

static void IRAM_ATTR onClkTimer() {
    clkTimerState = !clkTimerState;
    digitalWrite(PIN_CLK, clkTimerState);
}

static bool usingLEDC = false;
static unsigned long ledcStartTime = 0;
static volatile bool hltFired = false;
static bool hltIrqAttached = false;

static void IRAM_ATTR onHltRising() {
    // Verify HLT is actually HIGH (debounce — filter out brief glitches)
    if (digitalRead(PIN_HLT) == HIGH) {
        ledcWrite(CLK_LEDC_CHANNEL, 0);
        hltFired = true;
    }
}

static void startCustomClock(int hz) {
    stopClkMonitor();
    if (customClkHz > 0) {
        // Stop any existing LEDC before reconfiguring
        ledcWrite(CLK_LEDC_CHANNEL, 0);
        ledcDetachPin(PIN_CLK);
    }
    customClkHz = hz;
    pinMode(PIN_CLK, OUTPUT);

    if (hz < 150) hz = 150;  // LEDC prescaler can't go lower than ~150Hz

    // Use 8-bit resolution for proper clock output
    uint8_t resolution = 8;
    uint32_t duty = 128;  // 50% at 8-bit resolution

    ledcSetup(CLK_LEDC_CHANNEL, hz, resolution);
    ledcAttachPin(PIN_CLK, CLK_LEDC_CHANNEL);
    ledcWrite(CLK_LEDC_CHANNEL, duty);
    usingLEDC = true;
    ledcStartTime = millis();
    hltFired = false;
    // Don't attach HLT interrupt yet — HLT may be high during reset.
    // loop() will attach it after a grace period.
    startClkMonitor();  // keep measuring while custom clock runs
}

static void stopCustomClock() {
    ledcWrite(CLK_LEDC_CHANNEL, 0);
    ledcDetachPin(PIN_CLK);
    customClkHz = 0;
    usingLEDC = false;
    pinMode(PIN_CLK, INPUT);
    startClkMonitor();
}


static void enableOutput()  { digitalWrite(PIN_EN, HIGH); }
static void disableOutput() { digitalWrite(PIN_EN, LOW);  }

static void setAddress(unsigned int address) {
    // Page select: address 0x000-0x0FF = page 0 (code),  STK=0 HL=0
    //              address 0x100-0x1FF = page 1 (data),  STK=0 HL=1
    //              address 0x200-0x2FF = page 2 (stack), STK=1 HL=0
    //              address 0x300-0x3FF = page 3 (extra),  STK=1 HL=1
    digitalWrite(PIN_HL,  (address & 0x100) ? HIGH : LOW);
    digitalWrite(PIN_STK, (address & 0x200) ? HIGH : LOW);
    putOut(address & 0xFF);
    delayMicroseconds(2);
    enableOutput();
    digitalWrite(PIN_MI, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_MI, LOW);
    delayMicroseconds(2);
    disableOutput();
}

static void writeInstruction(uint8_t instr) {
    putOut(instr);
    enableOutput();
    delayMicroseconds(2);
    digitalWrite(PIN_RI, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_RI, LOW);
    delayMicroseconds(2);
    disableOutput();
}

static void uploadToMK1(const uint8_t* buf, int size) {
    int savedClkHz = customClkHz;  // save custom clock state

    stopOIMonitor();
    stopClkMonitor();
    busSetOutput();
    // Ensure page-select pins are OUTPUT for upload (may be INPUT from previous run)
    pinMode(PIN_HL, OUTPUT);
    pinMode(PIN_STK, OUTPUT);
    resetPulse();
    disableCU();
    delayMicroseconds(2);
    enableClock();  // static HIGH for writes (kills custom clock if running)
    delayMicroseconds(2);

    for (int i = 0; i < size; i++) {
        setAddress(i);
        delayMicroseconds(2);
        writeInstruction(buf[i]);
        delayMicroseconds(2);
    }

    disableClock();
    // Release page-select pins — let microcode control HL/STK during execution.
    // If ESP32 holds these as OUTPUT LOW, stack push/pop corrupts code page.
    pinMode(PIN_HL, INPUT);
    pinMode(PIN_STK, INPUT);

    // Release bus — ESP32 must not drive data pins during program execution,
    // otherwise VIA reads (exrw) can't put data on the bus.
    busSetInput();
    disableOutput();

    // Start OI monitor BEFORE releasing CPU — the program may execute
    // and halt within microseconds, so the interrupt must be ready first.
    startOIMonitor();

    resetPulse();
    enableCU();
    totalCycles = 0;
    cpuState = CPU_RUNNING;

    // Restore custom clock if one was running before upload
    if (savedClkHz > 0) {
        startCustomClock(savedClkHz);
    }
}

// ── Single step ──────────────────────────────────────────────────────

static uint8_t singleStep() {
    // Pulse CLK manually, read bus value after rising edge
    stopClkMonitor();

    // Flip to read mode so we can see the bus
    busSetInput();
    disableOutput();
    digitalWrite(PIN_DIR, LOW);       // B→A = bus→ESP32
    enableOutput();

    // Clock pulse
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, HIGH);
    delayMicroseconds(5);
    uint8_t busVal = busRead();
    digitalWrite(PIN_CLK, LOW);
    delayMicroseconds(5);
    pinMode(PIN_CLK, INPUT);

    // Restore write mode
    disableOutput();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();

    totalCycles++;
    return busVal;
}

// ── Opcode info table (auto-generated from microcode.py) ────────────
// Low 4 bits = total step count, bit 4 = has_immediate
static const uint8_t OPCODE_INFO[256] = {
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,  // 0x20-0x2F
    0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x03,  // 0x30-0x3F (original)
    0x04, 0x04, 0x04, 0x04, 0x05, 0x04, 0x04, 0x15, 0x04, 0x04, 0x04, 0x04, 0x05, 0x04, 0x04, 0x15,
    0x04, 0x04, 0x04, 0x04, 0x05, 0x04, 0x04, 0x15, 0x04, 0x04, 0x04, 0x04, 0x05, 0x04, 0x04, 0x15,
    0x04, 0x04, 0x04, 0x04, 0x05, 0x04, 0x04, 0x15, 0x04, 0x04, 0x04, 0x04, 0x06, 0x04, 0x04, 0x15,
    0x04, 0x04, 0x04, 0x04, 0x05, 0x04, 0x04, 0x15, 0x03, 0x03, 0x05, 0x03, 0x03, 0x03, 0x03, 0x03,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x15, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03, 0x15,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03, 0x15, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03, 0x15,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03, 0x15, 0x04, 0x04, 0x04, 0x04, 0x06, 0x04, 0x04, 0x15,
    0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15,
    0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x04, 0x04, 0x04, 0x14, 0x14, 0x04, 0x04, 0x14, 0x06,
    0x04, 0x14, 0x08, 0x04, 0x04, 0x14, 0x04, 0x06, 0x04, 0x04, 0x14, 0x18, 0x04, 0x04, 0x04, 0x18,
    0x07, 0x15, 0x04, 0x06, 0x04, 0x05, 0x04, 0x16, 0x04, 0x04, 0x15, 0x17, 0x04, 0x04, 0x08, 0x15,
    0x04, 0x04, 0x04, 0x17, 0x04, 0x04, 0x04, 0x14, 0x04, 0x04, 0x04, 0x05, 0x04, 0x04, 0x05, 0x15,
};

// ── Clock pulse (no bus read) ────────────────────────────────────────

static void clockPulse() {
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_CLK, LOW);
    delayMicroseconds(5);
    pinMode(PIN_CLK, INPUT);
}

// ── RAM read ────────────────────────────────────────────────────────

static uint8_t readByte(unsigned int address) {
    // Set address via MAR (same as write path)
    busSetOutput();
    setAddress(address);
    delayMicroseconds(2);

    // Switch bus to input, assert ~RO to read RAM onto bus
    busSetInput();
    disableOutput();
    digitalWrite(PIN_RO, LOW);   // ~RO active = RAM drives bus
    delayMicroseconds(2);
    uint8_t val = busRead();
    digitalWrite(PIN_RO, HIGH);  // release ~RO
    delayMicroseconds(2);
    return val;
}

static void readMemory(unsigned int startAddr, unsigned int count, uint8_t* dest) {
    // Read RAM by stepping from reset and capturing bus at each step.
    // Step 0 of each instruction: MI|PO → bus = PC (address)
    // Step 1 of each instruction: RO|II|PE → bus = RAM[PC] (data byte)
    // We step many cycles, capturing (address, data) pairs, then extract
    // the bytes for the requested address range.
    // Requires SW3 in manual (no internal oscillator).
    // Limitation: only reads sequential bytes from PC=0 onwards — startAddr
    // must be 0 for code page reads. Data page not accessible this way.

    stopClkMonitor();

    uint8_t mem[256];
    memset(mem, 0xFF, 256);

    // Reset CPU — PC=0, step counter=0
    resetPulse();
    delayMicroseconds(10);

    // Step every cycle, capture all bus values. Post-process:
    // When bus value matches expected next PC, it's step 0 (address).
    // The next step is always step 1 (data byte from RAM).
    // For 2-byte instructions, steps 2 and 3 also show addr+1 and immediate.
    unsigned int maxSteps = 16384;
    uint8_t lastVal = 0xFF;
    bool lastWasAddr = false;
    uint8_t nextExpectedPC = 0;

    for (unsigned int s = 0; s < maxSteps; s++) {
        uint8_t val = singleStep();

        if (lastWasAddr) {
            // Previous step was address, this step is data byte (step 1)
            uint8_t addr = lastVal;
            uint8_t opcode = val;
            mem[addr] = opcode;

            uint8_t info = OPCODE_INFO[opcode];
            bool hasImm = (info & 0x10) != 0;
            int totalSteps = info & 0x0F;

            if (hasImm) {
                // Step 2: addr+1 on bus (PO|MI)
                singleStep(); s++;
                // Step 3: immediate byte on bus (PE|RO|...)
                uint8_t immVal = singleStep(); s++;
                mem[(addr + 1) & 0xFF] = immVal;
                // Skip remaining execution steps (4 through totalSteps-1)
                for (int r = 4; r < totalSteps; r++) { clockPulse(); s++; }
                nextExpectedPC = (addr + 2) & 0xFF;
            } else {
                // Skip remaining execution steps (2 through totalSteps-1)
                for (int r = 2; r < totalSteps; r++) { clockPulse(); s++; }
                nextExpectedPC = (addr + 1) & 0xFF;
            }
            lastWasAddr = false;
        } else if (val == nextExpectedPC) {
            // Matches expected next address (or wildcard after jump) — this is step 0
            lastWasAddr = true;
        } else {
            lastWasAddr = false;
        }

        lastVal = val;

        // Early exit if we have all bytes
        if (s > 100 && s % 64 == 0) {
            bool complete = true;
            for (unsigned int i = 0; i < count; i++) {
                if (mem[(startAddr + i) & 0xFF] == 0xFF) { complete = false; break; }
            }
            if (complete) break;
        }
    }

    memcpy(dest, mem + startAddr, count);

    resetPulse();
    startClkMonitor();
}

// ── Bus debug ───────────────────────────────────────────────────────

// GET /bussniff — read raw bus value without touching CU or address
// Tests whether U59 read mode works at all
static void handleBusSniff() {
    busSetInput();
    disableOutput();                  // disable U59
    digitalWrite(PIN_DIR, LOW);       // B→A read mode
    enableOutput();                   // re-enable U59 in read mode
    delayMicroseconds(10);
    uint8_t val1 = busRead();         // read bus (whatever CPU left on it)

    // Now also try with ~RO asserted
    pinMode(PIN_RO, OUTPUT);
    digitalWrite(PIN_RO, LOW);
    delayMicroseconds(10);
    uint8_t val2 = busRead();
    digitalWrite(PIN_RO, HIGH);
    pinMode(PIN_RO, INPUT);

    // Restore write mode
    disableOutput();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();

    String json = "{\"bus_no_ro\":" + String(val1) +
                  ",\"bus_with_ro\":" + String(val2) + "}";
    server.send(200, "application/json", json);
}

// ── Output register sampling via OI pin ─────────────────────────────
// OI is driven by the microcode EEPROMs. When the CPU executes 'out',
// OI goes HIGH for one clock cycle while AO drives the bus with A's value.
// We attach an interrupt on OI rising edge to snapshot the bus value.

static volatile uint8_t lastOutputVal = 0;
static volatile bool outputCaptured = false;
static volatile uint32_t oiCount = 0;        // total OI events (may exceed buffer)
// Ring buffer size. Bumped from 256 → 4096 to fit `MK1_DEBUG_DUMP_PAGE3=1`
// runs (256 page-3 dump bytes + the program's normal output). ESP32-S3
// has 320 KB SRAM so the ~20 KB cost (4096 × (1+4)) is trivial. Must
// remain a power of two so the modulo can stay an `&` mask.
#define OI_BUF_SIZE 4096
#define OI_BUF_MASK (OI_BUF_SIZE - 1)
static volatile uint8_t oiHistory[OI_BUF_SIZE];      // ring buffer — keeps LAST OI_BUF_SIZE
// Parallel ring buffer of per-event timestamps (MK1-cycle counts).
// Set to `actualCycles` at capture time inside the RUNLOG/RUNNB loop;
// the ISR-capture path sets it to 0 (no cycle counter available there).
// Reporting: RUNNB/RUNLOG emit a `ts` array alongside `hist`/`vals`.
static volatile uint32_t oiTimes[OI_BUF_SIZE];

// Ring-buffer accessors. Before this change, the code stored only the
// first 256 events and dropped the rest. That's fatal for I2C-heavy
// programs where the real `out()` emissions are at the END of the run
// and get pushed past the buffer by the intermediate bus traffic. Now
// we store the most RECENT OI_BUF_SIZE events; real outputs stay visible.
//
// oiHistStored(): how many slots are in use (min(oiCount, OI_BUF_SIZE))
// oiHistAt(i):    i=0 is the OLDEST kept event, i=stored-1 is the NEWEST
static inline uint32_t oiHistStored() {
    return oiCount < OI_BUF_SIZE ? oiCount : OI_BUF_SIZE;
}
static inline uint8_t oiHistAt(uint32_t i) {
    if (oiCount <= OI_BUF_SIZE) return oiHistory[i];
    // Wrapped: physical index of oldest kept event = oiCount (mod size).
    return oiHistory[(oiCount + i) & OI_BUF_MASK];
}
static inline uint32_t oiTimeAt(uint32_t i) {
    if (oiCount <= OI_BUF_SIZE) return oiTimes[i];
    return oiTimes[(oiCount + i) & OI_BUF_MASK];
}

static uint32_t oiGpioMask = 0;  // precomputed at startOIMonitor

// Decode an 8-bit bus value from a GPIO register snapshot. Keep the
// zero-arg variant for back-compat; callers with their own snapshot
// (e.g. the RUNLOG/RUNNB loop that samples once and checks OI + reads
// bus from the SAME snapshot) should use `decodeBus(gpio)` directly.
static inline uint8_t IRAM_ATTR decodeBus(uint32_t gpio) {
    uint8_t val = 0;
    if (gpio & (1 << 5))  val |= 0x01;
    if (gpio & (1 << 6))  val |= 0x02;
    if (gpio & (1 << 7))  val |= 0x04;
    if (gpio & (1 << 8))  val |= 0x08;
    if (gpio & (1 << 9))  val |= 0x10;
    if (gpio & (1 << 10)) val |= 0x20;
    if (gpio & (1 << 17)) val |= 0x40;
    if (gpio & (1 << 18)) val |= 0x80;
    return val;
}
static inline uint8_t IRAM_ATTR readBusFast() {
    return decodeBus(GPIO.in);
}

static volatile uint32_t lastOiTime = 0;
static void IRAM_ATTR onOIRising() {
    // Debounce: ignore OI events within 1ms of the previous one.
    // Real out_imm events are seconds apart (during tone playback).
    // Post-HLT bus noise fires every few microseconds.
    uint32_t now = micros();
    if (now - lastOiTime < 1000) return;  // 1ms debounce
    lastOiTime = now;
    // ISR: snapshot GPIO now and decode, keeping OI+bus consistent.
    uint32_t gpio = GPIO.in;
    uint8_t val = decodeBus(gpio);
    // ISR can't access the per-cycle `actualCycles` counter used by
    // RUN/RUNNB/RUNLOG. Record wall-clock microseconds instead — enough
    // for external timing verification (ms delay accuracy etc.).
    oiHistory[oiCount & OI_BUF_MASK] = val;
    oiTimes[oiCount & OI_BUF_MASK] = now;
    oiCount++;
    lastOutputVal = val;
    outputCaptured = true;
}

static bool oiMonitorActive = false;

static void startOIMonitor() {
    if (oiMonitorActive) return;
    busSetInput();
    disableOutput();
    digitalWrite(PIN_DIR, LOW);   // B→A read mode
    enableOutput();
    int gpioNum = digitalPinToGPIONumber(PIN_OI);
    if (gpioNum < 0) gpioNum = PIN_OI;
    oiGpioMask = 1 << gpioNum;
    outputCaptured = false;
    oiCount = 0;
    attachInterrupt(digitalPinToInterrupt(PIN_OI), onOIRising, RISING);
    oiMonitorActive = true;
}

static void stopOIMonitor() {
    if (!oiMonitorActive) return;
    detachInterrupt(digitalPinToInterrupt(PIN_OI));
    disableOutput();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();
    enableOutput();
    oiMonitorActive = false;
}

static void detachOIMonitorForManualRun() {
    if (!oiMonitorActive) return;
    detachInterrupt(digitalPinToInterrupt(PIN_OI));
    oiMonitorActive = false;
}

// POST /upload_and_wait?timeout=10 — upload program, then immediately spin-poll for OI.
// Combines upload + capture in one request so we don't miss fast programs.
static void handleUploadAndWait() {
    if (uploadSize == 0) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"Nothing assembled\"}");
        return;
    }
    int timeoutSec = 10;
    if (server.hasArg("timeout")) timeoutSec = server.arg("timeout").toInt();
    if (timeoutSec < 1) timeoutSec = 1;
    if (timeoutSec > 60) timeoutSec = 60;

    int oiGpio = digitalPinToGPIONumber(PIN_OI);
    if (oiGpio < 0) oiGpio = PIN_OI;
    uint32_t oiMask = 1 << oiGpio;

    // Upload program (starts OI monitor with U59 in read mode).
    uploadToMK1(uploadBuf, uploadSize);

    bool found = false;
    uint8_t val = 0;

    // Don't spin-poll — it causes false ACKs on the I2C bus.
    // Just wait for the program to complete, then read the ISR result.
    // The ISR (from startOIMonitor) captures OI values in oiHistory.
    delay(timeoutSec * 1000);

    // Find first non-spurious ISR value (filter 63=floating, 127=HLT).
    // Iterate OLDEST→NEWEST through the ring buffer.
    uint32_t n = oiHistStored();
    for (uint32_t i = 0; i < n; i++) {
        uint8_t v = oiHistAt(i);
        if (v != 0x3F && v != 0x7F && v != 0xBF && v != 0xFF) {
            val = v;
            found = true;
            break;
        }
    }
    // If all spurious, report the oldest one anyway
    if (!found && n > 0) {
        val = oiHistAt(0);
        found = true;
    }

    String json = "{\"value\":" + String(val) +
                  ",\"found\":" + (found ? "true" : "false") +
                  ",\"oi_count\":" + String(oiCount) +
                  ",\"history\":[";
    for (uint32_t i = 0; i < n; i++) {
        if (i > 0) json += ",";
        json += String(oiHistAt(i));
    }
    json += "]}";
    server.send(200, "application/json", json);
}

// POST /wait_output?timeout=10 — spin-poll GPIO.in until OI fires, return bus value.
// GPIO.in captures OI + all 8 bus pins ATOMICALLY — no ISR latency.
// Blocks the web server until capture or timeout.
static void handleWaitOutput() {
    int timeoutSec = 10;
    if (server.hasArg("timeout")) timeoutSec = server.arg("timeout").toInt();
    if (timeoutSec < 1) timeoutSec = 1;
    if (timeoutSec > 60) timeoutSec = 60;

    int oiGpio = digitalPinToGPIONumber(PIN_OI);
    if (oiGpio < 0) oiGpio = PIN_OI;
    uint32_t oiMask = 1 << oiGpio;

    unsigned long deadline = millis() + (timeoutSec * 1000);
    bool found = false;
    uint8_t val = 0;

    while (millis() < deadline) {
        uint32_t gpio = GPIO.in;
        if (gpio & oiMask) {
            // OI is HIGH — extract bus value from SAME snapshot
            val = 0;
            if (gpio & (1 << 5))  val |= 0x01;
            if (gpio & (1 << 6))  val |= 0x02;
            if (gpio & (1 << 7))  val |= 0x04;
            if (gpio & (1 << 8))  val |= 0x08;
            if (gpio & (1 << 9))  val |= 0x10;
            if (gpio & (1 << 10)) val |= 0x20;
            if (gpio & (1 << 17)) val |= 0x40;
            if (gpio & (1 << 18)) val |= 0x80;
            found = true;
            break;
        }
    }

    String json = "{\"value\":" + String(val) +
                  ",\"found\":" + (found ? "true" : "false") + "}";
    server.send(200, "application/json", json);
}

// GET /read_output — return the last value captured from an 'out' instruction
static void handleReadOutput() {
    int intNum = digitalPinToInterrupt(PIN_OI);
    int gpioNum = digitalPinToGPIONumber(PIN_OI);
    int pinState = digitalRead(PIN_OI);
    // Find first value that isn't a known spurious bus state:
    // 0x3F (63) = floating bus after halt
    // 0x7F (127) = HLT opcode during fetch
    // Pick the NEWEST non-spurious value from the ring buffer — it's the
    // program's final out() emission (or the one right before halt).
    uint8_t reportVal = 0;
    bool found = false;
    uint32_t n = oiHistStored();
    for (uint32_t i = n; i > 0; i--) {
        uint8_t v = oiHistAt(i - 1);
        if (v != 0x3F && v != 0x7F) {
            reportVal = v;
            found = true;
            break;
        }
    }
    if (!found && n > 0) reportVal = oiHistAt(n - 1);  // fallback to newest
    String json = "{\"value\":" + String(reportVal) +
                  ",\"captured\":" + (outputCaptured ? "true" : "false") +
                  ",\"oi_count\":" + String(oiCount) +
                  ",\"oi_pin\":" + String(PIN_OI) +
                  ",\"oi_gpio\":" + String(gpioNum) +
                  ",\"oi_int\":" + String(intNum) +
                  ",\"oi_state\":" + String(pinState) +
                  ",\"oi_active\":" + (oiMonitorActive ? "true" : "false") +
                  ",\"history\":[";
    for (uint32_t i = 0; i < n; i++) {
        if (i > 0) json += ",";
        json += String(oiHistAt(i));
    }
    json += "]}";
    server.send(200, "application/json", json);
}

// POST /run_cycles?n=500000 — pulse clock N times (default 500000).
// Unlike LEDC clock, this respects program flow — stops after N pulses.
// Use with OI monitor to capture output values reliably.
static void handleRunCycles() {
    int n = 500000;
    if (server.hasArg("n")) n = server.arg("n").toInt();
    if (n > 100000000) n = 100000000;
    int halfPeriodUs = 1;  // microseconds per half-cycle (default ~500kHz — optimal for this board)
    if (server.hasArg("us")) halfPeriodUs = server.arg("us").toInt();
    if (halfPeriodUs < 0) halfPeriodUs = 0;
    if (halfPeriodUs > 1000) halfPeriodUs = 1000;
    bool noBreak = server.hasArg("nobreak");  // don't stop on OI — for multi-output programs

    stopCustomClock();
    // Detach OI interrupt — we'll poll OI directly in the clock loop
    detachOIMonitorForManualRun();
    // Reset OI capture state
    outputCaptured = false;
    oiCount = 0;

    // Poll OI at each clock edge instead of relying on interrupts.
    // Check OI during both HIGH and LOW phases to catch the pulse.
    int oiGpio = digitalPinToGPIONumber(PIN_OI);
    if (oiGpio < 0) oiGpio = PIN_OI;
    uint32_t oiMask = 1 << oiGpio;

    // U59 in read mode so we can see the bus via GPIO.in
    busSetInput();
    disableOutput();
    digitalWrite(PIN_DIR, LOW);       // B→A read mode
    enableOutput();

    // Pre-compute GPIO masks for fast register writes
    // PIN_CLK = A0 = GPIO1 (in GPIO.out, bit 1)
    int clkGpio = digitalPinToGPIONumber(PIN_CLK);
    uint32_t clkMask = 1 << clkGpio;

    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);

    int actualCycles = 0;
    for (int i = 0; i < n; i++) {
        actualCycles++;

        // CLK HIGH
        GPIO.out_w1ts = clkMask;
        if (halfPeriodUs > 0) delayMicroseconds(halfPeriodUs);

        // Check OI during HIGH phase
        uint32_t gpio1 = GPIO.in;
        if (gpio1 & oiMask) {
            uint8_t val = decodeBus(gpio1);
            oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
            oiCount++;
            lastOutputVal = val;
            outputCaptured = true;
            if (!noBreak) {
                GPIO.out_w1tc = clkMask;
                break;
            }
        }

        // CLK LOW
        GPIO.out_w1tc = clkMask;
        if (halfPeriodUs > 0) delayMicroseconds(halfPeriodUs);

        // Check OI during LOW phase
        uint32_t gpio2 = GPIO.in;
        if (gpio2 & oiMask) {
            uint8_t val = decodeBus(gpio2);
            oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
            oiCount++;
            lastOutputVal = val;
            outputCaptured = true;
            if (!noBreak) break;
        }
    }
    pinMode(PIN_CLK, INPUT);

    // Restore bus to write mode
    disableOutput();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();
    enableOutput();

    // Sample OI pin state and raw GPIO register for debug
    uint32_t rawGpio = GPIO.in;
    int oiDirect = digitalRead(PIN_OI);
    int oiFromReg = (rawGpio & oiMask) ? 1 : 0;
    String json = "{\"ok\":true,\"cycles\":" + String(actualCycles) +
                  ",\"max\":" + String(n) +
                  ",\"oi_gpio\":" + String(oiGpio) +
                  ",\"oi_mask\":\"0x" + String(oiMask, HEX) +
                  "\",\"oi_direct\":" + String(oiDirect) +
                  ",\"oi_from_reg\":" + String(oiFromReg) +
                  ",\"gpio_raw\":\"0x" + String(rawGpio, HEX) + "\"}";
    server.send(200, "application/json", json);
}

// GET /stepn?n=16 — single-step N times, return bus value at each step
static void handleStepN() {
    int n = 8;
    if (server.hasArg("n")) n = server.arg("n").toInt();
    if (n > 256) n = 256;
    if (n < 1) n = 1;

    String json = "{\"steps\":[";
    for (int i = 0; i < n; i++) {
        uint8_t val = singleStep();
        if (i > 0) json += ",";
        char hex[8];
        snprintf(hex, sizeof(hex), "\"0x%02X\"", val);
        json += hex;
    }
    json += "],\"total_cycles\":" + String(totalCycles) + "}";
    server.send(200, "application/json", json);
}

// GET /probe — disables CU, holds ~RO LOW and DIR in read mode for 10 seconds
static void handleProbe() {
    stopClkMonitor();
    busSetOutput();
    disableCU();                      // EEPROMs go high-Z — safe to drive ~RO
    enableClock();
    delayMicroseconds(5);

    // Set address 0 so we read a known location
    setAddress(0);
    delayMicroseconds(2);

    busSetInput();
    disableOutput();
    digitalWrite(PIN_DIR, LOW);       // read mode
    enableOutput();
    pinMode(PIN_RO, OUTPUT);
    digitalWrite(PIN_RO, LOW);        // ~RO asserted
    delayMicroseconds(10);

    uint8_t val = busRead();

    server.send(200, "application/json",
        "{\"status\":\"probing 10s — measure A6, TP20, U49 pin 19, bus pins\",\"bus\":" + String(val) + "}");

    delay(10000);                     // hold for 10 seconds

    digitalWrite(PIN_RO, HIGH);
    pinMode(PIN_RO, INPUT);
    disableOutput();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();
    disableClock();
    pinMode(PIN_HL, INPUT);
    pinMode(PIN_STK, INPUT);
    enableCU();
}

// ── MK1 VIA I2C scan — scans addresses 0x08-0x77 via daughter board ──
static int runI2CScanAddr(uint8_t addr) {
    // Build a scan program for a single 7-bit I2C address.
    // Returns 1 if ACK (device found), 0 if NACK, -1 on error.
    uint8_t writeAddr = addr << 1;  // 7-bit addr → 8-bit write addr
    char asmBuf[2048];
    int asmLen = snprintf(asmBuf, sizeof(asmBuf),
        "; I2C scan address 0x%02X\n"
        "    ldi $d, 0\n"
        ".dly:\n"
        "    dec\n"
        "    jnz .dly\n"
        "    clr $a\n"
        "    exw 0 0\n"
        "    ddrb_imm 0x00\n"
        "    clr $a\n"
        "    exw 0 3\n"
        // Bus recovery: 9 SCL clocks with SDA released. Required — if a prior
        // transaction left a slave holding SDA low, every address we scan
        // would appear to ACK. Inline (no loop) — per i2c_scan.c comment,
        // loop overhead shifts timing and causes false ACKs.
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // clock 1
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // 2
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // 3
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // 4
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // 5
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // 6
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // 7
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // 8
        "    ddrb_imm 0x02\n    ddrb_imm 0x00\n"  // 9
        // Clean STOP to end any prior transaction
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        // START
        "    exrw 2\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        // Send address byte
        "    ldi $a, 0x%02X\n"
        "    jal __sb\n"
        // Check ACK (bit 0 of last exrw 0 in __sb)
        "    tst 0x01\n"
        "    jnz .nack\n"
        // STOP
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        "    out_imm 1\n"
        "    hlt\n"
        ".nack:\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        "    out_imm 0\n"
        "    hlt\n"
        "__sb:\n"
        "    mov $a, $b\n"
        "    ldi $a, 8\n"
        "    mov $a, $c\n"
        ".isb:\n"
        "    mov $b, $a\n"
        "    tst 0x80\n"
        "    jnz .isbh\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    j .isbn\n"
        ".isbh:\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    ddrb_imm 0x02\n"
        ".isbn:\n"
        "    mov $b, $a\n"
        "    sll\n"
        "    mov $a, $b\n"
        "    mov $c, $a\n"
        "    dec\n"
        "    mov $a, $c\n"
        "    jnz .isb\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    exrw 0\n"
        "    ddrb_imm 0x02\n"
        "    ret\n",
        addr, writeAddr
    );

    if (asmLen < 0 || asmLen >= (int)sizeof(asmBuf)) {
        Serial.printf("I2C scan address 0x%02X program truncated\n", addr);
        return -1;
    }

    assembler.assemble(asmBuf);
    const AsmResult& r = assembler.result;
    if (r.error_count != 0) return -1;

    memcpy(uploadBuf, r.code, CODE_SIZE);
    uploadSize = CODE_SIZE;
    uploadToMK1(uploadBuf, uploadSize);

    // RESET + RUN (same proven sequence as readDS3231Temp)
    disableClock();
    resetPulse();
    enableCU();

    stopCustomClock();
    detachOIMonitorForManualRun();
    outputCaptured = false;
    oiCount = 0;

    int oiGpio = digitalPinToGPIONumber(PIN_OI);
    if (oiGpio < 0) oiGpio = PIN_OI;
    uint32_t oiMask = 1 << oiGpio;

    busSetInput(); disableOutput();
    digitalWrite(PIN_DIR, LOW); enableOutput();

    int clkGpio = digitalPinToGPIONumber(PIN_CLK);
    uint32_t clkMask = 1 << clkGpio;
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);

    int result = -1;
    for (int i = 0; i < 20000; i++) {
        if ((i & 0x3FFF) == 0) yield();
        GPIO.out_w1ts = clkMask;
        delayMicroseconds(1);
        uint32_t g1 = GPIO.in;
        if (g1 & oiMask) {
            result = decodeBus(g1);
            GPIO.out_w1tc = clkMask;
            break;
        }
        GPIO.out_w1tc = clkMask;
        delayMicroseconds(1);
        uint32_t g2 = GPIO.in;
        if (g2 & oiMask) {
            result = decodeBus(g2);
            break;
        }
    }
    pinMode(PIN_CLK, INPUT);
    disableOutput(); digitalWrite(PIN_DIR, HIGH);
    busSetOutput(); enableOutput();

    return result;
}

static void handleI2CScan() {
    String json = "{\"found\":[";
    bool first = true;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        int r = runI2CScanAddr(addr);
        if (r == 1) {
            if (!first) json += ",";
            json += "{\"addr\":" + String(addr) + ",\"hex\":\"0x" +
                    String(addr, HEX) + "\",\"write\":\"0x" +
                    String(addr << 1, HEX) + "\",\"read\":\"0x" +
                    String((addr << 1) | 1, HEX) + "\"}";
            first = false;
        }
    }
    json += "]}";
    server.send(200, "application/json", json);
}

static void loadAsmResultForUpload(const AsmResult& r) {
    int dataBytes = r.data_size < DATA_SIZE ? r.data_size : DATA_SIZE;
    int stackBytes = r.stack_size < DATA_SIZE ? r.stack_size : DATA_SIZE;
    int p3Bytes = r.page3_size < DATA_SIZE ? r.page3_size : DATA_SIZE;

    memset(uploadBuf, 0, sizeof(uploadBuf));
    memcpy(uploadBuf, r.code, CODE_SIZE);
    uploadSize = CODE_SIZE;

    if (dataBytes > 0 || stackBytes > 0 || p3Bytes > 0) {
        memcpy(uploadBuf + CODE_SIZE, r.data, dataBytes);
        uploadSize = CODE_SIZE + DATA_SIZE;
    }
    if (stackBytes > 0 || p3Bytes > 0) {
        memcpy(uploadBuf + CODE_SIZE + DATA_SIZE, r.stack, stackBytes);
        uploadSize = CODE_SIZE + DATA_SIZE + DATA_SIZE;
    }
    if (p3Bytes > 0) {
        memcpy(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE, r.page3, p3Bytes);
        uploadSize = CODE_SIZE + DATA_SIZE + DATA_SIZE + DATA_SIZE;
    }
}

static int runUploadedUntilFirstOut(int maxCycles) {
    disableClock();
    resetPulse();
    enableCU();

    stopCustomClock();
    detachOIMonitorForManualRun();
    outputCaptured = false;
    oiCount = 0;

    int oiGpio = digitalPinToGPIONumber(PIN_OI);
    if (oiGpio < 0) oiGpio = PIN_OI;
    uint32_t oiMask = 1 << oiGpio;

    busSetInput();
    disableOutput();
    digitalWrite(PIN_DIR, LOW);
    enableOutput();

    int clkGpio = digitalPinToGPIONumber(PIN_CLK);
    uint32_t clkMask = 1 << clkGpio;
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);

    int result = -1;
    for (int i = 0; i < maxCycles; i++) {
        if ((i & 0x3FFF) == 0) yield();
        GPIO.out_w1ts = clkMask;
        delayMicroseconds(1);
        uint32_t g1 = GPIO.in;
        if (g1 & oiMask) {
            result = decodeBus(g1);
            GPIO.out_w1tc = clkMask;
            break;
        }
        GPIO.out_w1tc = clkMask;
        delayMicroseconds(1);
        uint32_t g2 = GPIO.in;
        if (g2 & oiMask) {
            result = decodeBus(g2);
            break;
        }
    }

    pinMode(PIN_CLK, INPUT);
    disableOutput();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();
    enableOutput();
    return result;
}

static bool runUploadedUntilHalt(int maxCycles) {
    disableClock();
    resetPulse();
    enableCU();

    stopCustomClock();
    detachOIMonitorForManualRun();

    busSetInput();
    disableOutput();
    digitalWrite(PIN_DIR, LOW);
    enableOutput();

    int clkGpio = digitalPinToGPIONumber(PIN_CLK);
    uint32_t clkMask = 1 << clkGpio;
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);

    bool halted = false;
    for (int i = 0; i < maxCycles; i++) {
        if ((i & 0x3FFF) == 0) yield();
        GPIO.out_w1ts = clkMask;
        delayMicroseconds(1);
        uint32_t g1 = GPIO.in;
        if (i > 4 && hltOpcodeObserved(g1)) {
            halted = true;
            GPIO.out_w1tc = clkMask;
            break;
        }
        GPIO.out_w1tc = clkMask;
        delayMicroseconds(1);
        uint32_t g2 = GPIO.in;
        if (i > 4 && hltOpcodeObserved(g2)) {
            halted = true;
            break;
        }
    }

    pinMode(PIN_CLK, INPUT);
    disableOutput();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();
    enableOutput();
    return halted;
}

// runUploadedCaptureOIs: manual-clock execution that captures every OI
// event into the supplied buffer and stops at first HLT. Returns the
// number of OI events captured (clamped to bufSize). Used by READ_CHIP
// where the shim emits the read bytes via `out` and then halts.
static int runUploadedCaptureOIs(int maxCycles, uint8_t* buf, int bufSize) {
    disableClock();
    resetPulse();
    enableCU();

    stopCustomClock();
    detachOIMonitorForManualRun();

    int oiGpio = digitalPinToGPIONumber(PIN_OI);
    if (oiGpio < 0) oiGpio = PIN_OI;
    uint32_t oiMask = 1 << oiGpio;

    busSetInput();
    disableOutput();
    digitalWrite(PIN_DIR, LOW);
    enableOutput();

    int clkGpio = digitalPinToGPIONumber(PIN_CLK);
    uint32_t clkMask = 1 << clkGpio;
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);

    int captured = 0;
    bool prevOi = false;     // edge-detect: only latch on rising edge
    for (int i = 0; i < maxCycles; i++) {
        if ((i & 0x3FFF) == 0) yield();
        GPIO.out_w1ts = clkMask;
        delayMicroseconds(1);
        uint32_t g1 = GPIO.in;
        bool oi1 = (g1 & oiMask) != 0;
        if (oi1 && !prevOi && captured < bufSize) {
            buf[captured++] = decodeBus(g1);
        }
        prevOi = oi1;
        if (i > 4 && hltOpcodeObserved(g1)) {
            GPIO.out_w1tc = clkMask;
            break;
        }
        GPIO.out_w1tc = clkMask;
        delayMicroseconds(1);
        uint32_t g2 = GPIO.in;
        bool oi2 = (g2 & oiMask) != 0;
        if (oi2 && !prevOi && captured < bufSize) {
            buf[captured++] = decodeBus(g2);
        }
        prevOi = oi2;
        if (i > 4 && hltOpcodeObserved(g2)) {
            break;
        }
    }

    pinMode(PIN_CLK, INPUT);
    disableOutput();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();
    enableOutput();
    return captured;
}

// ── Web handlers ─────────────────────────────────────────────────────

static void handleRoot() {
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.send(200, "text/html", WEB_PAGE);
}

static void handleAssemble() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"errors\":[{\"line\":0,\"message\":\"No source received\"}]}");
        return;
    }

    String source = server.arg("plain");
    assembler.assemble(source.c_str());

    const AsmResult& r = assembler.result;

    String json = "{";
    json += "\"code_size\":" + String(r.code_size) + ",";
    json += "\"data_size\":" + String(r.data_size) + ",";
    json += "\"errors\":[";
    for (int i = 0; i < r.error_count; i++) {
        if (i > 0) json += ",";
        json += "{\"line\":" + String(r.errors[i].line) + ",\"message\":\"";
        for (const char* p = r.errors[i].message; *p; p++) {
            if (*p == '"') json += "\\\"";
            else if (*p == '\\') json += "\\\\";
            else json += *p;
        }
        json += "\"}";
    }
    json += "]}";

    if (r.error_count == 0) {
        uploadSize = 0;
        int codeBytes = r.code_size < CODE_SIZE ? r.code_size : CODE_SIZE;
        int dataBytes = r.data_size < DATA_SIZE ? r.data_size : DATA_SIZE;

        int p3Bytes = r.page3_size < DATA_SIZE ? r.page3_size : DATA_SIZE;

        memcpy(uploadBuf, r.code, CODE_SIZE);  // copy full page (includes HLT fill)

        uploadSize = CODE_SIZE;

        if (dataBytes > 0 || p3Bytes > 0) {
            memcpy(uploadBuf + CODE_SIZE, r.data, dataBytes);
            memset(uploadBuf + CODE_SIZE + dataBytes, 0, DATA_SIZE - dataBytes);
            uploadSize = CODE_SIZE + DATA_SIZE;
        }

        if (p3Bytes > 0) {
            // Skip page 2 (stack), write page 3 at offset 768
            memset(uploadBuf + CODE_SIZE + DATA_SIZE, 0, DATA_SIZE);  // page 2 = zeros
            memcpy(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE, r.page3, p3Bytes);
            memset(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE + p3Bytes, 0, DATA_SIZE - p3Bytes);
            uploadSize = CODE_SIZE + DATA_SIZE + DATA_SIZE + DATA_SIZE;  // 1024
        }

        // Add hex dump of code bytes to response
        json.setCharAt(json.length() - 1, ',');  // replace closing } with ,
        json += "\"code_hex\":\"";
        for (int i = 0; i < codeBytes; i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X", r.code[i]);
            json += hex;
            if (i < codeBytes - 1) json += " ";
        }
        json += "\",\"data_hex\":\"";
        for (int i = 0; i < dataBytes; i++) {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X", r.data[i]);
            json += hex;
            if (i < dataBytes - 1) json += " ";
        }
        json += "\",\"src_len\":" + String(source.length());
        // Checksum of upload buffer for WiFi corruption detection
        uint32_t cksum = 0;
        for (int i = 0; i < uploadSize; i++) cksum += uploadBuf[i];
        json += ",\"cksum\":" + String(cksum);
        json += "}";
    }

    server.send(200, "application/json", json);
}

static void handleUpload() {
    if (uploadSize == 0) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"Nothing assembled\"}");
        return;
    }
    uploadToMK1(uploadBuf, uploadSize);
    server.send(200, "application/json", "{\"ok\":true}");
}

// Parse #clock directive from source. Returns 0 if not found.
static int parseClockDirective(const String& source) {
    int idx = source.indexOf("#clock");
    if (idx < 0) return 0;
    idx += 6;  // skip "#clock"
    while (idx < (int)source.length() && source[idx] == ' ') idx++;
    int hz = 0;
    while (idx < (int)source.length() && source[idx] >= '0' && source[idx] <= '9') {
        hz = hz * 10 + (source[idx] - '0');
        idx++;
    }
    return hz;
}

// Assemble + upload in one request. Accepts assembly text in POST body.
static void handleRun() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"No source\"}");
        return;
    }

    String source = server.arg("plain");
    int clockDirective = parseClockDirective(source);
    assembler.assemble(source.c_str());
    const AsmResult& r = assembler.result;

    if (r.error_count > 0) {
        String json = "{\"ok\":false,\"errors\":[";
        for (int i = 0; i < r.error_count; i++) {
            if (i > 0) json += ",";
            json += "{\"line\":" + String(r.errors[i].line) + ",\"message\":\"";
            for (const char* p = r.errors[i].message; *p; p++) {
                if (*p == '"') json += "\\\"";
                else if (*p == '\\') json += "\\\\";
                else json += *p;
            }
            json += "\"}";
        }
        json += "]}";
        server.send(400, "application/json", json);
        return;
    }

    uploadSize = 0;
    int codeBytes = r.code_size < CODE_SIZE ? r.code_size : CODE_SIZE;
    int dataBytes = r.data_size < DATA_SIZE ? r.data_size : DATA_SIZE;
    int p3Bytes = r.page3_size < DATA_SIZE ? r.page3_size : DATA_SIZE;

    memcpy(uploadBuf, r.code, CODE_SIZE);  // copy full page (includes HLT fill)
    uploadSize = CODE_SIZE;

    if (dataBytes > 0 || p3Bytes > 0) {
        memcpy(uploadBuf + CODE_SIZE, r.data, dataBytes);
        memset(uploadBuf + CODE_SIZE + dataBytes, 0, DATA_SIZE - dataBytes);
        uploadSize = CODE_SIZE + DATA_SIZE;
    }

    if (p3Bytes > 0) {
        memset(uploadBuf + CODE_SIZE + DATA_SIZE, 0, DATA_SIZE);
        memcpy(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE, r.page3, p3Bytes);
        memset(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE + p3Bytes, 0, DATA_SIZE - p3Bytes);
        uploadSize = CODE_SIZE + DATA_SIZE + DATA_SIZE + DATA_SIZE;
    }

    uploadToMK1(uploadBuf, uploadSize);
    autoSaveSource(source);

    // Apply #clock directive if present and ESP32 clock is active (SW3 manual)
    if (clockDirective > 0) {
        startCustomClock(clockDirective);
    }

    String json = "{\"ok\":true,\"code_size\":" + String(codeBytes) +
                  ",\"data_size\":" + String(dataBytes) + "}";
    server.send(200, "application/json", json);
}

static void handleHalt() {
    enableClock();  // hold CLK high = freeze
    cpuState = CPU_HALTED;
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleReset() {
    disableClock();
    resetPulse();
    enableCU();
    totalCycles = 0;
    cpuState = CPU_RUNNING;
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleStep() {
    // If running, halt first
    if (cpuState == CPU_RUNNING) {
        enableClock();
        stepNum = 0;
    }
    cpuState = CPU_STEPPING;

    // Execute one clock cycle, read bus
    uint8_t busVal = singleStep();

    // Track PC (step 0 = MI|PO, bus has PC) and opcode (step 1 = RO|II|PE, bus has opcode)
    if (stepNum == 0) stepPC = busVal;
    else if (stepNum == 1) stepOpcode = busVal;

    String json = "{\"ok\":true,\"bus\":" + String(busVal) +
                  ",\"cycles\":" + String(totalCycles) +
                  ",\"step\":" + String(stepNum) +
                  ",\"pc\":" + String(stepPC) +
                  ",\"opcode\":" + String(stepOpcode) + "}";

    stepNum = (stepNum + 1) & 7;  // wrap 0-7
    server.send(200, "application/json", json);
}

static void handleResume() {
    // Resume from halt/step to free-running
    disableClock();
    enableCU();
    cpuState = CPU_RUNNING;
    server.send(200, "application/json", "{\"ok\":true}");
}

// POST /clock?hz=1000 — set ESP32-generated clock speed (SW3 must be in manual)
// POST /clock?hz=0    — stop ESP32 clock, return to monitoring
// GET  /clock         — return current setting
static void handleClock() {
    if (server.hasArg("hz")) {
        int hz = server.arg("hz").toInt();
        if (hz <= 0) {
            stopCustomClock();
            server.send(200, "application/json", "{\"ok\":true,\"hz\":0,\"mode\":\"monitor\"}");
        } else {
            // Refuse if external clock is running (SW3 on auto)
            updateClkMeasurement();
            if (measuredClkHz > 10 && customClkHz == 0) {
                server.send(409, "application/json",
                    "{\"ok\":false,\"error\":\"External clock detected at " +
                    String(measuredClkHz, 0) + " Hz. Switch SW3 to manual first.\"}");
                return;
            }
            if (hz > 500000) hz = 500000;  // cap at 500kHz (board fails at ~600kHz)
            startCustomClock(hz);
            cpuState = CPU_RUNNING;
            server.send(200, "application/json",
                "{\"ok\":true,\"hz\":" + String(hz) + ",\"mode\":\"esp32_clock\"}");
        }
    } else {
        // GET: return current state
        String mode = customClkHz > 0 ? "esp32_clock" : "external";
        String json = "{\"hz\":" + String(customClkHz) +
                      ",\"mode\":\"" + mode +
                      "\",\"measured_hz\":" + String(measuredClkHz, 1) + "}";
        server.send(200, "application/json", json);
    }
}

static void handleStatus() {
    updateClkMeasurement();

    // Sample bus (only meaningful at low speeds or when halted)
    // Skip if OI monitor is active — it needs the bus in read mode undisturbed
    uint8_t busVal = 0;
    if (!oiMonitorActive) {
        busSetInput();
        disableOutput();
        busVal = busRead();
    }

    const char* stateStr = "running";
    if (cpuState == CPU_HALTED) stateStr = "halted";
    else if (cpuState == CPU_STEPPING) stateStr = "stepping";

    String json = "{\"state\":\"" + String(stateStr) +
                  "\",\"clock_hz\":" + String(measuredClkHz, 1) +
                  ",\"custom_clock_hz\":" + String(customClkHz) +
                  ",\"bus\":" + String(busVal) +
                  ",\"cycles\":" + String(totalCycles) + "}";
    server.send(200, "application/json", json);
}

// GET /readtest — debug: read addr 0 and addr 3 individually
static void handleReadTest() {
    stopClkMonitor();
    digitalWrite(PIN_DIR, HIGH);
    busSetOutput();

    // Read address 0
    resetPulse();
    delayMicroseconds(5);
    clockPulse();  // step 0: MAR=PC=0
    delayMicroseconds(2);
    // Don't overwrite MAR — just read what step 1 gives us
    uint8_t val0 = singleStep();  // step 1: bus = RAM[0]

    // Read address 3 — new approach:
    // 1. Reset (step=0), disable CU
    // 2. Set address (MAR=3) while CU disabled
    // 3. Clock once with CU disabled (step counter 0→1, no register effects)
    // 4. Re-enable CU (now at step 1 = RO|II|PE, MAR still holds 3)
    // 5. Clock once — RO asserts, bus = RAM[3]
    resetPulse();
    delayMicroseconds(5);
    disableCU();
    delayMicroseconds(2);
    setAddress(3);
    delayMicroseconds(2);
    clockPulse();  // advance step counter 0→1 with CU disabled (no side effects)
    delayMicroseconds(2);
    enableCU();
    delayMicroseconds(2);
    uint8_t val3 = singleStep();  // step 1: RO|II|PE, bus = RAM[3]

    resetPulse();
    startClkMonitor();

    String json = "{\"addr0\":" + String(val0) + ",\"addr3\":" + String(val3) +
                  ",\"expect0\":\"0xD1\",\"expect3\":\"0x63\"}";
    server.send(200, "application/json", json);
}

// GET /read?addr=0&count=256  — read RAM bytes, returns hex dump
// addr: 10-bit address (0x000-0x3FF covering all 4 pages)
// count: number of bytes (default 256, max 1024)
static void handleRead() {
    unsigned int addr = 0;
    unsigned int count = 256;
    if (server.hasArg("addr")) addr = server.arg("addr").toInt();
    if (server.hasArg("count")) count = server.arg("count").toInt();
    if (count > 1024) count = 1024;
    if (addr + count > 1024) count = 1024 - addr;

    uint8_t buf[1024];
    readMemory(addr, count, buf);

    String json = "{\"addr\":" + String(addr) + ",\"count\":" + String(count) + ",\"hex\":\"";
    for (unsigned int i = 0; i < count; i++) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X", buf[i]);
        json += hex;
        if (i < count - 1) json += " ";
    }
    json += "\",\"debug_pc\":" + String(buf[255 < count ? 255 : 0]) + "}";
    server.send(200, "application/json", json);
}

static const char* AUTOSAVE_PATH = "/last.asm";
static const char* LCD_COLOR_PATH = "/lcd_color.cfg";

// Global LCD color (default to white)
static uint8_t g_lcdR = 0xFF;
static uint8_t g_lcdG = 0xFF;
static uint8_t g_lcdB = 0xFF;
static const char* PROGRAMS_DIR = "/programs";

static void ensureProgramsDir() {
    if (!FFat.exists(PROGRAMS_DIR)) {
        FFat.mkdir(PROGRAMS_DIR);
    }
}

static void autoSaveSource(const String& source) {
    File f = FFat.open(AUTOSAVE_PATH, "w");
    if (f) { f.print(source); f.close(); }
}

// Save named program
static void handleSave() {
    if (!server.hasArg("name") || !server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"Need name and source\"}");
        return;
    }
    ensureProgramsDir();
    String name = server.arg("name");
    name.replace("/", "");  // sanitise
    name.replace("..", "");
    if (name.length() == 0 || name.length() > 32) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"Invalid name\"}");
        return;
    }
    String path = String(PROGRAMS_DIR) + "/" + name + ".asm";
    File f = FFat.open(path, "w");
    if (!f) {
        server.send(500, "application/json", "{\"ok\":false,\"error\":\"Flash write failed\"}");
        return;
    }
    f.print(server.arg("plain"));
    f.close();
    server.send(200, "application/json", "{\"ok\":true}");
}

// Load named program
static void handleLoad() {
    if (!server.hasArg("name")) {
        // Legacy: load autosaved program (for boot restore of editor)
        File f = FFat.open(AUTOSAVE_PATH, "r");
        if (!f) { server.send(200, "text/plain", ""); return; }
        String content = f.readString();
        f.close();
        server.send(200, "text/plain", content);
        return;
    }
    String name = server.arg("name");
    String path = String(PROGRAMS_DIR) + "/" + name + ".asm";
    File f = FFat.open(path, "r");
    if (!f) {
        server.send(404, "application/json", "{\"ok\":false,\"error\":\"Not found\"}");
        return;
    }
    String content = f.readString();
    f.close();
    server.send(200, "text/plain", content);
}

// List saved programs
static void handleProgramList() {
    ensureProgramsDir();
    String json = "[";
    File dir = FFat.open(PROGRAMS_DIR);
    bool first = true;
    if (dir && dir.isDirectory()) {
        File entry = dir.openNextFile();
        while (entry) {
            String fname = entry.name();
            // Strip path prefix and .asm extension
            int lastSlash = fname.lastIndexOf('/');
            if (lastSlash >= 0) fname = fname.substring(lastSlash + 1);
            if (fname.endsWith(".asm")) {
                fname = fname.substring(0, fname.length() - 4);
                if (!first) json += ",";
                json += "\"" + fname + "\"";
                first = false;
            }
            entry = dir.openNextFile();
        }
    }
    json += "]";
    server.send(200, "application/json", json);
}

// Delete named program
static void handleProgramDelete() {
    if (!server.hasArg("name")) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"Need name\"}");
        return;
    }
    String name = server.arg("name");
    String path = String(PROGRAMS_DIR) + "/" + name + ".asm";
    if (!FFat.exists(path)) {
        server.send(404, "application/json", "{\"ok\":false,\"error\":\"Not found\"}");
        return;
    }
    FFat.remove(path);
    server.send(200, "application/json", "{\"ok\":true}");
}

// ── WiFi configuration ───────────────────────────────────────────────

static void loadWifiConfig() {
    File f = FFat.open(WIFI_CONFIG_PATH, "r");
    if (!f) return;
    String ssid = f.readStringUntil('\n'); ssid.trim();
    String psk = f.readStringUntil('\n'); psk.trim();
    f.close();
    if (ssid.length() > 0) {
        strncpy(staSSID, ssid.c_str(), sizeof(staSSID) - 1);
        strncpy(staPSK, psk.c_str(), sizeof(staPSK) - 1);
    }
}

static void loadLcdColor() {
    File f = FFat.open(LCD_COLOR_PATH, "r");
    if (f) {
        g_lcdR = (uint8_t)f.read();
        g_lcdG = (uint8_t)f.read();
        g_lcdB = (uint8_t)f.read();
        f.close();
    }
}

static void saveLcdColor(uint8_t r, uint8_t g, uint8_t b) {
    File f = FFat.open(LCD_COLOR_PATH, "w");
    if (f) {
        f.write(r);
        f.write(g);
        f.write(b);
        f.close();
    }
}

static void handleWifiGet() {
    String json = "{\"mode\":\"" + String(staMode ? "sta" : "ap") + "\"";
    json += ",\"ssid\":\"" + String(staMode ? staSSID : AP_SSID) + "\"";
    if (staMode)
        json += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
    else
        json += ",\"ip\":\"" + WiFi.softAPIP().toString() + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

static void handleWifiPost() {
    String body = server.arg("plain");
    if (body.length() == 0) {
        // Try form args
        if (server.hasArg("ssid")) {
            body = "{\"ssid\":\"" + server.arg("ssid") + "\",\"psk\":\"" + server.arg("psk") + "\"}";
        } else {
            server.send(400, "application/json", "{\"ok\":false,\"error\":\"no body\"}");
            return;
        }
    }

    // Extract ssid and psk from JSON
    int si = body.indexOf("\"ssid\"");
    int pi = body.indexOf("\"psk\"");
    if (si < 0) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"no ssid\"}");
        return;
    }

    auto extractVal = [](const String& s, int start) -> String {
        int colon = s.indexOf(':', start);
        if (colon < 0) return "";
        int q1 = s.indexOf('"', colon + 1);
        if (q1 < 0) return "";
        int q2 = s.indexOf('"', q1 + 1);
        if (q2 < 0) return "";
        return s.substring(q1 + 1, q2);
    };

    String newSSID = extractVal(body, si);
    String newPSK = pi >= 0 ? extractVal(body, pi) : "";

    if (newSSID.length() == 0) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"empty ssid\"}");
        return;
    }

    File f = FFat.open(WIFI_CONFIG_PATH, "w");
    if (!f) {
        server.send(500, "application/json", "{\"ok\":false,\"error\":\"write failed\"}");
        return;
    }
    f.println(newSSID);
    f.println(newPSK);
    f.close();

    server.send(200, "application/json", "{\"ok\":true,\"ssid\":\"" + newSSID + "\",\"message\":\"Reset ESP32 to connect\"}");
}

static bool connectToWifi() {
    if (staSSID[0] == 0) return false;

    WiFi.mode(WIFI_STA);
    WiFi.begin(staSSID, staPSK);

    // Wait up to 30 seconds
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 60) {
        delay(500);
        tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        staMode = true;
        return true;
    }

    WiFi.disconnect();
    return false;
}

static void startAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID);
    staMode = false;
}

// ── Serial upload (legacy protocol) ──────────────────────────────────

static const uint8_t SERIAL_MAGIC[] = { 0x4D, 0x4B };

static String serialLineBuf;

static void handleSerialCommand(const String& line) {
    // Serial command protocol: "CMD:<command> [args]"
    // Responses are JSON, same as HTTP endpoints.
    if (line.startsWith("ASM:")) {
        // ASM:<assembly source> — assemble and prepare upload buffer
        String source = line.substring(4);
        source.replace("\\n", "\n");  // unescape newlines
        assembler.assemble(source.c_str());
        const AsmResult& r = assembler.result;
        if (r.error_count > 0) {
            Serial.printf("{\"ok\":false,\"errors\":%d,\"detail\":[", r.error_count);
            for (int i = 0; i < r.error_count && i < 5; i++) {
                if (i) Serial.print(',');
                Serial.printf("{\"line\":%d,\"msg\":\"", r.errors[i].line);
                for (const char* p = r.errors[i].message; *p; p++) {
                    if (*p == '"') Serial.print("\\\"");
                    else Serial.print(*p);
                }
                Serial.print("\"}");
            }
            Serial.println("]}");
            return;
        }
        int codeBytes = r.code_size < CODE_SIZE ? r.code_size : CODE_SIZE;
        int dataBytes = r.data_size < DATA_SIZE ? r.data_size : DATA_SIZE;
        int stkBytes = r.stack_size < DATA_SIZE ? r.stack_size : DATA_SIZE;
        int p3Bytes = r.page3_size < DATA_SIZE ? r.page3_size : DATA_SIZE;
        memcpy(uploadBuf, r.code, CODE_SIZE);
        uploadSize = CODE_SIZE;
        if (dataBytes > 0 || stkBytes > 0 || p3Bytes > 0) {
            memcpy(uploadBuf + CODE_SIZE, r.data, dataBytes);
            memset(uploadBuf + CODE_SIZE + dataBytes, 0, DATA_SIZE - dataBytes);
            uploadSize = CODE_SIZE + DATA_SIZE;
        }
        if (stkBytes > 0 || p3Bytes > 0) {
            // Page 2 (stack page): overlay data at bottom, zeros at top
            memcpy(uploadBuf + CODE_SIZE + DATA_SIZE, r.stack, stkBytes);
            memset(uploadBuf + CODE_SIZE + DATA_SIZE + stkBytes, 0, DATA_SIZE - stkBytes);
            // Page 3
            memcpy(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE, r.page3, p3Bytes);
            memset(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE + p3Bytes, 0, DATA_SIZE - p3Bytes);
            uploadSize = CODE_SIZE + DATA_SIZE + DATA_SIZE + DATA_SIZE;
        }
        uint32_t cksum = 0;
        for (int i = 0; i < uploadSize; i++) cksum += uploadBuf[i];
        Serial.printf("{\"ok\":true,\"code\":%d,\"data\":%d,\"eeprom\":%d,\"ee64\":%d,\"cksum\":%u,\"hex\":\"",
            codeBytes, dataBytes, r.eeprom_size, r.ee64_size, cksum);
        for (int i = 0; i < codeBytes && i < 16; i++) {
            if (i) Serial.print(' ');
            Serial.printf("%02X", r.code[i]);
        }
        Serial.println("\"}");
    }
    else if (line == "UPLOAD") {
        if (uploadSize == 0) { Serial.println("{\"ok\":false}"); return; }

        // Write EEPROM data if present (before uploading user program).
        // Both writeEepromData and writeEe64Data internally re-run the
        // assembler for I²C shim programs, which zeros result.*_size
        // fields (and the code buffer) but leaves the eeprom/ee64
        // buffer contents intact. Save sizes up front so we don't lose
        // track after the first write.
        const AsmResult& er = assembler.result;
        int eepromBytes = er.eeprom_size;
        int ee64Bytes = er.ee64_size;
        if (eepromBytes > 0) {
            writeEepromData(er.eeprom, eepromBytes);
        }
        if (ee64Bytes > 0) {
            writeEe64Data(er.ee64, ee64Bytes);
        }

        uploadToMK1(uploadBuf, uploadSize);

        // Hard rule: ESP32 has NO role post-upload. EEPROM-backed overlays
        // are now preloaded by MK1 itself during init (reads from AT24C32
        // via I2C, writes to page3). We only write the EEPROM during upload.
        if (eepromBytes > 0 || ee64Bytes > 0)
            Serial.printf("{\"ok\":true,\"eeprom\":%d,\"ee64\":%d}\n",
                          eepromBytes, ee64Bytes);
        else
            Serial.println("{\"ok\":true}");
    }
    else if (line.startsWith("RUN:")) {
        // RUN:cycles,us[,nops] — run N cycles at us half-period
        // If us=0 and nops specified: use tight NOP loop for sub-µs delay
        // Each NOP iteration ≈ 4ns at 240MHz. nops=200 ≈ 833ns ≈ 600kHz
        int comma1 = line.indexOf(',', 4);
        int comma2 = comma1 > 0 ? line.indexOf(',', comma1 + 1) : -1;
        int n = line.substring(4, comma1 > 0 ? comma1 : line.length()).toInt();
        int us = comma1 > 0 ? line.substring(comma1 + 1, comma2 > 0 ? comma2 : line.length()).toInt() : 1;
        int nops = comma2 > 0 ? line.substring(comma2 + 1).toInt() : 0;
        if (n == 0) n = 50000000;    // n=0 = run until OI (~3.4 min at 248kHz)
        else if (n < 1) n = 1;
        if (n > 100000000) n = 100000000;
        if (us < 0) us = 0;
        if (nops < 0) nops = 0;
        if (nops > 10000) nops = 10000;

        stopCustomClock();
        detachOIMonitorForManualRun();
        outputCaptured = false;
        oiCount = 0;

        int oiGpio = digitalPinToGPIONumber(PIN_OI);
        if (oiGpio < 0) oiGpio = PIN_OI;
        uint32_t oiMask = 1 << oiGpio;

        busSetInput();
        disableOutput();
        digitalWrite(PIN_DIR, LOW);
        enableOutput();

        int clkGpio = digitalPinToGPIONumber(PIN_CLK);
        uint32_t clkMask = 1 << clkGpio;
        pinMode(PIN_CLK, OUTPUT);
        digitalWrite(PIN_CLK, LOW);

        // Also monitor HLT. Require the sampled bus to decode as the HLT
        // opcode so transient highs during I/O/delay calibration do not
        // prematurely terminate the hardware trace.

        int actualCycles = 0;
        bool aborted = false;
        bool halted = false;
        unsigned long t0_us = micros();
        for (int i = 0; i < n; i++) {
            // Abort if new serial data arrives (every ~130K cycles ≈ 0.5s at 248kHz)
            if ((i & 0x1FFFF) == 0x1FFFF && Serial.available()) {
                aborted = true;
                break;
            }
            actualCycles++;
            GPIO.out_w1ts = clkMask;
            if (us > 0) delayMicroseconds(us);
            else if (nops > 0) { for (volatile int j = 0; j < nops; j++) __asm__ __volatile__("nop"); }
            uint32_t gpio1 = GPIO.in;
            if (gpio1 & oiMask) {
                uint8_t val = decodeBus(gpio1);
                oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                GPIO.out_w1tc = clkMask;
                break;
            }
            if (i > 4 && hltOpcodeObserved(gpio1)) { halted = true; GPIO.out_w1tc = clkMask; break; }
            GPIO.out_w1tc = clkMask;
            if (us > 0) delayMicroseconds(us);
            else if (nops > 0) { for (volatile int j = 0; j < nops; j++) __asm__ __volatile__("nop"); }
            uint32_t gpio2 = GPIO.in;
            if (gpio2 & oiMask) {
                uint8_t val = decodeBus(gpio2);
                oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                break;
            }
            if (i > 4 && hltOpcodeObserved(gpio2)) { halted = true; break; }
        }
        unsigned long elapsed_us = micros() - t0_us;
        pinMode(PIN_CLK, INPUT);
        disableOutput();
        digitalWrite(PIN_DIR, HIGH);
        busSetOutput();
        enableOutput();

        // Report result (with abort flag)
        float actual_khz = (actualCycles > 100 && elapsed_us > 100)
            ? (float)actualCycles / elapsed_us * 1000.0f : 0;
        Serial.printf("{\"cyc\":%d,\"val\":%d,\"cap\":%s,\"us\":%lu,\"khz\":%.1f%s%s}\n",
            actualCycles,
            oiCount > 0 ? oiHistAt(oiHistStored() - 1) : 0,
            outputCaptured ? "true" : "false",
            elapsed_us,
            actual_khz,
            aborted ? ",\"aborted\":true" : "",
            halted ? ",\"halted\":true" : "");
    }
    else if (line.startsWith("RUNNB:")) {
        // RUNNB:cycles,us[,maxoi] — run N cycles, capture OI events
        // Break early when maxoi events captured (0 = unlimited)
        int comma1 = line.indexOf(',', 6);
        int comma2 = comma1 > 0 ? line.indexOf(',', comma1 + 1) : -1;
        int n = line.substring(6, comma1 > 0 ? comma1 : line.length()).toInt();
        int us = comma1 > 0 ? line.substring(comma1 + 1, comma2 > 0 ? comma2 : line.length()).toInt() : 1;
        int maxoi = comma2 > 0 ? line.substring(comma2 + 1).toInt() : 0;
        if (n <= 0) n = 5000000;
        if (n > 100000000) n = 100000000;
        if (us < 0) us = 0;
        if (maxoi < 0) maxoi = 0;

        stopCustomClock();
        detachOIMonitorForManualRun();
        outputCaptured = false;
        oiCount = 0;

        int oiGpio = digitalPinToGPIONumber(PIN_OI);
        if (oiGpio < 0) oiGpio = PIN_OI;
        uint32_t oiMask = 1 << oiGpio;
        busSetInput(); disableOutput();
        digitalWrite(PIN_DIR, LOW); enableOutput();
        int clkGpio = digitalPinToGPIONumber(PIN_CLK);
        uint32_t clkMask = 1 << clkGpio;
        pinMode(PIN_CLK, OUTPUT);
        digitalWrite(PIN_CLK, LOW);

        int actualCycles = 0;
        bool aborted = false;
        // Monitor HLT with opcode-qualified fast GPIO reads; see RUN.
        bool halted = false;
        for (int i = 0; i < n; i++) {
            if ((i & 0x1FFFF) == 0x1FFFF) {
                if (Serial.available()) { aborted = true; break; }
                // Heartbeat every ~131K cycles
                if ((i & 0x7FFFF) == 0x7FFFF) {
                    Serial.printf("{\"hb\":%d,\"oi\":%d}\n", i, oiCount);
                }
            }
            actualCycles++;
            GPIO.out_w1ts = clkMask;
            if (us > 0) delayMicroseconds(us);
            uint32_t gpio1 = GPIO.in;
            if (gpio1 & oiMask) {
                uint8_t val = decodeBus(gpio1);
                oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                if (maxoi > 0 && (int)oiCount >= maxoi) { actualCycles++; break; }
            }
            if (i > 4 && hltOpcodeObserved(gpio1)) { halted = true; GPIO.out_w1tc = clkMask; break; }
            GPIO.out_w1tc = clkMask;
            if (us > 0) delayMicroseconds(us);
            uint32_t gpio2 = GPIO.in;
            if (gpio2 & oiMask) {
                uint8_t val = decodeBus(gpio2);
                oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                if (maxoi > 0 && (int)oiCount >= maxoi) break;
            }
            if (i > 4 && hltOpcodeObserved(gpio2)) { halted = true; break; }
        }
        pinMode(PIN_CLK, INPUT);
        disableOutput(); digitalWrite(PIN_DIR, HIGH);
        busSetOutput(); enableOutput();
        // Report: cycles + full history
        (void)halted;  // halted flag reported below
        Serial.printf("{\"cyc\":%d,\"cnt\":%d,\"halted\":%s,\"hist\":[",
            actualCycles, oiCount, halted ? "true" : "false");
        // Send the NEWEST 32 events in chronological order. We keep the
        // last 256 in the ring buffer; the tail (where the program's
        // real output emissions live) is what we want to report.
        // Also emit per-event timestamps (MK1 cycle count at capture
        // time) so callers can verify e.g. ms-delay accuracy by taking
        // cycle-count deltas between successive out() events.
        uint32_t stored = oiHistStored();
        uint32_t start = stored > 32 ? stored - 32 : 0;
        for (uint32_t i = start; i < stored; i++) {
            if (i > start) Serial.print(',');
            Serial.print(oiHistAt(i));
        }
        Serial.print("],\"ts\":[");
        for (uint32_t i = start; i < stored; i++) {
            if (i > start) Serial.print(',');
            Serial.print(oiTimeAt(i));
        }
        Serial.printf("]%s}\n", aborted ? ",\"aborted\":true" : "");
    }
    else if (line.startsWith("RUNHZ:")) {
        // RUNHZ:cycles,hz[,maxoi] — like RUNNB but with arbitrary
        // target frequency in Hz, computed via ESP32 cycle counter.
        // Firmware subtracts a measured ~530 ns/half overhead so the
        // actual rate is close to the requested one. Safe range is
        // roughly 50 kHz to 400 kHz; above ~430 kHz the firmware
        // spuriously detects HLT on bus noise.
        int comma1 = line.indexOf(',', 6);
        int comma2 = comma1 > 0 ? line.indexOf(',', comma1 + 1) : -1;
        int n = line.substring(6, comma1 > 0 ? comma1 : line.length()).toInt();
        int hz = comma1 > 0 ? line.substring(comma1 + 1, comma2 > 0 ? comma2 : line.length()).toInt() : 145000;
        int maxoi = comma2 > 0 ? line.substring(comma2 + 1).toInt() : 0;
        if (n <= 0) n = 5000000;
        if (n > 100000000) n = 100000000;
        if (hz < 1) hz = 1;
        if (hz > 1000000) hz = 1000000;   // firmware can't really exceed this
        if (maxoi < 0) maxoi = 0;

        // half_period_ns = 1e9 / (2 * hz). Then subtract ~530 ns of
        // firmware overhead per half (GPIO writes + OI/HLT checks)
        // so the wait we actually impose is just the *gap* needed
        // beyond what the firmware itself takes.
        uint32_t half_period_ns = 1000000000UL / (2UL * (uint32_t)hz);
        const uint32_t FW_OVERHEAD_NS = 530;
        uint32_t wait_ns = (half_period_ns > FW_OVERHEAD_NS)
            ? (half_period_ns - FW_OVERHEAD_NS) : 0;
        // ESP32 runs at 240 MHz → 1 CCOUNT cycle ≈ 4.167 ns.
        uint32_t target_cycles = (wait_ns * 240) / 1000;

        stopCustomClock();
        detachOIMonitorForManualRun();
        outputCaptured = false;
        oiCount = 0;

        int oiGpio = digitalPinToGPIONumber(PIN_OI);
        if (oiGpio < 0) oiGpio = PIN_OI;
        uint32_t oiMask = 1 << oiGpio;
        busSetInput(); disableOutput();
        digitalWrite(PIN_DIR, LOW); enableOutput();
        int clkGpio = digitalPinToGPIONumber(PIN_CLK);
        uint32_t clkMask = 1 << clkGpio;
        pinMode(PIN_CLK, OUTPUT);
        digitalWrite(PIN_CLK, LOW);

        int actualCycles = 0;
        bool aborted = false;
        bool halted = false;
        // Edge-detect OI: count one event per low→high transition,
        // not per sample where OI=1. Without this, each `out`
        // instruction registers twice (once when CLK is high, once
        // when CLK is low) and small `maxoi` budgets fire on
        // transitional bus glitches before the program completes.
        bool prevOI = false;
        unsigned long t0_us = micros();
        for (int i = 0; i < n; i++) {
            if ((i & 0x1FFFF) == 0x1FFFF) {
                if (Serial.available()) { aborted = true; break; }
                if ((i & 0x7FFFF) == 0x7FFFF) {
                    Serial.printf("{\"hb\":%d,\"oi\":%d}\n", i, oiCount);
                }
            }
            actualCycles++;
            GPIO.out_w1ts = clkMask;
            if (target_cycles > 0) {
                uint32_t start = ESP.getCycleCount();
                while ((ESP.getCycleCount() - start) < target_cycles) ;
            }
            uint32_t gpio1 = GPIO.in;
            bool curOI1 = (gpio1 & oiMask) != 0;
            if (curOI1 && !prevOI) {
                uint8_t val = decodeBus(gpio1);
                oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                if (maxoi > 0 && (int)oiCount >= maxoi) { prevOI = curOI1; actualCycles++; break; }
            }
            prevOI = curOI1;
            if (i > 4 && hltOpcodeObserved(gpio1)) { halted = true; GPIO.out_w1tc = clkMask; break; }
            GPIO.out_w1tc = clkMask;
            if (target_cycles > 0) {
                uint32_t start = ESP.getCycleCount();
                while ((ESP.getCycleCount() - start) < target_cycles) ;
            }
            uint32_t gpio2 = GPIO.in;
            bool curOI2 = (gpio2 & oiMask) != 0;
            if (curOI2 && !prevOI) {
                uint8_t val = decodeBus(gpio2);
                oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                if (maxoi > 0 && (int)oiCount >= maxoi) { prevOI = curOI2; break; }
            }
            prevOI = curOI2;
            if (i > 4 && hltOpcodeObserved(gpio2)) { halted = true; break; }
        }
        unsigned long elapsed_us = micros() - t0_us;
        pinMode(PIN_CLK, INPUT);
        disableOutput(); digitalWrite(PIN_DIR, HIGH);
        busSetOutput(); enableOutput();

        float actual_khz = (actualCycles > 100 && elapsed_us > 100)
            ? (float)actualCycles / elapsed_us * 1000.0f : 0;
        Serial.printf("{\"cyc\":%d,\"cnt\":%d,\"halted\":%s,\"khz\":%.1f,\"hist\":[",
            actualCycles, oiCount, halted ? "true" : "false", actual_khz);
        uint32_t stored = oiHistStored();
        uint32_t start_idx = (oiCount > 32) ? (stored - 32) : 0;
        for (uint32_t i = start_idx; i < stored; i++) {
            if (i > start_idx) Serial.print(',');
            Serial.print(oiHistAt(i));
        }
        Serial.print("],\"ts\":[");
        for (uint32_t i = start_idx; i < stored; i++) {
            if (i > start_idx) Serial.print(',');
            Serial.print(oiTimeAt(i));
        }
        Serial.printf("]%s}\n", aborted ? ",\"aborted\":true" : "");
    }
    else if (line == "OI") {
        // Report the NEWEST non-spurious value. That's the program's last
        // meaningful out() — typically the result you care about when
        // structuring a test as `out(result); halt()`.
        uint8_t val = 0;
        bool found = false;
        uint32_t n = oiHistStored();
        for (uint32_t i = n; i > 0; i--) {
            uint8_t v = oiHistAt(i - 1);
            if (v != 0x3F && v != 0x7F) { val = v; found = true; break; }
        }
        if (!found && n > 0) val = oiHistAt(n - 1);
        Serial.printf("{\"val\":%d,\"cap\":%s,\"cnt\":%d}\n",
            val, outputCaptured ? "true" : "false", oiCount);
    }
    else if (line == "STATUS") {
        Serial.printf("{\"state\":\"%s\"}\n",
            cpuState == CPU_HALTED ? "halted" : "running");
    }
    else if (line == "RESET") {
        disableClock();
        resetPulse();
        enableCU();
        Serial.println("{\"ok\":true}");
    }
    else if (line == "TEMP") {
        // Read DS3231 temperature and display on 7-seg
        int tempC = readDS3231Temp();
        if (tempC >= 0) {
            Serial.printf("{\"ok\":true,\"temp\":%d}\n", tempC);
        } else {
            Serial.println("{\"ok\":false,\"error\":\"temp read failed\"}");
        }
    }
    else if (line.startsWith("RTCREG")) {
        int reg = 0x0F;
        int sp = line.indexOf(' ');
        if (sp >= 0) reg = strtol(line.substring(sp + 1).c_str(), nullptr, 0);
        Serial.printf("RTCREG: reading 0x%02X\n", reg & 0xFF);
        Serial.flush();
        int val = readDS3231Reg((uint8_t)reg);
        Serial.printf("{\"ok\":%s,\"reg\":%d,\"val\":%d}\n",
            val >= 0 ? "true" : "false", reg & 0xFF, val);
    }
    else if (line == "BOOTLCD") {
        Serial.println("BOOTLCD: scan");
        Serial.flush();
        bool lcdFound = (runI2CScanAddr(0x3E) == 1);
        Serial.printf("BOOTLCD: lcd_found=%s\n", lcdFound ? "true" : "false");
        Serial.flush();
        int rtcStatus = -1;
        int rtcHour = -1;
        int rtcMin = -1;
        if (lcdFound) {
            Serial.println("BOOTLCD: rtc_status");
            Serial.flush();
            rtcStatus = readDS3231Reg(0x0F);
            if (rtcStatus >= 0 && (rtcStatus & 0x80) == 0) {
                Serial.println("BOOTLCD: rtc_time");
                Serial.flush();
                rtcHour = readDS3231Reg(0x02);
                rtcMin = readDS3231Reg(0x01);
            }
        }
        Serial.println("BOOTLCD: temp");
        Serial.flush();
        int tempC = readDS3231Temp();
        bool rtcSet = rtcStatus >= 0 && (rtcStatus & 0x80) == 0 &&
                      rtcHour >= 0 && rtcMin >= 0;
        bool lcdOk = false;
        if (tempC >= 0 && lcdFound) {
            Serial.println("BOOTLCD: lcd_update");
            Serial.flush();
            lcdOk = updateBootLCD(tempC, rtcSet, (uint8_t)rtcHour, (uint8_t)rtcMin);
            readDS3231Temp();  // restore visible out() after LCD completion marker
        }
        Serial.printf("{\"ok\":%s,\"lcd_found\":%s,\"lcd_ok\":%s,\"rtc_set\":%s,\"temp\":%d,\"rtc_status\":%d,\"hour\":%d,\"minute\":%d}\n",
            (tempC >= 0 && (!lcdFound || lcdOk)) ? "true" : "false",
            lcdFound ? "true" : "false",
            lcdOk ? "true" : "false",
            rtcSet ? "true" : "false",
            tempC, rtcStatus, rtcHour, rtcMin);
    }
    else if (line == "NTPSYNC") {
        // Force NTP resync + RTC set + temperature display
        if (!staMode) {
            Serial.println("{\"ok\":false,\"error\":\"not in STA mode\"}");
        } else {
            configTzTime("GMT0BST,M3.5.0/1,M10.5.0/2", "pool.ntp.org", "time.google.com");
            // Invalidate cached time to force re-sync
            struct timeval tv = {0, 0};
            settimeofday(&tv, NULL);
            Serial.print("NTP sync...");
            int ntpTries = 0;
            while (time(NULL) < 1000000000 && ntpTries < 20) {
                delay(500);
                ntpTries++;
            }
            if (time(NULL) > 1000000000) {
                struct tm t;
                getLocalTime(&t);
                Serial.printf(" OK: %04d-%02d-%02d %02d:%02d:%02d\n",
                    t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                    t.tm_hour, t.tm_min, t.tm_sec);
                // Set RTC — reuse the same assembly as boot
                auto toBCD = [](int v) -> uint8_t { return ((v / 10) << 4) | (v % 10); };
                char asmBuf[2048];
                snprintf(asmBuf, sizeof(asmBuf),
                    "; NTP -> DS3231 RTC sync\n"
                    "    ldi $d, 0\n"
                    ".dly:\n"
                    "    dec\n"
                    "    jnz .dly\n"
                    "    clr $a\n"
                    "    exw 0 0\n"
                    "    ddrb_imm 0x00\n"
                    "    clr $a\n"
                    "    exw 0 3\n"
                    "    push $a\n"
                    "    pop $a\n"
                    "    jal __i2c_st\n"
                    "    ldi $a, 0xD0\n"
                    "    jal __i2c_sb\n"
                    "    clr $a\n"
                    "    jal __i2c_sb\n"
                    "    ldi $a, 0x%02X\n"
                    "    jal __i2c_sb\n"
                    "    ldi $a, 0x%02X\n"
                    "    jal __i2c_sb\n"
                    "    ldi $a, 0x%02X\n"
                    "    jal __i2c_sb\n"
                    "    ldi $a, 0x%02X\n"
                    "    jal __i2c_sb\n"
                    "    ldi $a, 0x%02X\n"
                    "    jal __i2c_sb\n"
                    "    ldi $a, 0x%02X\n"
                    "    jal __i2c_sb\n"
                    "    ldi $a, 0x%02X\n"
                    "    jal __i2c_sb\n"
                    "    jal __i2c_sp\n"
                    "    out_imm 0xDD\n"
                    "    hlt\n"
                    "__i2c_st:\n"
                    "    exrw 2\n"
                    "    ddrb_imm 0x01\n"
                    "    ddrb_imm 0x03\n"
                    "    ret\n"
                    "__i2c_sp:\n"
                    "    ddrb_imm 0x03\n"
                    "    ddrb_imm 0x01\n"
                    "    ddrb_imm 0x00\n"
                    "    ret\n"
                    "__i2c_sb:\n"
                    "    mov $a, $b\n"
                    "    ldi $a, 8\n"
                    "    mov $a, $c\n"
                    ".isb:\n"
                    "    mov $b, $a\n"
                    "    tst 0x80\n"
                    "    jnz .isbh\n"
                    "    ddrb_imm 0x03\n"
                    "    ddrb_imm 0x01\n"
                    "    ddrb_imm 0x03\n"
                    "    j .isbn\n"
                    ".isbh:\n"
                    "    ddrb_imm 0x02\n"
                    "    ddrb_imm 0x00\n"
                    "    ddrb_imm 0x02\n"
                    ".isbn:\n"
                    "    mov $b, $a\n"
                    "    sll\n"
                    "    mov $a, $b\n"
                    "    mov $c, $a\n"
                    "    dec\n"
                    "    mov $a, $c\n"
                    "    jnz .isb\n"
                    "    ddrb_imm 0x02\n"
                    "    ddrb_imm 0x00\n"
                    "    exrw 0\n"
                    "    ddrb_imm 0x02\n"
                    "    ret\n",
                    toBCD(t.tm_sec), toBCD(t.tm_min), toBCD(t.tm_hour),
                    toBCD(t.tm_wday + 1), toBCD(t.tm_mday), toBCD(t.tm_mon + 1),
                    toBCD(t.tm_year % 100)
                );
                assembler.assemble(asmBuf);
                const AsmResult& r = assembler.result;
                if (r.error_count == 0) {
                    memcpy(uploadBuf, r.code, CODE_SIZE);
                    uploadSize = CODE_SIZE;
                    uploadToMK1(uploadBuf, uploadSize);
                    // Run RTC set
                    stopCustomClock();
                    detachOIMonitorForManualRun();
                    int clkGpio = digitalPinToGPIONumber(PIN_CLK);
                    uint32_t clkMask = 1 << clkGpio;
                    int oiGpio = digitalPinToGPIONumber(PIN_OI);
                    if (oiGpio < 0) oiGpio = PIN_OI;
                    uint32_t oiMask = 1 << oiGpio;
                    busSetInput(); disableOutput();
                    digitalWrite(PIN_DIR, LOW); enableOutput();
                    pinMode(PIN_CLK, OUTPUT); digitalWrite(PIN_CLK, LOW);
                    bool rtcOk = false;
                    for (int i = 0; i < 50000; i++) {
                        GPIO.out_w1ts = clkMask; delayMicroseconds(1);
                        uint32_t gA = GPIO.in;
                        if (gA & oiMask) { rtcOk = (decodeBus(gA) == 0xDD); GPIO.out_w1tc = clkMask; break; }
                        GPIO.out_w1tc = clkMask; delayMicroseconds(1);
                        uint32_t gB = GPIO.in;
                        if (gB & oiMask) { rtcOk = (decodeBus(gB) == 0xDD); break; }
                    }
                    pinMode(PIN_CLK, INPUT);
                    disableOutput(); digitalWrite(PIN_DIR, HIGH);
                    busSetOutput(); enableOutput();
                    Serial.printf("RTC set: %s\n", rtcOk ? "OK" : "FAIL");
                }
                // Now read and show temperature
                int tempC = readDS3231Temp();
                if (tempC >= 0) Serial.printf("{\"ok\":true,\"temp\":%d}\n", tempC);
                else Serial.println("{\"ok\":false,\"error\":\"temp read failed\"}");
            } else {
                Serial.println("{\"ok\":false,\"error\":\"NTP timeout\"}");
            }
        }
    }
    else if (line == "P3HEX") {
        // Dump first 80 bytes of page3 from uploadBuf (for debugging overlay data)
        int p3off = CODE_SIZE + DATA_SIZE + DATA_SIZE;  // 768
        int maxDump = 200;
        Serial.print("{\"p3\":[");
        for (int i = 0; i < maxDump && p3off + i < (int)uploadSize; i++) {
            if (i) Serial.print(',');
            Serial.print(uploadBuf[p3off + i]);
        }
        Serial.println("]}");
    }
    else if (line.startsWith("CLOCK:")) {
        int hz = line.substring(6).toInt();
        if (hz > 0) {
            startCustomClock(hz);
            Serial.printf("{\"ok\":true,\"hz\":%d}\n", hz);
        } else {
            stopCustomClock();
            Serial.println("{\"ok\":true,\"hz\":0}");
        }
    }
    else if (line == "OICNT") {
        Serial.println(oiCount);
    }
    else if (line.startsWith("DUMP:")) {
        // DUMP:offset,count — dump uploadBuf bytes as hex
        int comma = line.indexOf(',', 5);
        int off = line.substring(5, comma > 0 ? comma : line.length()).toInt();
        int cnt = comma > 0 ? line.substring(comma + 1).toInt() : 16;
        if (cnt > 64) cnt = 64;
        for (int i = 0; i < cnt && off + i < (int)sizeof(uploadBuf); i++) {
            if (i) Serial.print(' ');
            Serial.printf("%02X", uploadBuf[off + i]);
        }
        Serial.println();
    }
    else if (line.startsWith("DUMP_EE:")) {
        // DUMP_EE:offset,count — dump assembler.result.eeprom bytes as hex.
        // Diagnostic for emission-order bugs in the EEPROM section (e.g. an
        // overlay landing at eeprom[0xF0] instead of eeprom[0x100] because
        // the 16-byte header pad was skipped). Reads the assembler's eeprom
        // buffer, NOT the AT24C32 — what was *meant* to be written.
        int comma = line.indexOf(',', 8);
        int off = line.substring(8, comma > 0 ? comma : line.length()).toInt();
        int cnt = comma > 0 ? line.substring(comma + 1).toInt() : 16;
        if (cnt > 64) cnt = 64;
        const AsmResult& er = assembler.result;
        for (int i = 0; i < cnt && off + i < EEPROM_SIZE; i++) {
            if (i) Serial.print(' ');
            Serial.printf("%02X", er.eeprom[off + i]);
        }
        Serial.println();
    }
    else if (line.startsWith("DUMP_EE64:")) {
        // DUMP_EE64:offset,count — dump assembler.result.ee64 bytes as hex.
        // Same shape as DUMP_EE but for the AT24C512 cold-tier image.
        int comma = line.indexOf(',', 10);
        int off = line.substring(10, comma > 0 ? comma : line.length()).toInt();
        int cnt = comma > 0 ? line.substring(comma + 1).toInt() : 16;
        if (cnt > 64) cnt = 64;
        const AsmResult& er = assembler.result;
        for (int i = 0; i < cnt && off + i < EE64_SIZE; i++) {
            if (i) Serial.print(' ');
            Serial.printf("%02X", er.ee64[off + i]);
        }
        Serial.println();
    }
    else if (line.startsWith("READ_CHIP:")) {
        // READ_CHIP:offset,count — random-read `count` bytes directly from
        // AT24C512 at byte address `offset`, return as space-separated hex.
        // Bypasses the cold dispatcher; talks I²C via a generated MK1 shim.
        // Ground-truth probe of what's actually on the chip.
        //
        // SIDE EFFECT: overwrites whatever user program was last UPLOAD'd
        // (the read shim runs from page 0). Re-upload after inspection.
        int comma = line.indexOf(',', 10);
        int off = line.substring(10, comma > 0 ? comma : line.length()).toInt();
        int cnt = comma > 0 ? line.substring(comma + 1).toInt() : 16;
        if (cnt < 1) cnt = 1;
        if (cnt > 64) cnt = 64;
        if (off < 0) off = 0;
        if (off >= EE64_SIZE) off = EE64_SIZE - 1;
        uint8_t buf[64];
        int got = readAt24c512(off, cnt, buf);
        for (int i = 0; i < got; i++) {
            if (i) Serial.print(' ');
            Serial.printf("%02X", buf[i]);
        }
        Serial.println();
    }
    else if (line.startsWith("RUNLOG:")) {
        // RUNLOG:cycles,us[,halt[,maxoi]] — run cycles cycles, capture OI
        // events. Halt detection is optional (HLT bodge signal is noisy
        // enough that checking it in the hot path can change timing-
        // sensitive tests). `maxoi`: stop the loop early once N OI events
        // have been captured (0 = unlimited; 1+ avoids long sequences of
        // duplicate OI events from programs that halt+restart over the
        // cycle window).
        int comma = line.indexOf(',', 7);
        int comma2 = comma > 0 ? line.indexOf(',', comma + 1) : -1;
        int comma3 = comma2 > 0 ? line.indexOf(',', comma2 + 1) : -1;
        int n = line.substring(7, comma > 0 ? comma : line.length()).toInt();
        int us = comma > 0 ? line.substring(comma + 1, comma2 > 0 ? comma2 : line.length()).toInt() : 1;
        bool haltDetect = comma2 > 0 ? (line.substring(comma2 + 1, comma3 > 0 ? comma3 : line.length()).toInt() != 0) : false;
        int maxoi = comma3 > 0 ? line.substring(comma3 + 1).toInt() : 0;
        if (n < 1) n = 1;
        if (n > 100000000) n = 100000000;
        if (us < 0) us = 0;
        if (maxoi < 0) maxoi = 0;

        stopCustomClock();
        detachOIMonitorForManualRun();
        outputCaptured = false;
        oiCount = 0;

        int oiGpio = digitalPinToGPIONumber(PIN_OI);
        if (oiGpio < 0) oiGpio = PIN_OI;
        uint32_t oiMask = 1 << oiGpio;

        busSetInput();
        disableOutput();
        digitalWrite(PIN_DIR, LOW);
        enableOutput();

        int clkGpio = digitalPinToGPIONumber(PIN_CLK);
        uint32_t clkMask = 1 << clkGpio;
        pinMode(PIN_CLK, OUTPUT);
        digitalWrite(PIN_CLK, LOW);

        bool halted = false;

        int actualCycles = 0;
        unsigned long t0_us = micros();
        for (int i = 0; i < n; i++) {
            actualCycles++;
            GPIO.out_w1ts = clkMask;
            if (us > 0) delayMicroseconds(us);
            uint32_t gpio1 = GPIO.in;
            if (gpio1 & oiMask) {
                uint8_t val = decodeBus(gpio1);
                oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                if (maxoi > 0 && (int)oiCount >= maxoi) { GPIO.out_w1tc = clkMask; break; }
            }
            if (haltDetect && i > 4 && hltOpcodeObserved(gpio1)) {
                halted = true;
                GPIO.out_w1tc = clkMask;
                break;
            }
            GPIO.out_w1tc = clkMask;
            if (us > 0) delayMicroseconds(us);
            uint32_t gpio2 = GPIO.in;
            if (gpio2 & oiMask) {
                uint8_t val = decodeBus(gpio2);
                oiHistory[oiCount & OI_BUF_MASK] = val; oiTimes[oiCount & OI_BUF_MASK] = actualCycles;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                if (maxoi > 0 && (int)oiCount >= maxoi) break;
            }
            if (haltDetect && i > 4 && hltOpcodeObserved(gpio2)) {
                halted = true;
                break;
            }
        }
        unsigned long elapsed_us = micros() - t0_us;
        pinMode(PIN_CLK, INPUT);
        disableOutput();
        digitalWrite(PIN_DIR, HIGH);
        busSetOutput();
        enableOutput();

        float actual_khz = (actualCycles > 100 && elapsed_us > 100)
            ? (float)actualCycles / elapsed_us * 1000.0f : 0;
        // Output captured values in chronological order (oldest → newest).
        // With the ring buffer, when oiCount > 256 we've kept the LATEST 256,
        // i.e. the tail of the program. That's what callers want to see.
        Serial.printf("{\"cyc\":%d,\"cnt\":%d,\"khz\":%.1f,\"halted\":%s,\"vals\":[",
            actualCycles, oiCount, actual_khz, halted ? "true" : "false");
        uint32_t show = oiHistStored();
        for (uint32_t i = 0; i < show; i++) {
            if (i) Serial.print(',');
            Serial.print(oiHistAt(i));
        }
        // Per-event MK1-cycle timestamps (alongside `vals`). Delta between
        // successive timestamps gives the cycle count between two out()s.
        // Divide by the measured `khz` to convert to ms for delay
        // verification.
        Serial.print("],\"ts\":[");
        for (uint32_t i = 0; i < show; i++) {
            if (i) Serial.print(',');
            Serial.print(oiTimeAt(i));
        }
        Serial.println("]}");
    }
    else if (line == "HALT") {
        enableClock();  // hold CLK high = freeze
        cpuState = CPU_HALTED;
        Serial.println("{\"ok\":true}");
    }
    else {
        Serial.println("{\"err\":\"unknown\"}");
    }
}

static void handleSerialUpload() {
    // Check for text commands (lines ending with \n)
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialLineBuf.length() > 0) {
                handleSerialCommand(serialLineBuf);
                serialLineBuf = "";
            }
        } else if (serialLineBuf.length() < 16384) {
            serialLineBuf += c;
        }
    }
}

// ── EEPROM data write via MK1 I2C shim ──────────────────────────────

static void writeEepromData(const uint8_t* data, int size) {
    // Copy data to local buffer — the source points into assembler.result.eeprom
    // which gets zeroed when we assemble shim programs below.
    uint8_t localData[512];
    if (size > (int)sizeof(localData)) size = sizeof(localData);
    Serial.printf("EEPROM pre-copy: size=%d src[0]=%d src[16]=%d src[59]=%d\n",
        size, data[0], size>16?data[16]:0, size>59?data[59]:0);
    memcpy(localData, data, size);
    data = localData;
    Serial.printf("EEPROM post-copy: dst[0]=%d dst[16]=%d dst[59]=%d\n",
        localData[0], size>16?localData[16]:0, size>59?localData[59]:0);

    // Check if EEPROM already has the right data (checksum match).
    // Checksum is at EEPROM[0x0000-0x0001], data starts at 0x0010.
    // The data buffer includes the 16-byte header (checksum + reserved).
    if (size < 16) return;

    uint16_t expected_cksum = data[0] | (data[1] << 8);

    // Read current EEPROM checksum via a small MK1 program
    // that reads EEPROM[0x0000] and EEPROM[0x0001]
    // For simplicity, skip checksum verification for now and always write.
    // TODO: add checksum read + compare to skip redundant writes.

    // No serial output here — caller reports EEPROM info in JSON response

    // Write in 32-byte pages (AT24C32 page size).
    // Each page write: assemble a shim that writes up to 32 bytes,
    // upload, run, verify via OI output.
    for (int offset = 0; offset < size; offset += 32) {
        int pageLen = size - offset;
        if (pageLen > 32) pageLen = 32;

        int addrHi = (offset >> 8) & 0xFF;
        int addrLo = offset & 0xFF;

        // Build MK1 assembly for this page write
        char asmBuf[2048];
        int pos = 0;
        pos += snprintf(asmBuf + pos, sizeof(asmBuf) - pos,
            "; EEPROM page write at 0x%04X (%d bytes)\n"
            "    ldi $d, 0\n"
            ".dly:\n"
            "    dec\n"
            "    jnz .dly\n"
            "    clr $a\n"
            "    exw 0 0\n"
            "    ddrb_imm 0x00\n"
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x00\n"
            // ACK poll: wait for any prior write to complete
            ".poll:\n"
            "    exrw 2\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x03\n"
            "    ldi $a, 0xAE\n"
            "    jal __sb\n"
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x00\n"
            "    tst 0x01\n"
            "    jnz .poll\n"
            // START page write
            "    exrw 2\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x03\n"
            "    ldi $a, 0xAE\n"
            "    jal __sb\n"
            "    ldi $a, %d\n"    // addr high
            "    jal __sb\n"
            "    ldi $a, %d\n"   // addr low
            "    jal __sb\n",
            offset, pageLen, addrHi, addrLo);

        // Emit data bytes
        for (int i = 0; i < pageLen; i++) {
            pos += snprintf(asmBuf + pos, sizeof(asmBuf) - pos,
                "    ldi $a, %d\n"
                "    jal __sb\n",
                data[offset + i]);
        }
        pos += snprintf(asmBuf + pos, sizeof(asmBuf) - pos,
            // STOP
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x00\n"
            "    hlt\n"
            "__sb:\n"
            "    mov $a, $b\n"
            "    ldi $a, 8\n"
            "    mov $a, $c\n"
            ".isb:\n"
            "    mov $b, $a\n"
            "    tst 0x80\n"
            "    jnz .isbh\n"
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x03\n"
            "    j .isbn\n"
            ".isbh:\n"
            "    ddrb_imm 0x02\n"
            "    ddrb_imm 0x00\n"
            "    ddrb_imm 0x02\n"
            ".isbn:\n"
            "    mov $b, $a\n"
            "    sll\n"
            "    mov $a, $b\n"
            "    mov $c, $a\n"
            "    dec\n"
            "    mov $a, $c\n"
            "    jnz .isb\n"
            "    ddrb_imm 0x02\n"
            "    ddrb_imm 0x00\n"
            "    exrw 0\n"
            "    ddrb_imm 0x02\n"
            "    ret\n");

        assembler.assemble(asmBuf);
        const AsmResult& pr = assembler.result;
        if (pr.error_count != 0) {
            Serial.printf("EEPROM write ASM error at offset 0x%04X\n", offset);
            continue;
        }
        Serial.printf("EEPROM shim page 0x%04X: %dB code, data[0]=%d data[%d]=%d\n",
            offset, pr.code_size, data[offset], offset+pageLen-1, data[offset+pageLen-1]);

        // Upload shim without using global uploadBuf
        uint8_t shimBuf[256];
        memcpy(shimBuf, pr.code, CODE_SIZE);
        uploadToMK1(shimBuf, CODE_SIZE);

        // Run shim — replicate serial RUN handler exactly
        stopCustomClock();
        // Detach OI ISR only (don't call stopOIMonitor which reconfigures bus)
        detachOIMonitorForManualRun();

        busSetInput();
        disableOutput();
        digitalWrite(PIN_DIR, LOW);
        enableOutput();

        int clkGpio = digitalPinToGPIONumber(PIN_CLK);
        uint32_t clkMask = 1 << clkGpio;
        pinMode(PIN_CLK, OUTPUT);
        digitalWrite(PIN_CLK, LOW);

        for (int i = 0; i < 100000; i++) {
            GPIO.out_w1ts = clkMask;
            delayMicroseconds(1);
            GPIO.out_w1tc = clkMask;
            delayMicroseconds(1);
        }
        pinMode(PIN_CLK, INPUT);
        // Restore bus to ESP32-output mode for next uploadToMK1.
        // uploadToMK1's stopOIMonitor() will skip (already inactive),
        // so we must set the bus direction here.
        disableOutput();
        digitalWrite(PIN_DIR, HIGH);    // ESP32 → CPU direction
        busSetOutput();
        enableOutput();
        disableClock();
    }
}

// ── AT24C512 (cold tier) write via MK1 I²C shim ─────────────────────
//
// Mirrors writeEepromData but targets the 64KB AT24C512 at 0x50.
// Shim sends device byte 0xA0 (vs 0xAE), uses full 16-bit addressing
// (the AT24C32 shim already does too — top 4 bits were just ignored
// by the 12-bit-addressed chip). Page size: 32 B, well within the
// AT24C512's 128 B page boundary, and small enough that the asmBuf
// holds the inlined `ldi $a,N; jal __sb` pair per byte.
//
// Reads from result.ee64 (which persists across the shim re-assemble
// because pass() only resets the size counters, not the buffer
// contents). Caller is responsible for saving ee64_size before
// invoking this — see UPLOAD handler.

static void writeEe64Data(const uint8_t* data, int size) {
    // 1-byte page writes. AT24C512 supports up to 128 per write, but
    // multi-byte page writes on this board exhibit deterministic
    // byte mis-write at specific (offset, value) combinations even
    // with the protocol's per-byte ACK. Single-byte page writes
    // sidestep that — each byte is its own complete I²C transaction
    // (START + SLA+W + addr_hi + addr_lo + 1 byte + STOP). Slower
    // (~5 ms write cycle per byte vs 5 ms per ≤128-byte page), but
    // verifies byte-exact via the host's READ_CHIP retry.
    static const int PAGE_SZ = 1;
    // CRITICAL: snapshot `data` to a local mirror BEFORE the per-page
    // loop. Each iteration's `assembler.assemble(asmBuf)` calls
    // `memset(&result, 0, sizeof(result))` to clear stale state, and
    // `data` typically points into `result.ee64`. Without this copy,
    // iter 1's snprintf reads correct bytes, iter 1's assemble zeros
    // result.ee64, iter 2+'s snprintf reads zeros and writes zeros to
    // the chip — same bug `writeEepromData` already works around with
    // a 512 B stack buffer (it just happens to fit since AT24C32 is
    // 4 KB max). For the 64 KB AT24C512, can't put 64 KB on the stack;
    // use a static mirror instead.
    static uint8_t ee64Mirror[EE64_SIZE];
    if (size > EE64_SIZE) size = EE64_SIZE;
    memcpy(ee64Mirror, data, size);
    data = ee64Mirror;
    Serial.printf("EE64 write start: size=%d src[0]=%d src[%d]=%d\n",
        size, size > 0 ? data[0] : 0, size - 1, size > 0 ? data[size - 1] : 0);

    static char asmBuf[3072];   // shared across pages — saves stack

    for (int offset = 0; offset < size; offset += PAGE_SZ) {
        int pageLen = size - offset;
        if (pageLen > PAGE_SZ) pageLen = PAGE_SZ;

        int addrHi = (offset >> 8) & 0xFF;
        int addrLo = offset & 0xFF;

        int pos = 0;
        pos += snprintf(asmBuf + pos, sizeof(asmBuf) - pos,
            "; EE64 page write at 0x%04X (%d bytes)\n"
            "    ldi $d, 0\n"
            ".dly:\n"
            "    dec\n"
            "    jnz .dly\n"
            "    clr $a\n"
            "    exw 0 0\n"
            "    ddrb_imm 0x00\n"
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x00\n"
            // Retry counter: $c = 4 attempts. Per-byte ACK is checked
            // after every jal __sb; any NACK aborts to .nack which
            // sends STOP, decrements $c, and re-runs the whole
            // transaction. Without this, a single data-byte NACK
            // would silently lose the rest of the page (every
            // subsequent send is ignored by the chip until STOP).
            "    ldi $c, 4\n"
            ".attempt:\n"
            // ACK poll: wait for any prior write cycle to complete.
            ".poll:\n"
            "    exrw 2\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x03\n"
            "    ldi $a, 0xA0\n"
            "    jal __sb\n"
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x00\n"
            "    tst 0x01\n"
            "    jnz .poll\n"
            // START + dev write + addr_hi + addr_lo (with per-byte ACK).
            "    exrw 2\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x03\n"
            "    ldi $a, 0xA0\n"
            "    jal __sb\n"
            "    tst 0x01\n"
            "    jnz .nack\n"
            "    ldi $a, %d\n"
            "    jal __sb\n"
            "    tst 0x01\n"
            "    jnz .nack\n"
            "    ldi $a, %d\n"
            "    jal __sb\n"
            "    tst 0x01\n"
            "    jnz .nack\n",
            offset, pageLen, addrHi, addrLo);

        for (int i = 0; i < pageLen; i++) {
            pos += snprintf(asmBuf + pos, sizeof(asmBuf) - pos,
                "    ldi $a, %d\n"
                "    jal __sb\n"
                "    tst 0x01\n"
                "    jnz .nack\n",
                data[offset + i]);
        }
        pos += snprintf(asmBuf + pos, sizeof(asmBuf) - pos,
            // All bytes ACKed — STOP and halt.
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x00\n"
            "    hlt\n"
            // NACK handler: STOP, decrement retry, re-attempt or give up.
            ".nack:\n"
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x00\n"
            "    decc\n"
            "    jnz .attempt\n"
            // All retries exhausted — STOP again and halt. ESP32 has no
            // way to tell write failed (no return path), so we just
            // give up gracefully rather than spinning forever.
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x00\n"
            "    hlt\n"
            // __sb mirrors the kernel's `__i2c_sb` (mk1cc2.py line ~3579)
            // byte-for-byte: byte arrives in A, gets stashed in D, B holds
            // an 8-cycle counter, decb decrements in place each iter,
            // single `exrw 0` reads ACK at end. This is the same routine
            // the C-level `i2c_send_byte` builtin uses, which has been
            // exercised end-to-end against AT24C32 AND AT24C512 in the
            // hw_regression corpus and the Phase 1/2 tests. The previous
            // shim-local __sb (different register convention, dec+mov
            // dance for counter) was AT24C32-tolerant but AT24C512 lost
            // bytes mid-page-write. Mirroring the kernel routine exactly
            // removes the divergence.
            "__sb:\n"
            "    mov $a, $d\n"      // D = byte (survives ldi A clobber)
            "    ldi $b, 8\n"        // B = counter
            ".isb:\n"
            "    mov $d, $a\n"      // A = byte (reload for tst)
            "    tst 0x80\n"
            "    jnz .isbh\n"
            "    ddrb_imm 0x03\n"
            "    ddrb_imm 0x01\n"
            "    ddrb_imm 0x03\n"
            "    j .isbn\n"
            ".isbh:\n"
            "    ddrb_imm 0x02\n"
            "    ddrb_imm 0x00\n"
            "    ddrb_imm 0x02\n"
            ".isbn:\n"
            "    sll\n"              // A = byte << 1
            "    mov $a, $d\n"      // D = shifted byte for next iter
            "    decb\n"             // B--, sets ZF
            "    jnz .isb\n"
            // ACK clock — single exrw 0 matches the kernel routine
            "    ddrb_imm 0x02\n"
            "    ddrb_imm 0x00\n"
            "    exrw 0\n"
            "    ddrb_imm 0x02\n"
            "    ret\n");

        assembler.assemble(asmBuf);
        const AsmResult& pr = assembler.result;
        if (pr.error_count != 0) {
            Serial.printf("EE64 write ASM error at offset 0x%04X\n", offset);
            continue;
        }
        Serial.printf("EE64 shim page 0x%04X: %dB code\n", offset, pr.code_size);

        uint8_t shimBuf[256];
        memcpy(shimBuf, pr.code, CODE_SIZE);
        uploadToMK1(shimBuf, CODE_SIZE);

        stopCustomClock();
        detachOIMonitorForManualRun();

        busSetInput();
        disableOutput();
        digitalWrite(PIN_DIR, LOW);
        enableOutput();

        int clkGpio = digitalPinToGPIONumber(PIN_CLK);
        uint32_t clkMask = 1 << clkGpio;
        pinMode(PIN_CLK, OUTPUT);
        digitalWrite(PIN_CLK, LOW);

        for (int i = 0; i < 100000; i++) {
            if ((i & 0x3FFF) == 0) yield();   // feed task watchdog
            GPIO.out_w1ts = clkMask;
            delayMicroseconds(1);
            GPIO.out_w1tc = clkMask;
            delayMicroseconds(1);
        }
        pinMode(PIN_CLK, INPUT);
        disableOutput();
        digitalWrite(PIN_DIR, HIGH);
        busSetOutput();
        enableOutput();
        disableClock();

    }
}

// ── DS3231 temperature → 7-seg ───────────────────────────────────────

static int readDS3231Temp() {
    // Assemble and run a MK1 program that reads DS3231 register 0x11
    // (temperature integer °C) and outputs it to the 7-seg display.
    // Returns temperature value, or -1 on failure.
    const char* tempAsm =
        "; DS3231 temperature read (auto-generated)\n"
        "    ldi $d, 0\n"
        ".dly:\n"
        "    dec\n"
        "    jnz .dly\n"
        "    clr $a\n"
        "    exw 0 0\n"
        "    ddrb_imm 0x00\n"
        "    clr $a\n"
        "    exw 0 3\n"
        // Clean STOP to end any prior transaction
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        // Read temperature from DS3231 register 0x11
        "    jal __i2c_st\n"
        "    ldi $a, 0xD0\n"
        "    jal __i2c_sb\n"
        "    ldi $a, 0x11\n"
        "    jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        "    jal __i2c_st\n"
        "    ldi $a, 0xD1\n"
        "    jal __i2c_sb\n"
        "    jal __i2c_rb\n"
        "    ddrb_imm 0x00\n"
        "    ddrb_imm 0x02\n"
        "    jal __i2c_sp\n"
        "    mov $d, $a\n"
        "    out\n"
        "    hlt\n"
        "__i2c_st:\n"
        "    exrw 2\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    ret\n"
        "__i2c_sp:\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        "    ret\n"
        "__i2c_sb:\n"
        "    mov $a, $b\n"
        "    ldi $a, 8\n"
        "    mov $a, $c\n"
        ".isb:\n"
        "    mov $b, $a\n"
        "    tst 0x80\n"
        "    jnz .isbh\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    j .isbn\n"
        ".isbh:\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    ddrb_imm 0x02\n"
        ".isbn:\n"
        "    mov $b, $a\n"
        "    sll\n"
        "    mov $a, $b\n"
        "    mov $c, $a\n"
        "    dec\n"
        "    mov $a, $c\n"
        "    jnz .isb\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    exrw 0\n"
        "    ddrb_imm 0x02\n"
        "    ret\n"
        "__i2c_rb:\n"
        "    ldi $b, 0\n"
        "    ldi $d, 8\n"
        ".rb:\n"
        "    mov $b, $a\n"
        "    sll\n"
        "    mov $a, $b\n"
        "    ddrb_imm 0x00\n"
        "    nop\n"
        "    exrw 0\n"
        "    tst 0x01\n"
        "    jz .rz\n"
        "    mov $b, $a\n"
        "    ori 0x01, $a\n"
        "    mov $a, $b\n"
        ".rz:\n"
        "    ddrb_imm 0x02\n"
        "    mov $d, $a\n"
        "    dec\n"
        "    mov $a, $d\n"
        "    jnz .rb\n"
        "    mov $b, $d\n"
        "    ret\n";

    assembler.assemble(tempAsm);
    const AsmResult& rt = assembler.result;
    if (rt.error_count != 0) {
        Serial.println("Temperature program assembly failed");
        return -1;
    }

    memcpy(uploadBuf, rt.code, CODE_SIZE);
    uploadSize = CODE_SIZE;

    // Use the exact same sequence as the serial UPLOAD → RESET → RUN path
    uploadToMK1(uploadBuf, uploadSize);

    // RESET (same as serial RESET handler)
    disableClock();
    resetPulse();
    enableCU();

    // RUN (same as serial RUN handler — stop on first OI)
    stopCustomClock();
    detachOIMonitorForManualRun();
    outputCaptured = false;
    oiCount = 0;

    int oiGpio = digitalPinToGPIONumber(PIN_OI);
    if (oiGpio < 0) oiGpio = PIN_OI;
    uint32_t oiMask = 1 << oiGpio;

    busSetInput();
    disableOutput();
    digitalWrite(PIN_DIR, LOW);
    enableOutput();

    int clkGpio = digitalPinToGPIONumber(PIN_CLK);
    uint32_t clkMask = 1 << clkGpio;
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);

    int tempVal = -1;
    for (int i = 0; i < 50000; i++) {
        if ((i & 0x3FFF) == 0) yield();
        GPIO.out_w1ts = clkMask;
        delayMicroseconds(1);
        uint32_t gpio1 = GPIO.in;
        if (gpio1 & oiMask) {
            tempVal = readBusFast();
            GPIO.out_w1tc = clkMask;
            break;
        }
        GPIO.out_w1tc = clkMask;
        delayMicroseconds(1);
        uint32_t gpio2 = GPIO.in;
        if (gpio2 & oiMask) {
            tempVal = readBusFast();
            break;
        }
    }
    pinMode(PIN_CLK, INPUT);
    disableOutput(); digitalWrite(PIN_DIR, HIGH);
    busSetOutput(); enableOutput();

    return tempVal;
}

static int readDS3231Reg(uint8_t reg) {
    char asmBuf[4096];
    int asmLen = snprintf(asmBuf, sizeof(asmBuf),
        "; DS3231 register read 0x%02X (auto-generated)\n"
        "    ldi $d, 0\n"
        ".dly:\n"
        "    dec\n"
        "    jnz .dly\n"
        "    clr $a\n"
        "    exw 0 0\n"
        "    ddrb_imm 0x00\n"
        "    clr $a\n"
        "    exw 0 3\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        "    jal __i2c_st\n"
        "    ldi $a, 0xD0\n"
        "    jal __i2c_sb\n"
        "    ldi $a, 0x%02X\n"
        "    jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        "    jal __i2c_st\n"
        "    ldi $a, 0xD1\n"
        "    jal __i2c_sb\n"
        "    jal __i2c_rb\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        "    mov $d, $a\n"
        "    out\n"
        "    hlt\n"
        "__i2c_st:\n"
        "    exrw 2\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    ret\n"
        "__i2c_sp:\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        "    ret\n"
        "__i2c_sb:\n"
        "    mov $a, $b\n"
        "    ldi $a, 8\n"
        "    mov $a, $c\n"
        ".isb:\n"
        "    mov $b, $a\n"
        "    tst 0x80\n"
        "    jnz .isbh\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    j .isbn\n"
        ".isbh:\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    ddrb_imm 0x02\n"
        ".isbn:\n"
        "    mov $b, $a\n"
        "    sll\n"
        "    mov $a, $b\n"
        "    mov $c, $a\n"
        "    dec\n"
        "    mov $a, $c\n"
        "    jnz .isb\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    exrw 0\n"
        "    ddrb_imm 0x02\n"
        "    ret\n"
        "__i2c_rb:\n"
        "    ldi $b, 0\n"
        "    ldi $d, 8\n"
        ".rb:\n"
        "    mov $b, $a\n"
        "    sll\n"
        "    mov $a, $b\n"
        "    ddrb_imm 0x00\n"
        "    exrw 0\n"
        "    tst 0x01\n"
        "    jz .rz\n"
        "    mov $b, $a\n"
        "    ori 0x01, $a\n"
        "    mov $a, $b\n"
        ".rz:\n"
        "    ddrb_imm 0x02\n"
        "    mov $d, $a\n"
        "    dec\n"
        "    mov $a, $d\n"
        "    jnz .rb\n"
        "    mov $b, $d\n"
        "    ret\n",
        reg, reg
    );

    if (asmLen < 0 || asmLen >= (int)sizeof(asmBuf)) {
        Serial.printf("RTC register 0x%02X program truncated\n", reg);
        return -1;
    }

    assembler.assemble(asmBuf);
    const AsmResult& r = assembler.result;
    if (r.error_count != 0) {
        Serial.printf("RTC register 0x%02X program assembly failed\n", reg);
        return -1;
    }
    if (r.code_size > CODE_SIZE || r.data_size > DATA_SIZE ||
        r.stack_size > DATA_SIZE || r.page3_size > DATA_SIZE) {
        Serial.printf("RTC register 0x%02X program too large: code=%d data=%d stack=%d page3=%d\n",
            reg, r.code_size, r.data_size, r.stack_size, r.page3_size);
        return -1;
    }

    loadAsmResultForUpload(r);
    uploadToMK1(uploadBuf, uploadSize);
    return runUploadedUntilFirstOut(50000);
}

// readAt24c512: random-read `count` bytes from chip[offset..offset+count-1]
// directly via a generated MK1 shim. Returns number of bytes read into `out`
// (clamped to count). Used by the READ_CHIP debug command — does NOT go
// through the cold dispatcher, so it's a ground-truth probe of what's
// actually on the AT24C512.
//
// Side effect: overwrites the user program's uploadBuf (the shim runs from
// page 0). Caller is responsible for re-uploading the user program after
// inspecting the chip.
static int readAt24c512(int offset, int count, uint8_t* out) {
    if (count <= 0) return 0;
    if (count > 64) count = 64;     // hard cap (matches OI capture practical size)
    static char asmBuf[3072];   // static to avoid stack pressure when called
                                // from inside writeEe64Data's verify loop
    int pos = snprintf(asmBuf, sizeof(asmBuf),
        "; READ_CHIP offset=0x%04X count=%d (auto-generated)\n"
        "    ldi $d, 0\n"
        ".dly:\n"
        "    dec\n"
        "    jnz .dly\n"
        "    clr $a\n"
        "    exw 0 0\n"
        "    ddrb_imm 0x00\n"
        "    clr $a\n"
        "    exw 0 3\n"
        // Random-read setup: START + 0xA0 + addr_hi + addr_lo
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        "    jal __i2c_st\n"
        "    ldi $a, 0xA0\n"
        "    jal __i2c_sb\n"
        "    ldi $a, %d\n"     // addr_hi
        "    jal __i2c_sb\n"
        "    ldi $a, %d\n"     // addr_lo
        "    jal __i2c_sb\n"
        // Repeated start + read mode addr
        "    ddrb_imm 0x00\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    ldi $a, 0xA1\n"
        "    jal __i2c_sb\n"
        "    ldi $c, %d\n",    // count
        offset, count, (offset >> 8) & 0xFF, offset & 0xFF, count);
    // Per-byte read loop. After each `out`, the bus needs an ACK/NACK
    // clock for the next byte (or final NACK to terminate). Last byte
    // gets a NACK; every prior byte gets an ACK.
    pos += snprintf(asmBuf + pos, sizeof(asmBuf) - pos,
        ".rloop:\n"
        "    jal __i2c_rb\n"
        "    out\n"
        "    decc\n"
        "    jz .rdone\n"
        // ACK clock for next byte (master pulls SDA low during 9th tick)
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x02\n"
        "    j .rloop\n"
        ".rdone:\n"
        // NACK + STOP
        "    ddrb_imm 0x00\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x00\n"
        "    hlt\n");
    // Helpers — same byte-for-byte routines as the kernel/firmware shims.
    pos += snprintf(asmBuf + pos, sizeof(asmBuf) - pos,
        "__i2c_st:\n"
        "    exrw 2\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    ret\n"
        "__i2c_sb:\n"
        "    mov $a, $d\n"
        "    ldi $b, 8\n"
        ".isb:\n"
        "    mov $d, $a\n"
        "    tst 0x80\n"
        "    jnz .isbh\n"
        "    ddrb_imm 0x03\n"
        "    ddrb_imm 0x01\n"
        "    ddrb_imm 0x03\n"
        "    j .isbn\n"
        ".isbh:\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    ddrb_imm 0x02\n"
        ".isbn:\n"
        "    sll\n"
        "    mov $a, $d\n"
        "    decb\n"
        "    jnz .isb\n"
        "    ddrb_imm 0x02\n"
        "    ddrb_imm 0x00\n"
        "    exrw 0\n"
        "    ddrb_imm 0x02\n"
        "    ret\n"
        "__i2c_rb:\n"
        "    ldi $b, 0\n"
        "    ldi $d, 8\n"
        ".rb:\n"
        "    sllb\n"
        "    ddrb_imm 0x00\n"
        "    exrw 0\n"
        "    exrw 0\n"
        "    tst 0x01\n"
        "    jz .rz\n"
        "    incb\n"
        ".rz:\n"
        "    ddrb_imm 0x02\n"
        "    decd\n"
        "    jnz .rb\n"
        "    mov $b, $a\n"
        "    ret\n");

    if (pos < 0 || pos >= (int)sizeof(asmBuf)) {
        Serial.println("READ_CHIP shim truncated");
        return 0;
    }

    assembler.assemble(asmBuf);
    const AsmResult& r = assembler.result;
    if (r.error_count != 0) {
        Serial.println("READ_CHIP shim assembly failed");
        return 0;
    }

    loadAsmResultForUpload(r);
    uploadToMK1(uploadBuf, uploadSize);

    // Allow plenty of cycles: per byte ~ 200 cycles for I²C bit-bang +
    // overhead; 64 bytes × 200 ≈ 12 800. 50 000 is generous.
    return runUploadedCaptureOIs(50000, out, count);
}

static bool bcdByteValid(uint8_t v, uint8_t maxVal) {
    uint8_t tens = (v >> 4) & 0x0F;
    uint8_t ones = v & 0x0F;
    if (tens > 9 || ones > 9) return false;
    return (uint8_t)(tens * 10 + ones) <= maxVal;
}

// Boot LCD update is decomposed into a sequence of small, self-contained MK1
// programs ("snippets"). Each snippet sets up the VIA, performs a few I2C
// transactions to the DFRobot RGB V2.0 module, then emits 0xDD and halts so
// the ESP32 has a per-step ack. This avoids the overlay/dual-section regime
// that the compiler uses to pack large LCD programs into 256 bytes — the
// boot path has no competing user code, so each snippet fits flat in page 0.
//
// The I2C primitives below mirror the proven sequence in readDS3231Temp().

#define BOOT_LCD_I2C_HELPERS_ASM \
    "__i2c_st:\n" \
    "    exrw 2\n" \
    "    ddrb_imm 0x01\n" \
    "    ddrb_imm 0x03\n" \
    "    ret\n" \
    "__i2c_sp:\n" \
    "    ddrb_imm 0x03\n" \
    "    ddrb_imm 0x01\n" \
    "    ddrb_imm 0x00\n" \
    "    ret\n" \
    "__i2c_sb:\n" \
    "    mov $a, $b\n" \
    "    ldi $a, 8\n" \
    "    mov $a, $c\n" \
    ".isb:\n" \
    "    mov $b, $a\n" \
    "    tst 0x80\n" \
    "    jnz .isbh\n" \
    "    ddrb_imm 0x03\n" \
    "    ddrb_imm 0x01\n" \
    "    ddrb_imm 0x03\n" \
    "    j .isbn\n" \
    ".isbh:\n" \
    "    ddrb_imm 0x02\n" \
    "    ddrb_imm 0x00\n" \
    "    ddrb_imm 0x02\n" \
    ".isbn:\n" \
    "    mov $b, $a\n" \
    "    sll\n" \
    "    mov $a, $b\n" \
    "    mov $c, $a\n" \
    "    dec\n" \
    "    mov $a, $c\n" \
    "    jnz .isb\n" \
    "    ddrb_imm 0x02\n" \
    "    ddrb_imm 0x00\n" \
    "    exrw 0\n" \
    "    ddrb_imm 0x02\n" \
    "    ret\n"

#define BOOT_LCD_VIA_PROLOGUE_ASM \
    "    ldi $d, 0\n" \
    ".dly:\n" \
    "    dec\n" \
    "    jnz .dly\n" \
    "    clr $a\n" \
    "    exw 0 0\n" \
    "    ddrb_imm 0x00\n" \
    "    clr $a\n" \
    "    exw 0 3\n" \
    "    ddrb_imm 0x03\n" \
    "    ddrb_imm 0x01\n" \
    "    ddrb_imm 0x00\n"

// Generic snippet runner: returns the first out() value (0..255) or -1 on
// assembly/upload/timeout failure. Use this when the snippet emits a value
// the caller wants to inspect (e.g. calibration). For ack-style snippets
// that emit 0xDD on success, prefer runBootLCDSnippet().
static int runBootSnippet(const char* asmSrc, const char* tag, int maxCycles) {
    assembler.assemble(asmSrc);
    const AsmResult& r = assembler.result;
    if (r.error_count != 0) {
        Serial.printf("Boot %s: assembly failed (%d errors)\n", tag, r.error_count);
        return -1;
    }
    if (r.code_size > CODE_SIZE) {
        Serial.printf("Boot %s: code too large (%d > %d)\n",
                      tag, r.code_size, CODE_SIZE);
        return -1;
    }
    loadAsmResultForUpload(r);
    uploadToMK1(uploadBuf, uploadSize);
    return runUploadedUntilFirstOut(maxCycles);
}

static bool runBootLCDSnippet(const char* asmSrc, const char* tag, int maxCycles) {
    int result = runBootSnippet(asmSrc, tag, maxCycles);
    if (result != 0xDD) {
        Serial.printf("Boot %s: ack mismatch (expected 0xDD, got %d)\n", tag, result);
        return false;
    }
    return true;
}

// Cached MK1 clock calibration (blocks = MK1_clock / 4608). Set on first
// successful runBootCalibSnippet(). Stable across the session because the
// MK1 clock source doesn't change at runtime; if it ever does, callers can
// re-run with forceRecalibrate=true.
static int g_calibBlocks = 0;

// Run __delay_cal: configure DS3231 SQW=1Hz, sync to a rising edge, count
// 256-iter (dec+jnz) blocks during the 500ms HIGH phase. Emits the block
// count via out() and halts. Mirrors mk1cc2.py:__delay_cal exactly so the
// runtime cycle math (9 cycles per inner iter, blocks = F/4608) holds.
static int runBootCalibSnippet(bool forceRecalibrate = false) {
    if (g_calibBlocks > 0 && !forceRecalibrate) return g_calibBlocks;

    static const char* CALIB_ASM =
        "; Boot calibration (DS3231 SQW 1Hz HIGH-phase block count)\n"
        BOOT_LCD_VIA_PROLOGUE_ASM
        // DS3231 register 0x0E = 0x00 → SQW out, 1 Hz.
        "    jal __i2c_st\n"
        "    ldi $a, 0xD0\n    jal __i2c_sb\n"
        "    ldi $a, 0x0E\n    jal __i2c_sb\n"
        "    clr $a\n          jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        // PA0 reads SQW.
        "    ddra_imm 0x00\n"
        // Sync: wait for LOW (.s1) then rising edge (.s2 → HIGH).
        ".s1:\n"
        "    exrw 1\n"
        "    tst 0x01\n"
        "    jz .s2\n"
        "    j .s1\n"
        ".s2:\n"
        "    exrw 1\n"
        "    tst 0x01\n"
        "    jnz .cal\n"
        "    j .s2\n"
        // Calibrate: count 256-iter (dec+jnz) blocks while SQW is HIGH.
        ".cal:\n"
        "    ldi $b, 0\n"
        "    clr $a\n"
        ".chi:\n"
        "    dec\n"
        "    jnz .chi\n"
        "    mov $b, $a\n"      // A = B (current block count)
        "    inc\n"               // A = B + 1
        "    mov $a, $b\n"      // B = A
        "    exrw 1\n"
        "    tst 0x01\n"
        "    jz .cdn\n"
        "    clr $a\n"
        "    j .chi\n"
        ".cdn:\n"
        "    mov $b, $a\n"      // A = B (final blocks)
        "    out\n"
        "    hlt\n"
        BOOT_LCD_I2C_HELPERS_ASM;

    // Up to ~600ms of MK1 cycles for the SQW sync + 500ms HIGH phase + I2C
    // setup. At ~500kHz (max realistic) that's ~300k cycles; budget 4M.
    int result = runBootSnippet(CALIB_ASM, "calib", 4000000);
    if (result <= 0 || result > 255) {
        Serial.printf("Boot calib: invalid blocks (%d) — DS3231 SQW unreachable?\n",
                      result);
        return -1;
    }
    Serial.printf("Boot calib: blocks=%d (≈%d Hz MK1 clock)\n",
                  result, result * 4608);
    g_calibBlocks = result;
    return result;
}

static bool runBootLCDInitSnippet(uint8_t r, uint8_t g, uint8_t b) {
    static char asmBuf[6000];
    int n = snprintf(asmBuf, sizeof(asmBuf),
        "; Boot LCD init snippet (DFRobot RGB V2.0: AiP31068L 0x3E + PCA9633 0x2D)\n"
        BOOT_LCD_VIA_PROLOGUE_ASM
        // Power-on settle ~30ms (30 * 256 inner dec/jnz iterations).
        "    ldi $c, 30\n"
        ".po_o:\n"
        "    ldi $a, 0\n"
        ".po_i:\n"
        "    dec\n"
        "    jnz .po_i\n"
        "    decc\n"
        "    jnz .po_o\n"
        // Function Set 0x38: 8-bit, 2-line, 5x8 dots
        "    jal __i2c_st\n"
        "    ldi $a, 0x7C\n    jal __i2c_sb\n"
        "    ldi $a, 0x80\n    jal __i2c_sb\n"
        "    ldi $a, 0x38\n    jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        // Display ON 0x0C: display on, cursor off, blink off
        "    jal __i2c_st\n"
        "    ldi $a, 0x7C\n    jal __i2c_sb\n"
        "    ldi $a, 0x80\n    jal __i2c_sb\n"
        "    ldi $a, 0x0C\n    jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        // Clear Display 0x01
        "    jal __i2c_st\n"
        "    ldi $a, 0x7C\n    jal __i2c_sb\n"
        "    ldi $a, 0x80\n    jal __i2c_sb\n"
        "    ldi $a, 0x01\n    jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        // Post-clear settle ~3ms
        "    ldi $c, 5\n"
        ".cl_o:\n"
        "    ldi $a, 0\n"
        ".cl_i:\n"
        "    dec\n"
        "    jnz .cl_i\n"
        "    decc\n"
        "    jnz .cl_o\n"
        // Entry Mode 0x06: increment, no shift
        "    jal __i2c_st\n"
        "    ldi $a, 0x7C\n    jal __i2c_sb\n"
        "    ldi $a, 0x80\n    jal __i2c_sb\n"
        "    ldi $a, 0x06\n    jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        // PCA9633 V2.0 @ 0x2D: R=0x01, G=0x02, B=0x03
        "    jal __i2c_st\n"
        "    ldi $a, 0x5A\n    jal __i2c_sb\n"
        "    ldi $a, 0x01\n    jal __i2c_sb\n"
        "    ldi $a, %u\n      jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        "    jal __i2c_st\n"
        "    ldi $a, 0x5A\n    jal __i2c_sb\n"
        "    ldi $a, 0x02\n    jal __i2c_sb\n"
        "    ldi $a, %u\n      jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        "    jal __i2c_st\n"
        "    ldi $a, 0x5A\n    jal __i2c_sb\n"
        "    ldi $a, 0x03\n    jal __i2c_sb\n"
        "    ldi $a, %u\n      jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        "    out_imm 0xDD\n"
        "    hlt\n"
        BOOT_LCD_I2C_HELPERS_ASM,
        r, g, b);
    if (n < 0 || n >= (int)sizeof(asmBuf)) {
        Serial.println("Boot LCD init: snippet truncated");
        return false;
    }
    return runBootLCDSnippet(asmBuf, "init", 1500000);
}

static bool runLCDColorSnippet(uint8_t r, uint8_t g, uint8_t b) {
    static char asmBuf[4000];
    int n = snprintf(asmBuf, sizeof(asmBuf),
        "; LCD Color snippet (PCA9633 V2.0 @ 0x2D: R=0x01, G=0x02, B=0x03)\n"
        BOOT_LCD_VIA_PROLOGUE_ASM
        "    jal __i2c_st\n"
        "    ldi $a, 0x5A\n    jal __i2c_sb\n"
        "    ldi $a, 0x01\n    jal __i2c_sb\n"
        "    ldi $a, %u\n      jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        "    jal __i2c_st\n"
        "    ldi $a, 0x5A\n    jal __i2c_sb\n"
        "    ldi $a, 0x02\n    jal __i2c_sb\n"
        "    ldi $a, %u\n      jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        "    jal __i2c_st\n"
        "    ldi $a, 0x5A\n    jal __i2c_sb\n"
        "    ldi $a, 0x03\n    jal __i2c_sb\n"
        "    ldi $a, %u\n      jal __i2c_sb\n"
        "    jal __i2c_sp\n"
        "    out_imm 0xDD\n"
        "    hlt\n"
        BOOT_LCD_I2C_HELPERS_ASM,
        r, g, b);
    if (n < 0 || n >= (int)sizeof(asmBuf)) return false;
    return runBootLCDSnippet(asmBuf, "color", 1000000);
}

static bool runBootLCDLineSnippet(uint8_t setAddrCmd, const uint8_t* chars,
                                  int count, const char* tag) {
    if (count < 1 || count > 16) return false;

    static char asmBuf[6000];
    int written = snprintf(asmBuf, sizeof(asmBuf),
        "; Boot LCD line snippet (set DDRAM addr cmd 0x%02X then %d chars)\n"
        BOOT_LCD_VIA_PROLOGUE_ASM
        // Set DDRAM address (cmd byte): write LCD command 0x80 | addr
        "    jal __i2c_st\n"
        "    ldi $a, 0x7C\n    jal __i2c_sb\n"
        "    ldi $a, 0x80\n    jal __i2c_sb\n"
        "    ldi $a, %u\n      jal __i2c_sb\n"
        "    jal __i2c_sp\n",
        setAddrCmd, count, setAddrCmd);
    if (written < 0 || written >= (int)sizeof(asmBuf)) return false;
    char* p = asmBuf + written;
    int remain = (int)sizeof(asmBuf) - written;

    for (int i = 0; i < count; i++) {
        int wr = snprintf(p, remain,
            "    jal __i2c_st\n"
            "    ldi $a, 0x7C\n    jal __i2c_sb\n"
            "    ldi $a, 0x40\n    jal __i2c_sb\n"
            "    ldi $a, %u\n      jal __i2c_sb\n"
            "    jal __i2c_sp\n",
            chars[i]);
        if (wr < 0 || wr >= remain) return false;
        p += wr; remain -= wr;
    }

    int wr = snprintf(p, remain,
        "    out_imm 0xDD\n"
        "    hlt\n"
        BOOT_LCD_I2C_HELPERS_ASM);
    if (wr < 0 || wr >= remain) {
        Serial.printf("Boot LCD %s: snippet truncated\n", tag);
        return false;
    }

    return runBootLCDSnippet(asmBuf, tag, 1000000);
}

// Boot beep: play a square wave on PA1 via the same primitive as Twinkle's
// __tone. Self-calibrating — uses runBootCalibSnippet() to obtain blocks
// (= MK1_clock / 4608), then derives the per-half-period inner-loop count
// from the cycle math. Best-effort — failure does not fail updateBootLCD
// because the LCD content is already written.
//
// Duty cycle: __tone's outer-loop tail (16-bit period decrement, 15 cycles)
// runs while PA1 is LOW, making the LOW half 14 cycles longer than HIGH:
//
//   HIGH half =  1 (exw rest) + 3 (mov)  + 9*N + 5 (clr) + 3 (exw setup) = 12 + 9N
//   LOW  half =  1            + 3        + 9*N + 15 (tail) + 4 (ldi) + 3 = 26 + 9N
//
// To restore 50% duty we pad HIGH with exactly 14 cycles of A-preserving
// instructions (`nop nop nop tst 0` = 3+3+3+5). Pad size is constant — it
// matches the structural LOW-side overhead and is independent of clock and
// frequency. Per-period cost becomes 18*innerCount + 52.
//
// Solving for innerCount given target freqHz and clock F:
//   period_cycles = F / freqHz = 18*innerCount + 52
//   innerCount    = (F/freqHz - 52) / 18
//
// Substituting F = blocks * 4608:
//   innerCount    = (blocks * 4608/freqHz - 52) / 18
//                 = blocks * 256 / freqHz  -  52/18
//                 ≈ blocks * 256 / freqHz  -  3
static bool runBootBeepSnippet(int freqHz, int durationMs) {
    if (freqHz <= 0 || durationMs <= 0) return false;

    int blocks = runBootCalibSnippet();
    if (blocks <= 0) {
        Serial.println("Boot beep: skipped (calibration failed)");
        return false;
    }

    int innerCount = (blocks * 256 + freqHz / 2) / freqHz - 3;
    if (innerCount < 1)   innerCount = 1;
    if (innerCount > 255) innerCount = 255;

    // totalPeriods = freq * duration_ms / 1000  (clock-independent)
    int totalPeriods = (durationMs * freqHz + 500) / 1000;
    if (totalPeriods < 1)     totalPeriods = 1;
    if (totalPeriods > 65535) totalPeriods = 65535;
    int periodsLow  = totalPeriods & 0xFF;
    int periodsHigh = (totalPeriods >> 8) & 0xFF;

    static char asmBuf[3000];
    int n = snprintf(asmBuf, sizeof(asmBuf),
        "; Boot beep (PA1 square wave, %d Hz / %d ms; blocks=%d innerCount=%d periods=%d)\n"
        BOOT_LCD_VIA_PROLOGUE_ASM
        "    ddra_imm 0x02\n"
        "    ldi $a, %u\n    mov $a, $c\n"
        "    ldi $a, %u\n    mov $a, $b\n"
        "    ldi $a, %u\n    mov $a, $d\n"
        "    jal __tone\n"
        "    out_imm 0xDD\n"
        "    hlt\n"
        "__tone:\n"
        ".tloop:\n"
        "    ldi $a, 0x02\n"
        "    exw 0 1\n"
        "    mov $c, $a\n"
        ".thi:\n"
        "    dec\n"
        "    jnz .thi\n"
        "    nop\n"             // duty-cycle pad: 14 cycles to balance the
        "    nop\n"             // LOW-side outer-tail. Constant — independent
        "    nop\n"             // of MK1 clock and target frequency. None of
        "    tst 0\n"           // these instructions clobber $a.
        "    clr $a\n"
        "    exw 0 1\n"
        "    mov $c, $a\n"
        ".tlo:\n"
        "    dec\n"
        "    jnz .tlo\n"
        "    mov $b, $a\n"
        "    dec\n"
        "    mov $a, $b\n"
        "    jnz .tloop\n"
        "    mov $d, $a\n"
        "    tst 0xFF\n"
        "    jz .tdn\n"
        "    decd\n"
        "    j .tloop\n"
        ".tdn:\n"
        "    ret\n",
        freqHz, durationMs, blocks, innerCount, totalPeriods,
        innerCount, periodsLow, periodsHigh);
    if (n < 0 || n >= (int)sizeof(asmBuf)) {
        Serial.println("Boot beep: snippet truncated");
        return false;
    }

    int toneCycles = totalPeriods * (18 * innerCount + 52);
    int maxCycles  = toneCycles + 5000;
    if (maxCycles > 5000000) maxCycles = 5000000;

    return runBootLCDSnippet(asmBuf, "beep", maxCycles);
}

static bool updateBootLCD(int tempC, bool rtcSet, uint8_t hourBcd, uint8_t minBcd) {
    uint8_t timeChars[5] = {' ', ' ', ' ', ' ', ' '};
    if (rtcSet && bcdByteValid(hourBcd, 23) && bcdByteValid(minBcd, 59)) {
        timeChars[0] = '0' + ((hourBcd >> 4) & 0x0F);
        timeChars[1] = '0' + (hourBcd & 0x0F);
        timeChars[2] = ':';
        timeChars[3] = '0' + ((minBcd >> 4) & 0x0F);
        timeChars[4] = '0' + (minBcd & 0x0F);
    }

    uint8_t tempChars[4] = {'?', '?', 0xDF, 'C'};
    if (tempC >= 0 && tempC <= 99) {
        tempChars[0] = tempC >= 10 ? ('0' + (tempC / 10)) : ' ';
        tempChars[1] = '0' + (tempC % 10);
    } else if (tempC < 0 && tempC >= -9) {
        tempChars[0] = '-';
        tempChars[1] = '0' + (-tempC);
    }

    if (!runBootLCDInitSnippet(g_lcdR, g_lcdG, g_lcdB)) return false;
    if (!runBootLCDLineSnippet(0x80, timeChars, 5, "time")) return false;
    if (!runBootLCDLineSnippet(0x8C, tempChars, 4, "temp")) return false;
    runBootBeepSnippet(896, 200);  // best-effort PC-style boot beep
    return true;
}

static void handleSetLcdColor() {
    if (!server.hasArg("r") || !server.hasArg("g") || !server.hasArg("b")) {
        server.send(400, "application/json", "{\"ok\":false,\"error\":\"Missing r, g, or b\"}");
        return;
    }
    uint8_t r = (uint8_t)server.arg("r").toInt();
    uint8_t g = (uint8_t)server.arg("g").toInt();
    uint8_t b = (uint8_t)server.arg("b").toInt();

    bool ok = runLCDColorSnippet(r, g, b);
    if (ok) {
        g_lcdR = r; g_lcdG = g; g_lcdB = b;
        saveLcdColor(r, g, b);
    }

    // Restore the saved program (snippet overwrote RAM)
    File f = FFat.open(AUTOSAVE_PATH, "r");
    if (f) {
        String source = f.readString();
        f.close();
        if (source.length() > 0) {
            assembler.assemble(source.c_str());
            const AsmResult& res = assembler.result;
            if (res.error_count == 0) {
                loadAsmResultForUpload(res);
                uploadToMK1(uploadBuf, uploadSize);
            }
        }
    }

    server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

// ── Setup & Loop ─────────────────────────────────────────────────────

void setup() {
    Serial.setRxBufferSize(16384);  // 16KB RX buffer for large ASM uploads
    Serial.begin(115200);

    pinMode(PIN_MI, OUTPUT);
    pinMode(PIN_HL, OUTPUT);
    pinMode(PIN_RI, OUTPUT);
    pinMode(PIN_EN, OUTPUT);
    pinMode(PIN_STK, OUTPUT);
    digitalWrite(PIN_STK, LOW);
    pinMode(PIN_DIR, OUTPUT);
    digitalWrite(PIN_DIR, HIGH);  // U59 DIR: HIGH = A→B = ESP32→bus (write mode)
    pinMode(PIN_RO, INPUT);       // ~RO: high-Z by default (EEPROM drives this line during normal operation)
    pinMode(PIN_OI, INPUT);       // OI: read to detect when CPU executes 'out'
    pinMode(PIN_HLT, INPUT);      // HLT: read to detect when CPU halts
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, HIGH);  // hold MK1 in reset immediately — prevents garbage execution during boot
    pinMode(PIN_CLK, INPUT);
    pinMode(PIN_CU_EN, OUTPUT);
    pinMode(PIN_CLK_SENSE, INPUT);
    busSetOutput();

    if (!FFat.begin(false)) {
        FFat.format();
        FFat.begin(true);
    }
    loadWifiConfig();
    loadLcdColor();

    // Restore last program from flash (or upload HLT as safe default)
    {
        bool restored = false;
        File f = FFat.open(AUTOSAVE_PATH, "r");
        if (f) {
            String source = f.readString();
            f.close();
            if (source.length() > 0) {
                int clockDirective = parseClockDirective(source);
                assembler.assemble(source.c_str());
                const AsmResult& r = assembler.result;
                if (r.error_count == 0 && r.code_size > 0) {
                    int codeBytes = r.code_size < CODE_SIZE ? r.code_size : CODE_SIZE;
                    memcpy(uploadBuf, r.code, codeBytes);
                    memset(uploadBuf + codeBytes, 0, CODE_SIZE - codeBytes);
                    uploadSize = CODE_SIZE;
                    uploadToMK1(uploadBuf, uploadSize);
                    if (clockDirective > 0) startCustomClock(clockDirective);
                    Serial.printf("Restored program from flash (%d bytes, clock %d Hz)\n", codeBytes, clockDirective);
                    restored = true;
                }
            }
        }
        if (!restored) {
            uploadBuf[0] = 0x7F;  // HLT
            uploadSize = 1;
            uploadToMK1(uploadBuf, uploadSize);
            Serial.println("No saved program — uploaded HLT");
        }
    }

    // Try STA mode first, fall back to AP
    if (!connectToWifi()) {
        startAP();
    }

    // ── NTP → DS3231 RTC sync ──────────────────────────────────────────
    // If connected to WiFi and time hasn't been set (cold boot), sync NTP
    // then program the DS3231 RTC via an MK1 I2C program.
    if (staMode && time(NULL) < 1000000000) {
        // UK timezone: GMT0BST,M3.5.0/1,M10.5.0/2
        configTzTime("GMT0BST,M3.5.0/1,M10.5.0/2", "pool.ntp.org", "time.google.com");
        Serial.print("NTP sync...");
        int ntpTries = 0;
        while (time(NULL) < 1000000000 && ntpTries < 20) {
            delay(500);
            ntpTries++;
        }
        if (time(NULL) > 1000000000) {
            struct tm t;
            getLocalTime(&t);
            Serial.printf(" OK: %04d-%02d-%02d %02d:%02d:%02d\n",
                t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                t.tm_hour, t.tm_min, t.tm_sec);

            // Convert to BCD for DS3231 registers 0x00-0x06
            auto toBCD = [](int v) -> uint8_t { return ((v / 10) << 4) | (v % 10); };
            uint8_t sec  = toBCD(t.tm_sec);
            uint8_t min  = toBCD(t.tm_min);
            uint8_t hour = toBCD(t.tm_hour);  // 24-hour mode (bit 6 = 0)
            uint8_t dow  = toBCD(t.tm_wday + 1);  // DS3231: 1=Sunday
            uint8_t date = toBCD(t.tm_mday);
            uint8_t mon  = toBCD(t.tm_mon + 1);
            uint8_t year = toBCD(t.tm_year % 100);

            // Build MK1 assembly to write DS3231 time registers via I2C
            // DS3231 I2C addr: 0x68 → write = 0xD0
            // Write 7 bytes starting at register 0x00
            char asmBuf[2048];
            snprintf(asmBuf, sizeof(asmBuf),
                "; NTP → DS3231 RTC sync (auto-generated)\n"
                "    ldi $d, 0\n"
                ".dly:\n"
                "    dec\n"
                "    jnz .dly\n"
                "    clr $a\n"
                "    exw 0 0\n"
                "    ddrb_imm 0x00\n"
                "    clr $a\n"
                "    exw 0 3\n"
                "    push $a\n"
                "    pop $a\n"
                "    jal __i2c_st\n"
                "    ldi $a, 0xD0\n"  // DS3231 write address
                "    jal __i2c_sb\n"
                "    clr $a\n"        // register 0x00
                "    jal __i2c_sb\n"
                "    ldi $a, 0x%02X\n"  // seconds
                "    jal __i2c_sb\n"
                "    ldi $a, 0x%02X\n"  // minutes
                "    jal __i2c_sb\n"
                "    ldi $a, 0x%02X\n"  // hours (24h)
                "    jal __i2c_sb\n"
                "    ldi $a, 0x%02X\n"  // day of week
                "    jal __i2c_sb\n"
                "    ldi $a, 0x%02X\n"  // date
                "    jal __i2c_sb\n"
                "    ldi $a, 0x%02X\n"  // month
                "    jal __i2c_sb\n"
                "    ldi $a, 0x%02X\n"  // year
                "    jal __i2c_sb\n"
                "    jal __i2c_sp\n"
                "    out_imm 0xDD\n"
                "    hlt\n"
                "__i2c_st:\n"
                "    exrw 2\n"
                "    ddrb_imm 0x01\n"
                "    ddrb_imm 0x03\n"
                "    ret\n"
                "__i2c_sp:\n"
                "    ddrb_imm 0x03\n"
                "    ddrb_imm 0x01\n"
                "    ddrb_imm 0x00\n"
                "    ret\n"
                "__i2c_sb:\n"
                "    mov $a, $b\n"
                "    ldi $a, 8\n"
                "    mov $a, $c\n"
                ".isb:\n"
                "    mov $b, $a\n"
                "    tst 0x80\n"
                "    jnz .isbh\n"
                "    ddrb_imm 0x03\n"
                "    ddrb_imm 0x01\n"
                "    ddrb_imm 0x03\n"
                "    j .isbn\n"
                ".isbh:\n"
                "    ddrb_imm 0x02\n"
                "    ddrb_imm 0x00\n"
                "    ddrb_imm 0x02\n"
                ".isbn:\n"
                "    mov $b, $a\n"
                "    sll\n"
                "    mov $a, $b\n"
                "    mov $c, $a\n"
                "    dec\n"
                "    mov $a, $c\n"
                "    jnz .isb\n"
                "    ddrb_imm 0x02\n"
                "    ddrb_imm 0x00\n"
                "    exrw 0\n"
                "    ddrb_imm 0x02\n"
                "    ret\n",
                sec, min, hour, dow, date, mon, year
            );

            // Assemble and run
            assembler.assemble(asmBuf);
            const AsmResult& r = assembler.result;
            if (r.error_count == 0) {
                memcpy(uploadBuf, r.code, CODE_SIZE);
                uploadSize = CODE_SIZE;
                uploadToMK1(uploadBuf, uploadSize);

                // Run at 500kHz (us=1) — enough cycles for VIA init + I2C write
                stopCustomClock();
                detachOIMonitorForManualRun();
                outputCaptured = false;
                oiCount = 0;

                int clkGpio = digitalPinToGPIONumber(PIN_CLK);
                uint32_t clkMask = 1 << clkGpio;
                int oiGpio = digitalPinToGPIONumber(PIN_OI);
                if (oiGpio < 0) oiGpio = PIN_OI;
                uint32_t oiMask = 1 << oiGpio;

                busSetInput();
                disableOutput();
                digitalWrite(PIN_DIR, LOW);
                enableOutput();
                pinMode(PIN_CLK, OUTPUT);
                digitalWrite(PIN_CLK, LOW);

                bool rtcOk = false;
                for (int i = 0; i < 50000; i++) {
                    GPIO.out_w1ts = clkMask;
                    delayMicroseconds(1);
                    uint32_t g1 = GPIO.in;
                    if (g1 & oiMask) {
                        uint8_t val = decodeBus(g1);
                        rtcOk = (val == 0xDD);
                        GPIO.out_w1tc = clkMask;
                        break;
                    }
                    GPIO.out_w1tc = clkMask;
                    delayMicroseconds(1);
                    uint32_t g2 = GPIO.in;
                    if (g2 & oiMask) {
                        uint8_t val = decodeBus(g2);
                        rtcOk = (val == 0xDD);
                        break;
                    }
                }
                pinMode(PIN_CLK, INPUT);
                disableOutput();
                digitalWrite(PIN_DIR, HIGH);
                busSetOutput();
                enableOutput();

                Serial.printf("DS3231 RTC set: %s\n", rtcOk ? "OK" : "FAIL");
            } else {
                Serial.printf("RTC program assembly failed (%d errors)\n", r.error_count);
            }
        } else {
            Serial.println(" NTP timeout");
        }
    }

    // ── Boot status display ──────────────────────────────────────────
    {
        bool lcdFound = (runI2CScanAddr(0x3E) == 1);
        int rtcStatus = -1;
        int rtcHour = -1;
        int rtcMin = -1;

        if (lcdFound) {
            rtcStatus = readDS3231Reg(0x0F);  // DS3231 OSF bit marks unset/lost time
            if (rtcStatus >= 0 && (rtcStatus & 0x80) == 0) {
                rtcHour = readDS3231Reg(0x02);
                rtcMin = readDS3231Reg(0x01);
            }
        }

        // Keep this after the probe/register reads: those use out() for the
        // ESP32 harness, and the boot-visible out() value should remain temp.
        int tempC = readDS3231Temp();
        if (tempC >= 0) {
            Serial.printf("DS3231 temperature: %d°C (on 7-seg)\n", tempC);
            if (lcdFound) {
                bool lcdSet = rtcStatus >= 0 && (rtcStatus & 0x80) == 0 &&
                              rtcHour >= 0 && rtcMin >= 0;
                bool lcdOk = updateBootLCD(tempC, lcdSet,
                                           (uint8_t)rtcHour, (uint8_t)rtcMin);
                readDS3231Temp();  // restore visible out() after LCD completion marker
                Serial.printf("Boot LCD status: %s%s\n",
                    lcdOk ? "OK" : "FAIL",
                    lcdSet ? "" : " (RTC time unset)");
            } else {
                Serial.println("Boot LCD status: not found at 0x3E");
            }
        } else {
            Serial.println("DS3231 temperature read failed");
        }
        // Restore the saved program (temp read overwrote RAM)
        File f = FFat.open(AUTOSAVE_PATH, "r");
        if (f) {
            String source = f.readString();
            f.close();
            if (source.length() > 0) {
                assembler.assemble(source.c_str());
                const AsmResult& r2 = assembler.result;
                if (r2.error_count == 0 && r2.code_size > 0) {
                    memcpy(uploadBuf, r2.code, CODE_SIZE);
                    uploadSize = CODE_SIZE;
                    uploadToMK1(uploadBuf, uploadSize);
                    Serial.println("Restored saved program");
                }
            }
        }
    }

    MDNS.begin(MDNS_HOST);  // mk1.local

    server.on("/", HTTP_GET, handleRoot);
    server.on("/assemble", HTTP_POST, handleAssemble);
    server.on("/upload", HTTP_POST, handleUpload);
    server.on("/halt", HTTP_POST, handleHalt);
    server.on("/reset", HTTP_POST, handleReset);
    server.on("/step", HTTP_POST, handleStep);
    server.on("/resume", HTTP_POST, handleResume);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/clock", HTTP_GET, handleClock);
    server.on("/clock", HTTP_POST, handleClock);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/load", HTTP_GET, handleLoad);
    server.on("/programs", HTTP_GET, handleProgramList);
    server.on("/programs/delete", HTTP_POST, handleProgramDelete);
    server.on("/read", HTTP_GET, handleRead);
    server.on("/bussniff", HTTP_GET, handleBusSniff);
    server.on("/read_output", HTTP_GET, handleReadOutput);
    server.on("/wait_output", HTTP_POST, handleWaitOutput);
    server.on("/upload_and_wait", HTTP_POST, handleUploadAndWait);
    server.on("/run_cycles", HTTP_POST, handleRunCycles);
    server.on("/i2cscan", HTTP_GET, handleI2CScan);
    server.on("/set_lcd_color", HTTP_POST, handleSetLcdColor);
    server.on("/readtest", HTTP_GET, handleReadTest);
    server.on("/stepn", HTTP_GET, handleStepN);
    server.on("/probe", HTTP_GET, handleProbe);
    server.on("/wifi", HTTP_GET, handleWifiGet);
    server.on("/wifi", HTTP_POST, handleWifiPost);
    server.begin();

    MDNS.addService("http", "tcp", 80);

    startClkMonitor();
}

void loop() {
    server.handleClient();
    handleSerialUpload();
    disableOutput();
    updateClkMeasurement();

    // Attach HLT interrupt: wait until HLT is LOW (CPU running), then arm.
    // This avoids false triggers from HLT being high during reset.
    if (usingLEDC && !hltIrqAttached && !hltFired) {
        if (digitalRead(PIN_HLT) == LOW) {
            attachInterrupt(digitalPinToInterrupt(PIN_HLT), onHltRising, RISING);
            hltIrqAttached = true;
        }
    }

    // HLT detection: ISR already killed LEDC, just update state
    if (hltFired) {
        hltFired = false;
        hltIrqAttached = false;
        detachInterrupt(digitalPinToInterrupt(PIN_HLT));
        usingLEDC = false;
        cpuState = CPU_HALTED;
    }
}
