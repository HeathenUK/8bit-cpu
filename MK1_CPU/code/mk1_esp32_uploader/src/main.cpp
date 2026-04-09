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

    // Use 1-bit resolution for all frequencies — gives exact 50% duty
    // and accurate frequency via integer prescaler (80MHz / (prescaler * 2))
    uint8_t resolution = 1;
    uint32_t duty = 1;  // 50% at 1-bit resolution

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
static volatile uint32_t oiCount = 0;
static volatile uint8_t oiHistory[256];

static uint32_t oiGpioMask = 0;  // precomputed at startOIMonitor

static inline uint8_t IRAM_ATTR readBusFast() {
    uint32_t gpio = GPIO.in;
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

static void IRAM_ATTR onOIRising() {
    uint8_t val = readBusFast();
    if (oiCount < 256) oiHistory[oiCount] = val;
    oiCount++;  // always count, even if value is same (for rate measurement)
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

    // Find first non-spurious ISR value (filter 63=floating, 127=HLT)
    uint32_t n = oiCount < 256 ? oiCount : 16;
    for (uint32_t i = 0; i < n; i++) {
        if (oiHistory[i] != 0x3F && oiHistory[i] != 0x7F &&
            oiHistory[i] != 0xBF && oiHistory[i] != 0xFF) {
            val = oiHistory[i];
            found = true;
            break;
        }
    }
    // If all spurious, report the first one anyway
    if (!found && n > 0) {
        val = oiHistory[0];
        found = true;
    }

    String json = "{\"value\":" + String(val) +
                  ",\"found\":" + (found ? "true" : "false") +
                  ",\"oi_count\":" + String(oiCount) +
                  ",\"history\":[";
    for (uint32_t i = 0; i < n; i++) {
        if (i > 0) json += ",";
        json += String(oiHistory[i]);
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
    uint8_t reportVal = 0;
    bool found = false;
    uint32_t n = oiCount < 256 ? oiCount : 16;
    for (uint32_t i = 0; i < n; i++) {
        if (oiHistory[i] != 0x3F && oiHistory[i] != 0x7F) {
            reportVal = oiHistory[i];
            found = true;
            break;
        }
    }
    if (!found && n > 0) reportVal = oiHistory[0];  // fallback to first
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
        json += String(oiHistory[i]);
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
    if (n > 5000000) n = 5000000;
    int halfPeriodUs = 1;  // microseconds per half-cycle (default ~500kHz — optimal for this board)
    if (server.hasArg("us")) halfPeriodUs = server.arg("us").toInt();
    if (halfPeriodUs < 0) halfPeriodUs = 0;
    if (halfPeriodUs > 1000) halfPeriodUs = 1000;

    stopCustomClock();
    // Detach OI interrupt — we'll poll OI directly in the clock loop
    if (oiMonitorActive) {
        detachInterrupt(digitalPinToInterrupt(PIN_OI));
    }
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
            uint8_t val = readBusFast();
            if (oiCount < 256) oiHistory[oiCount] = val;
            oiCount++;
            lastOutputVal = val;
            outputCaptured = true;
            GPIO.out_w1tc = clkMask;
            break;
        }

        // CLK LOW
        GPIO.out_w1tc = clkMask;
        if (halfPeriodUs > 0) delayMicroseconds(halfPeriodUs);

        // Check OI during LOW phase
        uint32_t gpio2 = GPIO.in;
        if (gpio2 & oiMask) {
            uint8_t val = readBusFast();
            if (oiCount < 256) oiHistory[oiCount] = val;
            oiCount++;
            lastOutputVal = val;
            outputCaptured = true;
            break;
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

// ── ESP32 direct I2C scan (bypasses MK1, uses Wire library) ─────────
// Connect display SDA/SCL directly to ESP32 A6/A7 for this test
static void handleI2CScan() {
    Wire.begin(A6, A7);  // SDA=A6, SCL=A7
    Wire.setClock(100000);  // 100kHz standard mode

    String json = "{\"found\":[";
    bool first = true;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            if (!first) json += ",";
            json += String(addr);
            first = false;
        }
    }
    json += "]}";

    Wire.end();
    // Restore A6/A7 to their normal functions
    pinMode(A6, INPUT);   // ~RO: high-Z
    pinMode(A7, INPUT);

    server.send(200, "application/json", json);
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
            Serial.printf("{\"ok\":false,\"errors\":%d}\n", r.error_count);
            return;
        }
        int codeBytes = r.code_size < CODE_SIZE ? r.code_size : CODE_SIZE;
        int dataBytes = r.data_size < DATA_SIZE ? r.data_size : DATA_SIZE;
        int p3Bytes = r.page3_size < DATA_SIZE ? r.page3_size : DATA_SIZE;
        memcpy(uploadBuf, r.code, CODE_SIZE);
        uploadSize = CODE_SIZE;
        if (dataBytes > 0 || p3Bytes > 0) {
            memcpy(uploadBuf + CODE_SIZE, r.data, dataBytes);
            memset(uploadBuf + CODE_SIZE + dataBytes, 0, DATA_SIZE - dataBytes);
            uploadSize = CODE_SIZE + DATA_SIZE;
        }
        if (p3Bytes > 0) {
            memset(uploadBuf + CODE_SIZE + DATA_SIZE, 0, DATA_SIZE);  // page 2 = zeros
            memcpy(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE, r.page3, p3Bytes);
            memset(uploadBuf + CODE_SIZE + DATA_SIZE + DATA_SIZE + p3Bytes, 0, DATA_SIZE - p3Bytes);
            uploadSize = CODE_SIZE + DATA_SIZE + DATA_SIZE + DATA_SIZE;
        }
        uint32_t cksum = 0;
        for (int i = 0; i < uploadSize; i++) cksum += uploadBuf[i];
        Serial.printf("{\"ok\":true,\"code\":%d,\"data\":%d,\"cksum\":%u,\"hex\":\"", codeBytes, dataBytes, cksum);
        for (int i = 0; i < codeBytes && i < 16; i++) {
            if (i) Serial.print(' ');
            Serial.printf("%02X", r.code[i]);
        }
        Serial.println("\"}");
    }
    else if (line == "UPLOAD") {
        if (uploadSize == 0) { Serial.println("{\"ok\":false}"); return; }
        uploadToMK1(uploadBuf, uploadSize);
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
        if (n < 1) n = 1;
        if (n > 5000000) n = 5000000;
        if (us < 0) us = 0;
        if (nops < 0) nops = 0;
        if (nops > 10000) nops = 10000;

        stopCustomClock();
        if (oiMonitorActive) detachInterrupt(digitalPinToInterrupt(PIN_OI));
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

        int actualCycles = 0;
        unsigned long t0_us = micros();
        for (int i = 0; i < n; i++) {
            actualCycles++;
            GPIO.out_w1ts = clkMask;
            if (us > 0) delayMicroseconds(us);
            else if (nops > 0) { for (volatile int j = 0; j < nops; j++) __asm__ __volatile__("nop"); }
            uint32_t gpio1 = GPIO.in;
            if (gpio1 & oiMask) {
                uint8_t val = readBusFast();
                if (oiCount < 256) oiHistory[oiCount] = val;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                GPIO.out_w1tc = clkMask;
                break;
            }
            GPIO.out_w1tc = clkMask;
            if (us > 0) delayMicroseconds(us);
            else if (nops > 0) { for (volatile int j = 0; j < nops; j++) __asm__ __volatile__("nop"); }
            uint32_t gpio2 = GPIO.in;
            if (gpio2 & oiMask) {
                uint8_t val = readBusFast();
                if (oiCount < 256) oiHistory[oiCount] = val;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                break;
            }
        }
        unsigned long elapsed_us = micros() - t0_us;
        pinMode(PIN_CLK, INPUT);
        disableOutput();
        digitalWrite(PIN_DIR, HIGH);
        busSetOutput();
        enableOutput();

        // Report actual frequency: cycles / elapsed_time
        float actual_khz = (actualCycles > 100 && elapsed_us > 100)
            ? (float)actualCycles / elapsed_us * 1000.0f : 0;
        Serial.printf("{\"cyc\":%d,\"val\":%d,\"cap\":%s,\"us\":%lu,\"khz\":%.1f}\n",
            actualCycles,
            oiCount > 0 ? oiHistory[0] : 0,
            outputCaptured ? "true" : "false",
            elapsed_us,
            actual_khz);
    }
    else if (line == "OI") {
        uint8_t val = 0;
        bool found = false;
        uint32_t n = oiCount < 256 ? oiCount : 16;
        for (uint32_t i = 0; i < n; i++) {
            if (oiHistory[i] != 0x3F && oiHistory[i] != 0x7F) {
                val = oiHistory[i]; found = true; break;
            }
        }
        if (!found && n > 0) val = oiHistory[0];
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
    else if (line.startsWith("RUNLOG:")) {
        // RUNLOG:cycles,us — run without stopping on OI, log all OI values
        int comma = line.indexOf(',', 7);
        int n = line.substring(7, comma > 0 ? comma : line.length()).toInt();
        int us = comma > 0 ? line.substring(comma + 1).toInt() : 1;
        if (n < 1) n = 1;
        if (n > 5000000) n = 5000000;
        if (us < 0) us = 0;

        stopCustomClock();
        if (oiMonitorActive) detachInterrupt(digitalPinToInterrupt(PIN_OI));
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

        int actualCycles = 0;
        unsigned long t0_us = micros();
        for (int i = 0; i < n; i++) {
            actualCycles++;
            GPIO.out_w1ts = clkMask;
            if (us > 0) delayMicroseconds(us);
            uint32_t gpio1 = GPIO.in;
            if (gpio1 & oiMask) {
                uint8_t val = readBusFast();
                if (oiCount < 256) oiHistory[oiCount] = val;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
                // DON'T break — keep running
            }
            GPIO.out_w1tc = clkMask;
            if (us > 0) delayMicroseconds(us);
            uint32_t gpio2 = GPIO.in;
            if (gpio2 & oiMask) {
                uint8_t val = readBusFast();
                if (oiCount < 256) oiHistory[oiCount] = val;
                oiCount++;
                lastOutputVal = val;
                outputCaptured = true;
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
        // Output all captured values
        Serial.printf("{\"cyc\":%d,\"cnt\":%d,\"khz\":%.1f,\"vals\":[", actualCycles, oiCount, actual_khz);
        int show = oiCount < 256 ? oiCount : 256;
        for (int i = 0; i < show; i++) {
            if (i) Serial.print(',');
            Serial.print(oiHistory[i]);
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

// ── Setup & Loop ─────────────────────────────────────────────────────

void setup() {
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
                if (oiMonitorActive) detachInterrupt(digitalPinToInterrupt(PIN_OI));
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
                    if (GPIO.in & oiMask) {
                        uint8_t val = readBusFast();
                        rtcOk = (val == 0xDD);
                        GPIO.out_w1tc = clkMask;
                        break;
                    }
                    GPIO.out_w1tc = clkMask;
                    delayMicroseconds(1);
                    if (GPIO.in & oiMask) {
                        uint8_t val = readBusFast();
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

            // Restore the saved program (the RTC set program overwrote RAM)
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
                        Serial.println("Restored saved program after RTC set");
                    }
                }
            }
        } else {
            Serial.println(" NTP timeout");
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
