// MK1 CPU — ESP32 Web IDE + Program Uploader
//
// Runs a WiFi access point with a web-based assembly editor.
// Enter MK1 assembly -> Assemble & Run -> program is written to SRAM.
//
// Connect to WiFi "MK1-CPU" (no password) -> open http://192.168.4.1
//
// Also supports serial upload with "MK" magic header for uploader.py.

#include <Arduino.h>
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
static uint8_t uploadBuf[512];
static int uploadSize = 0;

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

static void enableClock() {
    stopClkMonitor();
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, HIGH);
}

static void disableClock() {
    stopClkMonitor();
    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, LOW);
    pinMode(PIN_CLK, INPUT);
    startClkMonitor();
}

static void disableCU() { digitalWrite(PIN_CU_EN, HIGH); }
static void enableCU()  { digitalWrite(PIN_CU_EN, LOW);  }

static void enableOutput()  { digitalWrite(PIN_EN, HIGH); }
static void disableOutput() { digitalWrite(PIN_EN, LOW);  }

static void setAddress(unsigned int address) {
    digitalWrite(PIN_HL, address > 0xFF);
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
    stopClkMonitor();
    busSetOutput();
    resetPulse();
    disableCU();
    delayMicroseconds(2);
    enableClock();
    delayMicroseconds(2);

    for (int i = 0; i < size; i++) {
        setAddress(i);
        delayMicroseconds(2);
        writeInstruction(buf[i]);
        delayMicroseconds(2);
    }

    disableClock();  // also restarts clock monitor
    digitalWrite(PIN_HL, LOW);
    resetPulse();
    enableCU();
    totalCycles = 0;
    cpuState = CPU_RUNNING;
}

// ── Single step ──────────────────────────────────────────────────────

static uint8_t singleStep() {
    // Pulse CLK manually, read bus value after rising edge
    stopClkMonitor();
    busSetInput();
    disableOutput();

    pinMode(PIN_CLK, OUTPUT);
    digitalWrite(PIN_CLK, HIGH);
    delayMicroseconds(5);
    uint8_t busVal = busRead();
    digitalWrite(PIN_CLK, LOW);
    delayMicroseconds(5);
    pinMode(PIN_CLK, INPUT);

    totalCycles++;
    return busVal;
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

        memcpy(uploadBuf, r.code, codeBytes);
        memset(uploadBuf + codeBytes, 0, CODE_SIZE - codeBytes);
        uploadSize = CODE_SIZE;

        if (dataBytes > 0) {
            memcpy(uploadBuf + CODE_SIZE, r.data, dataBytes);
            memset(uploadBuf + CODE_SIZE + dataBytes, 0, DATA_SIZE - dataBytes);
            uploadSize = CODE_SIZE + DATA_SIZE;
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
        json += "\"}";
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
    }
    cpuState = CPU_STEPPING;

    // Execute one clock cycle, read bus
    uint8_t busVal = singleStep();

    String json = "{\"ok\":true,\"bus\":" + String(busVal) +
                  ",\"cycles\":" + String(totalCycles) + "}";
    server.send(200, "application/json", json);
}

static void handleResume() {
    // Resume from halt/step to free-running
    disableClock();
    enableCU();
    cpuState = CPU_RUNNING;
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleStatus() {
    updateClkMeasurement();

    // Sample bus (only meaningful at low speeds or when halted)
    busSetInput();
    disableOutput();
    uint8_t busVal = busRead();

    const char* stateStr = "running";
    if (cpuState == CPU_HALTED) stateStr = "halted";
    else if (cpuState == CPU_STEPPING) stateStr = "stepping";

    String json = "{\"state\":\"" + String(stateStr) +
                  "\",\"clock_hz\":" + String(measuredClkHz, 1) +
                  ",\"bus\":" + String(busVal) +
                  ",\"cycles\":" + String(totalCycles) + "}";
    server.send(200, "application/json", json);
}

static const char* SAVE_PATH = "/program.asm";

static void handleSave() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"ok\":false}");
        return;
    }
    File f = FFat.open(SAVE_PATH, "w");
    if (!f) {
        server.send(500, "application/json", "{\"ok\":false,\"error\":\"Flash write failed\"}");
        return;
    }
    f.print(server.arg("plain"));
    f.close();
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleLoad() {
    File f = FFat.open(SAVE_PATH, "r");
    if (!f) {
        server.send(200, "text/plain", "");
        return;
    }
    String content = f.readString();
    f.close();
    server.send(200, "text/plain", content);
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

    // Wait up to 10 seconds
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
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

static void handleSerialUpload() {
    if (Serial.available() >= 2) {
        uint8_t header[2];
        Serial.readBytes(header, 2);
        if (header[0] == SERIAL_MAGIC[0] && header[1] == SERIAL_MAGIC[1]) {
            uint8_t buf[512];
            int n = Serial.readBytes(buf, sizeof(buf));
            Serial.write(0x00);
            uploadToMK1(buf, n);
        } else {
            while (Serial.available()) Serial.read();
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
    pinMode(PIN_CLK, INPUT);
    pinMode(PIN_RST, INPUT);
    pinMode(PIN_CU_EN, OUTPUT);
    pinMode(PIN_CLK_SENSE, INPUT);
    busSetOutput();

    if (!FFat.begin(false)) {
        FFat.format();
        FFat.begin(true);
    }
    loadWifiConfig();

    // Try STA mode first, fall back to AP
    if (!connectToWifi()) {
        startAP();
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
    server.on("/save", HTTP_POST, handleSave);
    server.on("/load", HTTP_GET, handleLoad);
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
}
