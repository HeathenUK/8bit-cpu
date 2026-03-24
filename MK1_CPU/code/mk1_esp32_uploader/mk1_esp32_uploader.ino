// MK1 8-bit Computer — ESP32 Uploader
// Port of mk1_computer_uplodader for the Arduino Nano ESP32.
//
// Changes from the AVR original:
//   - Replaces PORTD/PORTB direct register writes with digitalWrite()
//     (ESP32 has no AVR port registers).
//   - Slightly increased settle times (2us) to account for ESP32's faster
//     clock — the 74HCT latches still need real propagation time.
//   - Serial protocol is identical: send up to 512 bytes, receive a 0x00 ack.
//
// Pin mapping is unchanged from the original Nano footprint. The Arduino Nano
// ESP32 maps these Arduino pin numbers to ESP32 GPIOs internally.
//
// Note on logic levels: 74HCT inputs use TTL thresholds (VIH = 2.0V),
// so the ESP32's 3.3V outputs are valid logic HIGH.

// --- Control pins (accent signals) ---
#define MI    10    // Memory Address Register latch
#define HL    11    // High/Low address space select
#define RI    12    // RAM write strobe
#define EN    13    // Output enable (drive data onto bus)
#define CLK   A0    // Clock line
#define RST   A1    // Reset line
#define CU_EN A2    // Control Unit enable (active-low; HIGH = disabled)

// --- Data bus pins: D2..D9 carry 8 bits of data ---
static const uint8_t DATA_PINS[] = { 2, 3, 4, 5, 6, 7, 8, 9 };
#define DATA_PIN_COUNT 8

// -----------------------------------------------------------------------
// Initialisation
// -----------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(MI,    OUTPUT);
  pinMode(HL,    OUTPUT);
  pinMode(RI,    OUTPUT);
  pinMode(EN,    OUTPUT);
  pinMode(CLK,   INPUT);   // tri-state until we need it
  pinMode(RST,   INPUT);   // tri-state until we need it
  pinMode(CU_EN, OUTPUT);

  for (uint8_t i = 0; i < DATA_PIN_COUNT; i++) {
    pinMode(DATA_PINS[i], OUTPUT);
  }
}

// -----------------------------------------------------------------------
// Low-level bus helpers
// -----------------------------------------------------------------------

// Place an 8-bit value on the data bus (D2..D9).
void putOut(uint8_t data) {
  for (uint8_t i = 0; i < DATA_PIN_COUNT; i++) {
    digitalWrite(DATA_PINS[i], (data >> i) & 1);
  }
  delayMicroseconds(2);
}

// -----------------------------------------------------------------------
// Clock & reset — accent the lines only while needed, then tri-state
// -----------------------------------------------------------------------
void pulseReset() {
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  delayMicroseconds(10);
  digitalWrite(RST, LOW);
  pinMode(RST, INPUT);
}

void pulseClock() {
  pinMode(CLK, OUTPUT);
  digitalWrite(CLK, HIGH);
  delayMicroseconds(2);
  digitalWrite(CLK, LOW);
  pinMode(CLK, INPUT);
}

void enableClock() {
  pinMode(CLK, OUTPUT);
  digitalWrite(CLK, HIGH);
}

void disableClock() {
  pinMode(CLK, OUTPUT);
  digitalWrite(CLK, LOW);
  pinMode(CLK, INPUT);
}

// -----------------------------------------------------------------------
// Control Unit enable / disable
// -----------------------------------------------------------------------
void disableControlUnit() {
  digitalWrite(CU_EN, HIGH);   // active-low: HIGH disables
}

void enableControlUnit() {
  digitalWrite(CU_EN, LOW);
}

// -----------------------------------------------------------------------
// Bus output enable / disable
// -----------------------------------------------------------------------
void enableOutput() {
  digitalWrite(EN, HIGH);
}

void disableOutput() {
  digitalWrite(EN, LOW);
}

// -----------------------------------------------------------------------
// Address & instruction write — identical protocol to the AVR version
// -----------------------------------------------------------------------
void setAddress(unsigned int address) {
  digitalWrite(HL, address > 0xFF);
  putOut(address & 0xFF);
  delayMicroseconds(2);
  enableOutput();
  digitalWrite(MI, HIGH);
  delayMicroseconds(2);
  digitalWrite(MI, LOW);
  delayMicroseconds(2);
  disableOutput();
}

void writeInstruction(uint8_t instr) {
  putOut(instr);
  enableOutput();
  delayMicroseconds(2);
  digitalWrite(RI, HIGH);
  delayMicroseconds(2);
  digitalWrite(RI, LOW);
  delayMicroseconds(2);
  disableOutput();
}

// -----------------------------------------------------------------------
// Main loop — serial upload protocol (compatible with uploader.py)
// -----------------------------------------------------------------------
uint8_t buffer[512];

void loop() {
  if (Serial.available()) {
    int p_size = Serial.readBytes(buffer, sizeof(buffer));
    Serial.write(0x00);    // ack

    pulseReset();
    disableControlUnit();

    delayMicroseconds(2);
    enableClock();
    delayMicroseconds(2);

    for (int i = 0; i < p_size; i++) {
      setAddress(i);
      delayMicroseconds(2);
      writeInstruction(buffer[i]);
      delayMicroseconds(2);
    }

    disableClock();
    digitalWrite(HL, LOW);
    pulseReset();
    enableControlUnit();
  }
  disableOutput();
}
