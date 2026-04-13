// Test: read font data from EEPROM, display on LCD
// Font stored in AT24C32 EEPROM at compile-assigned addresses

eeprom unsigned char font[] = {
    0x3E, 0x51, 0x49, 0x45, 0x3E,  // 0
    0x00, 0x42, 0x7F, 0x40, 0x00,  // 1
    0x42, 0x61, 0x51, 0x49, 0x46,  // 2
};

void main() {
    i2c_init();
    i2c_bus_reset();

    // Read font[0] (should be 0x3E = 62)
    unsigned char val;
    val = font[0];
    out(val);
    halt();
}
