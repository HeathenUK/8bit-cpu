// Test EEPROM persistent storage: write font data, read back

eeprom unsigned char test_data[] = { 0xDE, 0xAD };

void main() {
    i2c_init();

    unsigned char val;
    val = test_data[0];
    out(val);
    halt();
}
