// MK1 EEPROM + RTC demo
// Reads temperature, writes it to EEPROM, reads it back, displays

void main(void) {
    i2c_init();
    
    // Read temperature from DS3231
    unsigned char temp = rtc_read_temp();
    
    // Store it in EEPROM at address 0x0B00
    eeprom_write_byte(0x0B, 0x00, temp);
    
    // Read it back from EEPROM
    unsigned char val = eeprom_read_byte(0x0B, 0x00);
    
    // Display on 7-segment
    out(val);
    halt();
}
