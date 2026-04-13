// EEPROM read — exactly matching hand-written sequence, no bus_reset

void main() {
    i2c_init();
    // No bus_reset — just STOP
    i2c_stop();

    i2c_start();
    i2c_send_byte(0xAE);
    i2c_send_byte(0x00);
    i2c_send_byte(0x00);
    i2c_stop();

    i2c_start();
    i2c_send_byte(0xAF);
    unsigned char val;
    val = i2c_read_byte();
    i2c_nack();
    i2c_stop();
    out(val);
    halt();
}
