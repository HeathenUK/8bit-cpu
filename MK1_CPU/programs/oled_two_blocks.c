/* Two solid 6x7 blocks at cells (0,0) and (1,0) using manual i2c
 * sends. If both blocks render side-by-side correctly, the chip
 * respects 0x15/0x75 between transactions and the bug is in
 * __oled_putc_xfer. If block 2 stair-steps, the bug is in the
 * window-set protocol itself. */
void block_at_0_0(void) {
    i2c_start();
    i2c_send_byte(0x7A); i2c_send_byte(0x00);
    i2c_send_byte(0x15); i2c_send_byte(0); i2c_send_byte(2);
    i2c_send_byte(0x75); i2c_send_byte(0); i2c_send_byte(6);
    i2c_stop();
    i2c_start();
    i2c_send_byte(0x7A); i2c_send_byte(0x40);
    i2c_send_byte(0xFF); i2c_send_byte(0xFF); i2c_send_byte(0xFF);
    i2c_send_byte(0xFF); i2c_send_byte(0xFF); i2c_send_byte(0xFF);
    i2c_send_byte(0xFF); i2c_send_byte(0xFF); i2c_send_byte(0xFF);
    i2c_send_byte(0xFF); i2c_send_byte(0xFF); i2c_send_byte(0xFF);
    i2c_send_byte(0xFF); i2c_send_byte(0xFF); i2c_send_byte(0xFF);
    i2c_send_byte(0xFF); i2c_send_byte(0xFF); i2c_send_byte(0xFF);
    i2c_send_byte(0xFF); i2c_send_byte(0xFF); i2c_send_byte(0xFF);
    i2c_stop();
}

void block_at_1_0(void) {
    i2c_start();
    i2c_send_byte(0x7A); i2c_send_byte(0x00);
    i2c_send_byte(0x15); i2c_send_byte(3); i2c_send_byte(5);
    i2c_send_byte(0x75); i2c_send_byte(0); i2c_send_byte(6);
    i2c_stop();
    i2c_start();
    i2c_send_byte(0x7A); i2c_send_byte(0x40);
    i2c_send_byte(0x88); i2c_send_byte(0x88); i2c_send_byte(0x88);
    i2c_send_byte(0x88); i2c_send_byte(0x88); i2c_send_byte(0x88);
    i2c_send_byte(0x88); i2c_send_byte(0x88); i2c_send_byte(0x88);
    i2c_send_byte(0x88); i2c_send_byte(0x88); i2c_send_byte(0x88);
    i2c_send_byte(0x88); i2c_send_byte(0x88); i2c_send_byte(0x88);
    i2c_send_byte(0x88); i2c_send_byte(0x88); i2c_send_byte(0x88);
    i2c_send_byte(0x88); i2c_send_byte(0x88); i2c_send_byte(0x88);
    i2c_stop();
}

void main(void) {
    i2c_init();
    oled_init();
    block_at_0_0();
    block_at_1_0();
    while (1);
}
