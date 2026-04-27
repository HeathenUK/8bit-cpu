/* SSD1327 Phase 3 tracer: render letter 'A' at cell (0,0). Splits the
 * window-setup and pixel-stream into helpers so they overlay-partition
 * cleanly within page-0's 250B limit. */
void window_cell_0_0(void) {
    i2c_start();
    i2c_send_byte(0x7A); i2c_send_byte(0x00);   /* addr + CMD_STREAM */
    i2c_send_byte(0x15); i2c_send_byte(0x00); i2c_send_byte(0x02);
    i2c_send_byte(0x75); i2c_send_byte(0x00); i2c_send_byte(0x06);
    i2c_stop();
}

void render_letter_a(void) {
    i2c_start();
    i2c_send_byte(0x7A); i2c_send_byte(0x40);   /* addr + DATA_STREAM */
    /* Letter 'A', 5x5 pixel art expanded to 4bpp (fg=0xF, bg=0x0):
     *   .XX..  X...X  XXXXX  X...X  X...X  (descender)  (line gap) */
    i2c_send_byte(0x0F); i2c_send_byte(0xFF); i2c_send_byte(0x00);
    i2c_send_byte(0xF0); i2c_send_byte(0x00); i2c_send_byte(0xF0);
    i2c_send_byte(0xFF); i2c_send_byte(0xFF); i2c_send_byte(0xF0);
    i2c_send_byte(0xF0); i2c_send_byte(0x00); i2c_send_byte(0xF0);
    i2c_send_byte(0xF0); i2c_send_byte(0x00); i2c_send_byte(0xF0);
    i2c_send_byte(0x00); i2c_send_byte(0x00); i2c_send_byte(0x00);
    i2c_send_byte(0x00); i2c_send_byte(0x00); i2c_send_byte(0x00);
    i2c_stop();
}

void main(void) {
    i2c_init();
    oled_init();
    window_cell_0_0();
    render_letter_a();
    while (1);
}
