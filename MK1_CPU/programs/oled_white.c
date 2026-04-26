/* SSD1327 128x128 OLED bring-up. oled_fill(byte) initialises the
 * chip and fills the whole 8 KB GDDRAM with the supplied byte
 * (each byte is two 4 bpp pixels — 0x00 = black, 0xFF = white). */
void main(void) {
    i2c_init();
    oled_fill(0x00);   /* black — both 4bpp pixels at 0/15, via unrolled i2c_send_const */
    while (1);
}
