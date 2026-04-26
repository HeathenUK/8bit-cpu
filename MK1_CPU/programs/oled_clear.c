/* SSD1327 Phase 2: explicit oled_init() + oled_clear(). Demonstrates
 * the split API — init runs once, clear streams 8192 × 0x00 to GDDRAM. */
void main(void) {
    i2c_init();
    oled_init();
    oled_clear();
    while (1);
}
