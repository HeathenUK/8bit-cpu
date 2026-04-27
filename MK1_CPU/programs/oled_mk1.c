/* SSD1327 Phase 3: render "MK1" using oled_putc() builtin. Each
 * letter is wrapped in its own user function so the overlay
 * partitioner can move them off page 0 (a single oled_putc call is
 * already ~150 B of inline code). */
void put_M(void) { oled_putc('M', 0, 0); }
void put_K(void) { oled_putc('K', 1, 0); }
void put_1(void) { oled_putc('1', 2, 0); }

void main(void) {
    i2c_init();
    oled_init();
    put_M();
    put_K();
    put_1();
    while (1);
}
