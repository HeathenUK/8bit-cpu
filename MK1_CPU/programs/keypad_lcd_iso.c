/* Isolation: same init as keypad_lcd, but no lcd_char in the loop.
 * Tests whether i2c_init + lcd_init alone break keypad_scan. */
void main(void) {
    unsigned char k;
    unsigned char prev;

    i2c_init();
    lcd_init();
    keypad_init();

    prev = 0xFF;
    while (1) {
        k = keypad_scan();
        if (k != prev) {
            out(k);
            prev = k;
        }
    }
}
