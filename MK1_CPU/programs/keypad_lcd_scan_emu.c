/* Emulates __keypad_scan in C source: 4-row scan with tight exw/exrw,
 * edge-emit on row hit. If this works in the lcd-init context but
 * __keypad_scan doesn't, the bug is in the helper/overlay path. */
void main(void) {
    unsigned char r;
    unsigned char prev;

    i2c_init();
    lcd_init();
    keypad_init();

    prev = 0xFF;
    while (1) {
        /* Row 0 */
        exw(0xE0, 1, 0);
        r = exrw(0) & 0xF0;
        if (r != 0xF0) {
            if (r != prev) { out(r); prev = r; }
            continue;
        }
        /* Row 1 */
        exw(0xD0, 1, 0);
        r = exrw(0) & 0xF0;
        if (r != 0xF0) {
            if (r != prev) { out(r | 1); prev = r; }
            continue;
        }
        /* Row 2 */
        exw(0xB0, 1, 0);
        r = exrw(0) & 0xF0;
        if (r != 0xF0) {
            if (r != prev) { out(r | 2); prev = r; }
            continue;
        }
        /* Row 3 */
        exw(0x70, 1, 0);
        r = exrw(0) & 0xF0;
        if (r != 0xF0) {
            if (r != prev) { out(r | 3); prev = r; }
            continue;
        }
        /* No key */
        if (prev != 0xFF) { out(0xFF); prev = 0xFF; }
        exw(0xF0, 1, 0);
    }
}
