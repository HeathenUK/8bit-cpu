/* Diagnostic: same init as keypad_lcd, but bypass __keypad_scan.
 * Drive row 0 LOW directly, read ORB, edge-emit changed values via OI.
 * If we see col bits flipping when keys on row 0 are pressed, scan
 * itself is fine and the bug is elsewhere. If raw ORB stays 0xF0, the
 * LCD is electrically pinning the col pull-ups (or a shadow is wrong). */
void main(void) {
    unsigned char r;
    unsigned char prev;

    i2c_init();
    lcd_init();
    keypad_init();

    /* Drive row 0 LOW (PA4 LOW, PA5-7 HIGH). VIA register 1 = ORA. */
    exw(0xE0, 1, 0);

    prev = 0;
    while (1) {
        r = exrw(0);             /* raw ORB read */
        r = r & 0xF0;            /* mask col bits */
        if (r != prev) {
            out(r);              /* edge-emit raw col pattern */
            prev = r;
        }
    }
}
