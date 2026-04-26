/* Isolation: i2c_init + keypad_init only — no lcd_init. */
void main(void) {
    unsigned char k;
    unsigned char prev;

    i2c_init();
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
