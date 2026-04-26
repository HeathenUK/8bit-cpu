/* keypad_wait() demo — blocks until press+release, returns key index 0-15.
 *
 * Each press cycle (press → 5ms debounce → release) → one OI emission.
 * Debounce uses calibrated __delay_Nms; requires delay_calibrate().
 *
 * Compare with keypad_demo.c which uses non-blocking keypad_scan().
 */
void main(void) {
    unsigned char k;

    i2c_init();
    delay_calibrate();
    keypad_init();

    out(0xDE); out(0xAD); out(0xBE);
    out(0xA5);

    while (1) {
        k = keypad_wait();
        out(k);
    }
}
