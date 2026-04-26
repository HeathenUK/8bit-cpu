/* Force overlay mode without lcd_init: i2c_init + delay_calibrate +
 * keypad_init. delay_calibrate adds ~80B which should push __keypad_scan
 * into the overlay. If scan still works → overlay isn't the problem.
 * If scan breaks → overlay IS the problem (and lcd_init is innocent). */
void main(void) {
    unsigned char k;
    unsigned char prev;

    i2c_init();
    delay_calibrate();
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
