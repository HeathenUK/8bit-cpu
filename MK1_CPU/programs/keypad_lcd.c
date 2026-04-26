/* Keypad → LCD: print the pressed key's character on the LCD.
 *
 * Hardware: 4x4 keypad on PA4-7 / PB4-7 (8K pull-ups), DFRobot V2.0
 * RGB LCD on I2C (PB0-1).
 *
 * Physical key layout:
 *
 *     C0  C1  C2  C3
 *    +---+---+---+---+
 *  R0| 1 | 2 | 3 | A |
 *  R1| 4 | 5 | 6 | B |
 *  R2| 7 | 8 | 9 | C |
 *  R3| * | 0 | # | D |
 *    +---+---+---+---+
 *
 * keypad_scan() returns 0..15 in row-major order; the 16-byte LUT
 * below maps that index to the printed character.
 *
 * Uses keypad_scan() (non-blocking) plus an edge detector instead of
 * keypad_wait(): scan is cheaper than wait+debounce for size, and
 * filtering "if k != prev" gives the same one-event-per-press feel.
 * AiP31068L auto-advances DDRAM, so each lcd_char() drops the next
 * character; we don't manage the cursor.
 */

unsigned char keymap[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D'
};

void main(void) {
    unsigned char k;
    unsigned char prev;

    i2c_init();
    lcd_init();
    keypad_init();
    /* delay_calibrate() not needed: keypad_scan() doesn't time-wait,
       lcd_init() uses an uncalibrated settling loop, and there's no
       tone/delay path here. Skipping calibrate also keeps boot fast. */

    prev = 0xFF;
    while (1) {
        k = keypad_scan();
        if (k != prev) {
            if (k != 0xFF) {
                lcd_char(keymap[k]);
            }
            prev = k;
        }
    }
}
