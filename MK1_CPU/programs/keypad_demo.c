/* Keypad scan demo using the new keypad_init() / keypad_scan() builtins.
 *
 * keypad_scan() returns a 0-15 key index (row*4 + col):
 *   0  1  2  3
 *   4  5  6  7
 *   8  9 10 11
 *  12 13 14 15
 *
 * 0xFF means no key currently pressed.
 *
 * This loop emits the key index via OI only when it changes, so the
 * 7-seg latches the current key value (or 0xFF when nothing is held).
 * Run via RUNNB to capture an edge-stream of presses/releases.
 */
void main(void) {
    unsigned char k;
    unsigned char prev;

    keypad_init();

    out(0xDE); out(0xAD); out(0xBE);
    out(0xA5);

    prev = 0xFF;
    while (1) {
        k = keypad_scan();
        if (k != prev) {
            out(k);
            prev = k;
        }
    }
}
