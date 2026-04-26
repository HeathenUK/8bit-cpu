/* Keypad row-scan test (edge-emit version).
 *
 * Cycles through 4 rows, only emits OI when the value for that row
 * changes vs the prior cycle. Idle stream produces no OI events;
 * press/release shows a clean transition pair. Ring buffer (last
 * 256 events) thus accumulates the most recent press/release edges
 * across the entire run window.
 *
 * Encoding per emitted byte:
 *   bits 7..4 = row (0..3)
 *   bits 3..0 = col-state nibble (0xF idle; 0xE=C0, 0xD=C1, 0xB=C2, 0x7=C3)
 */
void main(void) {
    unsigned char p0; unsigned char p1; unsigned char p2; unsigned char p3;
    unsigned char c;

    exw(0xF0, 3, 0);     /* DDRA = rows output */
    exw(0x00, 2, 0);     /* DDRB = all input */

    p0 = 0x0F; p1 = 0x1F; p2 = 0x2F; p3 = 0x3F;

    out(0xDE); out(0xAD); out(0xBE);
    out(0xA5);

    while (1) {
        exw(0xE0, 1, 0);
        c = 0x00 | (exrw(0) >> 4);
        if (c != p0) { out(c); p0 = c; }

        exw(0xD0, 1, 0);
        c = 0x10 | (exrw(0) >> 4);
        if (c != p1) { out(c); p1 = c; }

        exw(0xB0, 1, 0);
        c = 0x20 | (exrw(0) >> 4);
        if (c != p2) { out(c); p2 = c; }

        exw(0x70, 1, 0);
        c = 0x30 | (exrw(0) >> 4);
        if (c != p3) { out(c); p3 = c; }
    }
}
