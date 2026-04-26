/* Keypad wiring smoke test — no compiler/keypad-builtin work needed.
 *
 * Sets up VIA directly:
 *   DDRA = 0xF0  → PA4-7 outputs (rows), PA0-3 inputs (PA0=SQW unchanged)
 *   DDRB = 0x00  → PB all inputs (cols on PB4-7, others irrelevant here)
 *   ORA  = 0x00  → drive ALL rows LOW (pull every key's row down)
 *
 * Reads ORB in a loop and emits via OI. With 8K pull-ups to VCC on
 * PB4-7, expected behaviour:
 *   - no key pressed     → PB[4-7] = 1111 → ORB read & 0xF0 = 0xF0
 *   - any key on COL0    → PB4 = 0       → ORB & 0xF0 = 0xE0
 *   - any key on COL1    → PB5 = 0       → ORB & 0xF0 = 0xD0
 *   - any key on COL2    → PB6 = 0       → ORB & 0xF0 = 0xB0
 *   - any key on COL3    → PB7 = 0       → ORB & 0xF0 = 0x70
 *
 * (Lower nibble bits PB0-3 are floating in this test — should ignore.)
 *
 * Sentinel out(0xDE/0xAD/0xBE) + out(START) at start so we can spot
 * the boundary between "before user pressed anything" and the
 * column-state stream.
 *
 * Drive: a tight infinite loop. Run via RUNNB:large_budget,1 — user
 * presses keys during the run, ESP32 captures the latest 256 OI
 * events (ring buffer), and the dedup'd trace shows column states
 * as they changed.
 */
void main(void) {
    /* Direction registers FIRST. Order matters: setting outputs HIGH
     * before flipping DDR-to-output prevents transient drive of stale
     * ORA bits (which would matter if we cared about other PA pins). */
    exw(0xF0, 3, 0);     /* DDRA = 0xF0 */
    exw(0x00, 2, 0);     /* DDRB = 0x00 (PB all input) */
    exw(0x00, 1, 0);     /* ORA = 0 → all rows LOW */

    /* Sentinel + start marker so trace anchor is unambiguous */
    out(0xDE); out(0xAD); out(0xBE);
    out(0xA5);

    /* Read columns forever. Each unique value captured by OI ring. */
    while (1) {
        out(exrw(0));
    }
}
