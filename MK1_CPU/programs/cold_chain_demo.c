/* Cold→cold chaining test: helper A calls helper B from inside its body.
 *
 * Verifies that:
 *   - main calls A → A's bytes loaded into slot, A runs
 *   - A's body issues `b()` → __cold_call(B) → B's bytes overwrite slot
 *   - B runs, returns
 *   - dispatcher reloads A's bytes back into slot
 *   - A's next instruction (after the call) executes correctly
 *   - A returns → dispatcher restores sentinel, returns to main
 *   - main halts
 *
 * Each helper outputs distinctive sentinels so the OI history shows the
 * exact interleaving.
 *
 * Expected OI sequence: 0xA1 (A start), 0xB1 0xB2 (B), 0xA2 (A after call)
 *                       = decimal [161, 177, 178, 162]
 */
ee64 void b(void) {
    out(0xB1);
    out(0xB2);
}

ee64 void a(void) {
    out(0xA1);
    b();        /* compiler rewrites to cold_call(B's id) */
    out(0xA2);
}

void main(void) {
    i2c_init();
    a();
    halt();
}
