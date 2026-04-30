/* Real-world cold-tier demo: 5 distinct C functions, each pushed cold.
 *
 * Each function emits a long unique OI signature so the runlog shows
 * the dispatcher correctly:
 *   - loads each function's bytes from AT24C512 on demand,
 *   - reuses the same code-page slot (slot bytes change per call),
 *   - jumps in, runs, returns through the dispatcher.
 *
 * Without `ee64`: 5 functions × ~30 B + main + helpers exceeds the
 * 250 B page-0 budget. With cold-tier all five live in flash and only
 * one is loaded into a 30 B code-page slot at any time.
 */

ee64 void show_a(void) {
    out(0xA0); out(0xA1); out(0xA2); out(0xA3); out(0xA4);
    out(0xA5); out(0xA6); out(0xA7); out(0xA8); out(0xA9);
}
ee64 void show_b(void) {
    out(0xB0); out(0xB1); out(0xB2); out(0xB3); out(0xB4);
    out(0xB5); out(0xB6); out(0xB7); out(0xB8); out(0xB9);
}
ee64 void show_c(void) {
    out(0xC0); out(0xC1); out(0xC2); out(0xC3); out(0xC4);
    out(0xC5); out(0xC6); out(0xC7); out(0xC8); out(0xC9);
}
ee64 void show_d(void) {
    out(0xD0); out(0xD1); out(0xD2); out(0xD3); out(0xD4);
    out(0xD5); out(0xD6); out(0xD7); out(0xD8); out(0xD9);
}
ee64 void show_e(void) {
    out(0xE0); out(0xE1); out(0xE2); out(0xE3); out(0xE4);
    out(0xE5); out(0xE6); out(0xE7); out(0xE8); out(0xE9);
}

void main(void) {
    i2c_init();
    show_a();
    show_b();
    show_c();
    show_d();
    show_e();
    halt();
}
