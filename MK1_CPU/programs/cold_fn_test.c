/* Phase 6 — `ee64`-tagged C functions land cold automatically.
 *
 * `ee64 void chime(void) { out(0xCE); }` is a normal C function as
 * far as the user is concerned — body, signature, the works. The
 * compiler:
 *   - compiles the body to asm (with internal labels resolved to
 *     the cold slot's runtime address),
 *   - allocates a cold-helper id and an address in the AT24C512
 *     image,
 *   - emits the bytes in `section ee64` for ESP32 to upload,
 *   - emits init code that populates page3 metadata at startup,
 *   - rewrites the `chime();` call site below as
 *     `cold_call(0)` — through the runtime dispatcher.
 *
 * Expected: out(0xCE) = 206. End-to-end demonstration that
 * arbitrary C functions can be pushed cold without the user
 * hand-writing opcode arrays.
 */
ee64 void chime(void) {
    out(0xCE);
}

void main(void) {
    i2c_init();
    chime();    /* compiler rewrites this to cold_call(0) */
    halt();
}
