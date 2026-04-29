/* Phase 5 — verify the partitioner-emitted cold-tier path end-to-end.
 *
 * Declares a cold helper as `ee64 unsigned char` array. The compiler:
 *   - allocates a cold-helper id (0, by declaration order),
 *   - assigns an ee64 address in the AT24C512 image,
 *   - emits the bytes into a `section ee64` block (uploaded by ESP32
 *     to the 64KB chip during UPLOAD),
 *   - emits init code in the kernel preamble that populates the
 *     dispatcher's metadata table at page3[COLD_TABLE_BASE..].
 *
 * Then `cold_call(0)` triggers the runtime dispatcher (Phase 4)
 * which loads the helper from AT24C512 into the cold slot at
 * code[0xF0] and jumps in.
 *
 * Helper bytes are `ldi $a,0xCD; out; ret`
 *   = 0x38 0xCD 0x06 0x6C  (4 bytes, fits in 16 B slot).
 *
 * Expected: out(0xCD) = 205. No prior-run dependency — the bytes
 * are uploaded fresh by ESP32 from the compiler's ee64 image.
 */
ee64 unsigned char cold_helper_0[] = {0x38, 0xCD, 0x06, 0x6C};

void main(void) {
    i2c_init();          /* dispatcher uses I²C; needs the bus initialized */
    cold_call(0);
    halt();
}
