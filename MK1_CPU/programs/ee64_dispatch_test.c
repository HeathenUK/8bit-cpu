/* Phase 4 — verify the cold-call dispatcher (__cold_call).
 *
 * Populates the cold-tier metadata table at page3[0x10..0x12] for
 * helper-id 0 → (ee64 addr 0x0000, length 4), then `cold_call(0)`.
 * The dispatcher reads the table, bulk-loads from AT24C512 into
 * code[0xF0], jumps in.
 *
 * **PRE-REQUISITE**: ee64_run_test.c must run first to seed
 * ee64[0..3] with `ldi $a,0xAB; out; ret`. Phase 4 doesn't re-seed
 * because the page-write code blows the 250 B page-0 budget when
 * combined with the resident dispatcher + r2c_loop helpers.
 * Phase 5's hw_regression entry will handle seeding via the
 * partitioner's EEPROM upload path — no runtime page-write needed.
 *
 * Expected: out(0xAB) = 171. End-to-end dispatcher path verified.
 */
void main(void) {
    i2c_init();

    /* Seed metadata for cold-helper id 0:
     *   page3[0x10] = addr_hi (0)
     *   page3[0x11] = addr_lo (0)
     *   page3[0x12] = len     (4)
     */
    poke3(0, 0x10);
    poke3(0, 0x11);
    poke3(4, 0x12);

    cold_call(0);
    halt();
}
