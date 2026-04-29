/* Phase 1 — verify AT24C512 (64 KB EEPROM) at I²C address 0x50 responds.
 *
 * Writes 0xA5 to ee64[0x0000], waits for the write cycle, reads it
 * back, outputs the value via OI. Expected: out(0xA5).
 *
 * Differences from the AT24C32 (4 KB) at 0x57:
 *   - Device address: 0x50 → write byte 0xA0, read byte 0xA1.
 *   - Memory address: 16 bits (2 bytes) instead of 12 bits.
 *
 * If this prints something other than 0xA5 (or hangs / outputs nothing),
 * the chip isn't responding correctly and the cold-tier work doesn't
 * have a foundation. Diagnostics:
 *   - Different value → bit-bang timing mismatch or chip issue.
 *   - No output → bus locked (NACK on address byte, infinite ack-poll).
 *   - 0xFF → chip ACKed but data is uninitialised / write didn't stick.
 */
void main(void) {
    i2c_init();

    /* ── Write 0xA5 to ee64[0x0000] ── */
    i2c_start();
    i2c_send_byte(0xA0);   /* device addr 0x50, write */
    i2c_send_byte(0x00);   /* memory addr hi */
    i2c_send_byte(0x00);   /* memory addr lo */
    i2c_send_byte(0xA5);   /* data byte */
    i2c_stop();

    /* ACK-poll until the chip finishes its internal write cycle (≤5 ms).
     * Cheaper than delay(10) — no __delay_cal helper, no calibration
     * dependency, returns as soon as the chip is actually ready. */
    i2c_wait_ack(0xA0);

    /* ── Random read of ee64[0x0000] ── */
    i2c_start();
    i2c_send_byte(0xA0);
    i2c_send_byte(0x00);
    i2c_send_byte(0x00);
    i2c_repeated_start();
    i2c_send_byte(0xA1);   /* device addr 0x50, read */
    unsigned char x;
    x = i2c_read_byte();
    i2c_nack();
    i2c_stop();

    out(x);                /* expected: 0xA5 (= 165) */
    halt();
}
