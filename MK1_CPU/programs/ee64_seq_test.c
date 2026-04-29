/* Phase 2 — verify AT24C512 sequential read works.
 *
 * Page-write 4 bytes at addr 0, then sequential-read them back. Each
 * byte is emitted via OI so RUNLOG sees the actual values streamed
 * by the chip. Expected output sequence: 0x11, 0x22, 0x33, 0x44.
 *
 * Sequential read on the AT24C512 auto-increments the internal
 * address pointer after each clocked-out byte; the master ACKs to
 * keep going, NACKs the last byte to release the bus before STOP.
 */
void main(void) {
    i2c_init();

    /* ── Page write 4 bytes at addr 0 ── */
    i2c_start();
    i2c_send_byte(0xA0);
    i2c_send_byte(0x00);     /* addr hi */
    i2c_send_byte(0x00);     /* addr lo */
    i2c_send_byte(0x11);
    i2c_send_byte(0x22);
    i2c_send_byte(0x33);
    i2c_send_byte(0x44);
    i2c_stop();

    i2c_wait_ack(0xA0);      /* ACK-poll until write cycle completes */

    /* ── Set address pointer to 0, then sequential read ── */
    i2c_start();
    i2c_send_byte(0xA0);
    i2c_send_byte(0x00);
    i2c_send_byte(0x00);
    i2c_repeated_start();
    i2c_send_byte(0xA1);

    out(i2c_read_byte()); i2c_ack();     /* expect 0x11 */
    out(i2c_read_byte()); i2c_ack();     /* expect 0x22 */
    out(i2c_read_byte()); i2c_ack();     /* expect 0x33 */
    out(i2c_read_byte()); i2c_nack();    /* expect 0x44 (last byte) */
    i2c_stop();

    halt();
}
