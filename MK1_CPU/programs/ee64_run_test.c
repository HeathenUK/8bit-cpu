/* Phase 3 — verify the cold-tier round-trip end-to-end.
 *
 * Page-writes a 4-byte subroutine `ldi $a,0xAB; out; ret`
 * (= 0x38, 0xAB, 0x06, 0x6C) to ee64[0..3] in one I²C transaction,
 * bulk-reads it into a high code-page address via ee64_read_to_code,
 * then jal's there.
 *
 * Expected: a single out(0xAB) = 171.
 *
 * Page-write rather than 4× ee64_write_byte for code-size — each
 * ee64_write_byte costs ~60B (full START/STOP framing + ACK poll);
 * a single page-write is one START/STOP for all 4 bytes.
 */
void main(void) {
    i2c_init();

    /* Page-write the 4 executable bytes to ee64[0x0000..0x0003]. */
    i2c_start();
    i2c_send_byte(0xA0);     /* AT24C512 write address */
    i2c_send_byte(0x00);     /* addr hi */
    i2c_send_byte(0x00);     /* addr lo */
    i2c_send_byte(0x38);     /* ldi $a, ... */
    i2c_send_byte(0xAB);     /* ... 0xAB    */
    i2c_send_byte(0x06);     /* out          */
    i2c_send_byte(0x6C);     /* ret          */
    i2c_stop();
    i2c_wait_ack(0xA0);      /* wait for write cycle */

    /* Cold-load the 4 bytes into code[240] and jump. */
    ee64_read_to_code(0, 240, 4);
    call_code(240);

    halt();
}
