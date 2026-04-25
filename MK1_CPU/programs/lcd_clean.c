/*
 * lcd_clean.c — canonical reference: read DS3231 temperature, display on LCD.
 *
 * Writes the Celsius temperature to the top-left of a 16x2 LCD connected via
 * the DFRobot RGB V2.0 (AiP31068L + PCA9633) on the I2C bus, reading the
 * DS3231 RTC on the same I2C bus.
 *
 * Demonstrates the idioms the MK1 toolchain currently handles cleanly:
 *
 *   1. `i2c_init()` + `lcd_init()` once. The device-helper contract keeps
 *      the bus idle between device calls — no manual `i2c_bus_reset()`.
 *
 *   2. `rtc_read_temp()` uses repeated-START internally for an atomic
 *      register-pointer-set + data-read transaction.
 *
 *   3. `printf("%d\xDFC", t)` lowers to the compact `lcd_temp_u8` helper
 *      (~33 B shared) — emits decimal + degree glyph + 'C' without
 *      pulling in the full generic decimal helper. `%d` is "minimum
 *      digits"; for RTC temps (always >=10 in normal operation) the
 *      output is identical to a hand-rolled 2-digit decimal.
 *
 *   4. `halt()` emits `hlt` + infinite-loop trap so the display stays
 *      stable on ESP32-driven clock. On auto (555) clock, HLT gates the
 *      oscillator directly; the trap is harmless there.
 *
 * Expected LCD:  "NN°C"
 */

void main() {
    i2c_init();
    lcd_init();

    unsigned char t = rtc_read_temp();

    /* Explicit clear after a non-lcd_init statement so the peephole
     * doesn't elide it. Reset doesn't re-run lcd_init() — without an
     * explicit clear the display would accumulate stale content. */
    lcd_clear();

    printf("%d\xDFC", t);

    halt();
}
