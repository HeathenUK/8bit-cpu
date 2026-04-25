/* Display RTC temperature on LCD as "N°C" or "NN°C".
 * The compiler lowers `printf("%d\xDFC", t)` to `lcd_temp_u8(t)`,
 * a compact helper (~33 B shared) emitting decimal + degree + 'C'.
 * Note: printf's %d is "minimum digits", so a temp of 5 prints as
 * "5°C" not "05°C". For RTC temps (always >=10 in normal operation)
 * this matches the prior hand-rolled output. */
void main() {
    i2c_init();
    lcd_init();
    unsigned char temp = rtc_read_temp();
    i2c_bus_reset();
    printf("%d\xDFC", temp);
    out(temp);
    halt();
}
