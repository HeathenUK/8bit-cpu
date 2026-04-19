/* Display RTC temperature on LCD as "NN°C" */
void main() {
    i2c_init();
    lcd_init();
    unsigned char temp;
    temp = rtc_read_temp();
    /* Bus recovery: clears any slave confused by RTC transaction before
     * we write to LCD. Required when switching between slaves. */
    i2c_bus_reset();
    /* Note: lcd_init already clears the display (0x01 is in its init sequence),
     * so an explicit lcd_clear() here is redundant and blows code budget. */
    unsigned char tens;
    tens = 0;
    while (temp >= 10) {
        temp = temp - 10;
        tens = tens + 1;
    }
    lcd_char(tens + 48);
    lcd_char(temp + 48);
    lcd_char(0xDF);
    lcd_char('C');
    /* out after lcd_chars so RUN captures temp once display is drawn. */
    out(temp);
    halt();
}
