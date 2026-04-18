/* Display RTC temperature on LCD as "NN°C" */
void main() {
    i2c_init();
    lcd_init();
    unsigned char temp;
    temp = rtc_read_temp();
    /* Bus recovery: clears any slave confused by RTC transaction before
     * we write to LCD. Required when switching between slaves. */
    i2c_bus_reset();
    out(temp);
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
    halt();
}
