/* Minimal LCD/RTC path probe that avoids splitting main into many overlays. */
void main() {
    i2c_init();
    lcd_init();
    out(0xA1);

    unsigned char temp;
    temp = rtc_read_temp();
    out(0xA2);
    out(temp);

    lcd_char('O');
    lcd_char('K');
    out(0xA3);
    halt();
}
