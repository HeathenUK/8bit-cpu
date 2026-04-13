// HD44780 LCD temperature display via PCF8574 I2C on VIA

void main() {
    i2c_init();
    i2c_bus_reset();
    lcd_init();

    unsigned char temp;
    temp = rtc_read_temp();
    out(temp);

    unsigned char tens;
    unsigned char ones;
    tens = 0;
    ones = temp;
    while (ones >= 10) {
        ones = ones - 10;
        tens = tens + 1;
    }

    if (tens > 0) {
        lcd_char(tens + 48);
    }
    lcd_char(ones + 48);
    lcd_char(0xDF);
    lcd_char('C');

    halt();
}
