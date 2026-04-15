unsigned char read_temp() {
    unsigned char t;
    t = rtc_read_temp();
    return t;
}

void info_screen(unsigned char t) {
    lcd_cmd(0x01);
    lcd_print("MK1 8-bit CPU");
    lcd_cmd(0xC0);
    unsigned char tens;
    unsigned char ones;
    tens = 0;
    ones = t;
    while (ones >= 10) { ones = ones - 10; tens = tens + 1; }
    if (tens > 0) { lcd_char(tens + 48); }
    lcd_char(ones + 48);
    lcd_char(0xDF);
    lcd_print("C  Running!");
}

void main() {
    i2c_init();
    lcd_init();
    unsigned char t;
    t = read_temp();
    out(t);
    info_screen(t);
    halt();
}
