/* lcd_hello.c — write "Hi" to the LCD.
 *
 * Originally targeted HD44780 + PCF8574 on the 82C55 PPI hardware.
 * Refactored 2026-04-26 for the current MK1 target: W65C22S (VIA) +
 * DFRobot RGB V2.0 (AiP31068L LCD at 0x3E + PCA9633 backlight at 0x2D).
 * The compiler's lcd_init/lcd_char builtins handle the AiP31068L
 * direct-I2C protocol; no user-defined nibble/cmd/send-byte plumbing
 * needed.
 */
void main() {
    i2c_init();
    lcd_init();
    lcd_char('H');
    lcd_char('i');
    halt();
}
