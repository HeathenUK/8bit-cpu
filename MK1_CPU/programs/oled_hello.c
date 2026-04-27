/* SSD1327 Phase 4a: oled_set_cursor() + oled_text() with compile-
 * time-tracked cursor. Each char in a string literal expands to
 * one oled_putc(c, x, y) at emit time, cursor advancing per char.
 *
 * Page-1 budget caps this at ~6-7 chars per program (each char
 * uses 27 B for its window-set + pixel table). Phase 4b will add
 * a runtime cursor in kstate + a render helper that does font
 * lookup at runtime, dropping per-char cost to ~5 B and unlocking
 * full 21-char lines + oled_printf with %d/%x. */
void main(void) {
    i2c_init();
    oled_init();
    oled_set_cursor(0, 0);
    oled_text("MK1");
    oled_set_cursor(0, 2);
    oled_text("HI");
    while (1);
}
