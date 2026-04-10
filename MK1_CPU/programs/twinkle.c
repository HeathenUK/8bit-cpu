/* Simple tune test — minimal overlay usage */
void play_tune(void) {
    tone(262, 250); tone(262, 250);
    tone(392, 250); tone(392, 250);
    tone(440, 250); tone(440, 250);
    tone(392, 500);
    tone(349, 250); tone(349, 250);
    tone(330, 250); tone(330, 250);
    tone(294, 250); tone(294, 250);
    tone(262, 500);
}

void main(void) {
    i2c_init();
    delay_calibrate();
    ddra_imm(0x02);
    play_tune();
    out(0xDD);
    halt();
}
