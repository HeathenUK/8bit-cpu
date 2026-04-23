/* Tone probe that preserves the init-compatible prefix. */
void main(void) {
    i2c_init();
    delay_calibrate();
    out(0xA2);
    out(peek3(240));
    tone(1000, 20);
    out(0xB1);
    halt();
}
