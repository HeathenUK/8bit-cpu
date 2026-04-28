unsigned char keymap[16] = {
    5,  6,  7,  16,
    8,  9,  10, 17,
    11, 12, 13, 18,
    0,  4,  0,  19
};
unsigned char display[1];
unsigned char prev_key;

void show(unsigned char idx) {
    display[0] = idx;
    /* (col, row) = (8, 4): roughly screen-centre on the 16×8 cell grid */
    oled_print_buf(display, 1, 8, 4);
}

void poll_once(void) {
    unsigned char k;
    k = keypad_scan();
    if (k != prev_key) {
        if (k != 0xFF) show(keymap[k]);
        prev_key = k;
    }
}

void main(void) {
    i2c_init();
    oled_init();
    oled_clear();          /* wipe residual framebuffer from any prior program */
    keypad_init();
    prev_key = 0xFF;
    while (1) poll_once();
}
