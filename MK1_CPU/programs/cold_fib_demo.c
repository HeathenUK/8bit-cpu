/* Cold-tier demo, sized to fit in single shim page (≤32 B). */
ee64 void fib_demo(void) {
    unsigned char a;
    unsigned char b;
    unsigned char t;
    a = 0;
    b = 1;
    out(a);
    out(b);
    t = a + b; out(t); a = b; b = t;   /* 1 */
    t = a + b; out(t);                 /* 2 */
}

void main(void) {
    i2c_init();
    fib_demo();
    halt();
}
