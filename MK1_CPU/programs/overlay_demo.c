/* EEPROM overlay demo — multiple independent overlay functions.
 * Each function is called from main (no cross-overlay calls).
 *
 * Computes: ((input * 3) + 10) * 2 where input = peek3(0)
 */

unsigned char multiply(unsigned char a, unsigned char b) {
    unsigned char result = 0;
    while (b > 0) {
        result = result + a;
        b = b - 1;
    }
    return result;
}

unsigned char add_offset(unsigned char x, unsigned char offset) {
    return x + offset;
}

unsigned char double_it(unsigned char x) {
    return x + x;
}

void main(void) {
    i2c_init();
    unsigned char input = peek3(0);
    unsigned char step1 = multiply(input, 3);
    unsigned char step2 = add_offset(step1, 10);
    unsigned char step3 = double_it(step2);
    out(step3);
    halt();
}
