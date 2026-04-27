/* Force __keypad_scan into an overlay loaded via reload thunk:
 * do_scan() and other() both grow large enough that the partitioner
 * SCC-merge plus cold-helper-extraction puts __keypad_scan in its own
 * overlay, called from do_scan() via __ovrl_*. do_scan() then reports
 * each key change via OI.
 *
 * Used by keypad_overlay_repro.py as a sim test fixture for the
 * keypad-in-overlay path. On hardware this returns wrong/stuck values
 * (separate hardware bug, see WORKLOG); on the simulator it returns
 * the correct key indices, proving the software path is sound.
 */
unsigned char prev;
unsigned char filler[80];

void do_scan(void) {
    unsigned char k;
    k = keypad_scan();
    if (k != prev) {
        out(k);
        prev = k;
    }
    filler[0]++; filler[1]++; filler[2]++; filler[3]++;
    filler[4]++; filler[5]++; filler[6]++; filler[7]++;
    filler[8]++; filler[9]++; filler[10]++; filler[11]++;
}

void other(void) {
    filler[20]++; filler[21]++; filler[22]++; filler[23]++;
    filler[24]++; filler[25]++; filler[26]++; filler[27]++;
    filler[28]++; filler[29]++; filler[30]++; filler[31]++;
    filler[32]++; filler[33]++; filler[34]++; filler[35]++;
}

void main(void) {
    keypad_init();
    out(0xDE); out(0xAD); out(0xBE);
    prev = 0xFF;
    while (1) { do_scan(); other(); }
}
