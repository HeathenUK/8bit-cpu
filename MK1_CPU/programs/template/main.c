/* Example program — emits a few values via OI and halts.
 *
 * `out(N)` writes N (0..255) to the MK1's output register, raising the
 * OI strobe so the host (or a 7-seg latch) can pick it up. `halt()` is
 * a soft-halt that stops the clock — the ESP32 firmware sees the HLT
 * opcode on the bus and ends the run.
 *
 * Try:
 *   make           - compile to main.asm
 *   make run       - upload + run, expect val=42 on the firmware response
 *   make sim       - run in the simulator, no hardware needed
 *   make help      - list every target
 */
void main(void) {
    out(0xDE); out(0xAD); out(0xBE);   /* sentinel */
    out(42);
    halt();
}
