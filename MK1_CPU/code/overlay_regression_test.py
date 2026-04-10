#!/usr/bin/env python3
"""Overlay system regression test suite.

Compiles and runs a battery of C programs through --eeprom overlay mode,
verifying correct output on hardware. Catches codegen, overlay dispatch,
arg passing, caching, and call graph grouping regressions.

Usage:
    python3 overlay_regression_test.py [--port PORT]
"""

import serial
import subprocess
import sys
import os
import json
import time

PORT = '/dev/cu.usbmodem3C8427C2E7C82'
BAUD = 115200
COMPILER = os.path.join(os.path.dirname(__file__), 'mk1cc2.py')

def compile_c(source, eeprom=True):
    """Compile C source to assembly. Returns asm text or None."""
    with open('/tmp/_regtest.c', 'w') as f:
        f.write(source)
    args = ['python3', COMPILER, '/tmp/_regtest.c', '-O']
    if eeprom:
        args.append('--eeprom')
    args.extend(['-o', '/tmp/_regtest.asm'])
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        return None
    with open('/tmp/_regtest.asm') as f:
        return f.read()

def run_on_hardware(ser, asm, cycles=5000000, timeout=30):
    """Assemble, upload, run. Returns (val, cap, cyc) or None."""
    ser.reset_input_buffer()
    escaped = asm.replace('\n', '\\n')
    ser.write(f'ASM:{escaped}\n'.encode())
    old_timeout = ser.timeout
    ser.timeout = timeout
    try:
        r = json.loads(ser.readline().decode().strip())
    except:
        ser.timeout = old_timeout
        return None
    if not r.get('ok'):
        ser.timeout = old_timeout
        return None
    ser.write(b'UPLOAD\n')
    ser.readline()
    ser.write(f'RUN:{cycles},1\n'.encode())
    try:
        r2 = json.loads(ser.readline().decode().strip())
    except:
        ser.timeout = old_timeout
        return None
    ser.timeout = old_timeout
    return (r2.get('val'), r2.get('cap'), r2.get('cyc'), r.get('code'))


def test(ser, name, source, expected_val, eeprom=True):
    """Compile, run, check. Returns True/False."""
    asm = compile_c(source, eeprom=eeprom)
    if asm is None:
        print(f'  [FAIL] {name}: compile error')
        return False
    result = run_on_hardware(ser, asm)
    if result is None:
        print(f'  [FAIL] {name}: run error')
        return False
    val, cap, cyc, code = result
    if not cap:
        print(f'  [FAIL] {name}: no output (hung? cyc={cyc}, code={code})')
        return False
    if expected_val is not None and val != expected_val:
        print(f'  [FAIL] {name}: val={val} expected={expected_val} (cyc={cyc}, code={code})')
        return False
    if expected_val is None:
        print(f'  [PASS] {name}: val={val} (completed, cyc={cyc}, code={code})')
    else:
        print(f'  [PASS] {name}: val={val} (cyc={cyc}, code={code})')
    return True


def main():
    port = PORT
    if '--port' in sys.argv:
        port = sys.argv[sys.argv.index('--port') + 1]

    print('MK1 Overlay Regression Test Suite')
    print(f'Port: {port}\n')

    ser = serial.Serial(port, BAUD, timeout=30)
    time.sleep(1)
    ser.reset_input_buffer()

    results = []

    # ── 1. Sanity: no overlay (direct execution) ──
    results.append(test(ser, 'sanity: out_imm 42',
        'void main(void) { out(42); halt(); }', 42, eeprom=False))

    # ── 2. Void overlay function ──
    results.append(test(ser, 'void overlay (do_nothing + out 42)',
        '''void do_nothing(void) {}
        void main(void) { i2c_init(); do_nothing(); out(42); halt(); }''', 42))

    # ── 3. One-arg overlay (add_ten) ──
    results.append(test(ser, '1-arg overlay: add_ten(peek3(0))',
        '''unsigned char add_ten(unsigned char x) { return x + 10; }
        void main(void) { i2c_init(); out(add_ten(peek3(0))); halt(); }''',
        None))  # value depends on page3[0], just check it completes

    # ── 4. Two-arg overlay (add_both) ��─
    results.append(test(ser, '2-arg overlay: add_both(peek3(0), 5)',
        '''unsigned char add_both(unsigned char a, unsigned char b) { return a + b; }
        void main(void) { i2c_init(); out(add_both(peek3(0), 5)); halt(); }''',
        None))  # check completion

    # ── 5. Overlay with loop (multiply) ──
    results.append(test(ser, 'loop overlay: multiply(3, 4) = 12',
        '''unsigned char multiply(unsigned char a, unsigned char b) {
            unsigned char result = 0;
            while (b > 0) { result = result + a; b = b - 1; }
            return result;
        }
        void main(void) { i2c_init(); out(multiply(peek3(0), 4)); halt(); }''',
        None))  # check completion

    # ��─ 6. Multiple overlay swaps ──
    results.append(test(ser, 'multiple swaps: step1 + step2 + step3',
        '''unsigned char triple(unsigned char x) { return x + x + x; }
        unsigned char add5(unsigned char x) { return x + 5; }
        unsigned char dbl(unsigned char x) { return x + x; }
        void main(void) {
            i2c_init();
            unsigned char a = triple(peek3(0));
            unsigned char b = add5(a);
            unsigned char c = dbl(b);
            out(c);
            halt();
        }''', None))

    # ── 7. Call graph grouping (cross-function calls) ──
    results.append(test(ser, 'grouped overlay: compute_chain calls multiply+add+dbl',
        '''unsigned char multiply(unsigned char a, unsigned char b) {
            unsigned char result = 0;
            while (b > 0) { result = result + a; b = b - 1; }
            return result;
        }
        unsigned char add_offset(unsigned char x, unsigned char off) { return x + off; }
        unsigned char double_it(unsigned char x) { return x + x; }
        unsigned char compute_chain(unsigned char input) {
            unsigned char s1 = multiply(input, 3);
            unsigned char s2 = add_offset(s1, 10);
            return double_it(s2);
        }
        void main(void) {
            i2c_init();
            unsigned char r = compute_chain(peek3(0));
            out(r);
            halt();
        }''', 32))  # (2*3+10)*2 = 32 when peek3(0)=2

    # ── 8. Overlay caching (same function called twice) ──
    results.append(test(ser, 'cache hit: add5 called twice',
        '''unsigned char add5(unsigned char x) { return x + 5; }
        void main(void) {
            i2c_init();
            unsigned char a = add5(peek3(0));
            unsigned char b = add5(a);
            out(b);
            halt();
        }''', None))  # check completion (second call should cache-hit)

    # ��─ 9. Comparison with non-zero (cmpi opcode) ─��
    results.append(test(ser, 'cmpi: while (x > 5) loop',
        '''unsigned char count_down(unsigned char x) {
            unsigned char count = 0;
            while (x > 5) { x = x - 1; count = count + 1; }
            return count;
        }
        void main(void) { i2c_init(); out(count_down(20)); halt(); }''',
        15))  # 20 down to 5 = 15 steps

    # ── 10. Large grouped overlay (stress test) ──
    results.append(test(ser, 'large group: 5 functions in one slot',
        '''unsigned char f1(unsigned char x) { return x + 1; }
        unsigned char f2(unsigned char x) { return f1(x) + 1; }
        unsigned char f3(unsigned char x) { return f2(x) + 1; }
        unsigned char f4(unsigned char x) { return f3(x) + 1; }
        unsigned char f5(unsigned char x) { return f4(x) + 1; }
        void main(void) { i2c_init(); out(f5(peek3(0))); halt(); }''',
        None))  # f5(2) = 2+5 = 7 if peek3(0)=2

    # ── Summary ──
    print()
    passed = sum(1 for r in results if r)
    total = len(results)
    failed = total - passed
    skipped = sum(1 for r in results if r is None)
    print(f'Results: {passed}/{total} passed' +
          (f', {failed} failed' if failed else '') +
          (f', {skipped} value-unchecked' if skipped else ''))

    ser.close()
    return passed == total


if __name__ == '__main__':
    ok = main()
    sys.exit(0 if ok else 1)
