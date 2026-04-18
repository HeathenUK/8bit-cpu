#!/usr/bin/env python3
"""LCD output regression test.

Compiles a set of programs in different modes (flat / init extraction / overlay)
and uploads each to hardware. Prompts the user to confirm whether the LCD
displays the expected output.

This catches the class of bugs where I2C "succeeds" (no error from __i2c_sb)
but the PCF8574/LCD doesn't actually receive/display the data — which the
existing OI-based regression tests do NOT detect.

Usage:
    python3 lcd_regression_test.py
"""
import os
import subprocess
import sys
import time
import json
import serial

PORT = '/dev/cu.usbmodem3C8427C2E7C82'
COMPILER = os.path.join(os.path.dirname(__file__), 'mk1cc2.py')
PROG_DIR = os.path.join(os.path.dirname(__file__), '..', 'programs')


def compile_c(source, eeprom=False):
    """Compile C source to assembly. Returns (asm_text, mode_str) or (None, err)."""
    src_path = '/tmp/_lcd_regtest.c'
    asm_path = '/tmp/_lcd_regtest.asm'
    with open(src_path, 'w') as f:
        f.write(source)
    args = ['python3', COMPILER, src_path, '-O']
    if eeprom:
        args.append('--eeprom')
    args.extend(['-o', asm_path])
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        return None, r.stderr
    with open(asm_path) as f:
        asm = f.read()
    # Detect mode from compiler stderr
    mode = 'flat'
    if 'two-stage boot (overlay system)' in r.stderr:
        mode = 'overlay'
    elif 'two-stage boot (init extraction)' in r.stderr:
        mode = 'init-extraction'
    return asm, mode


def upload_and_run(ser, asm, cycles=5000000):
    """Upload and run an assembly program. Returns RUN response dict or None."""
    # Drain any pending serial data
    ser.timeout = 2
    while ser.read(8192):
        pass
    ser.timeout = 30

    escaped = asm.replace('\n', '\\n')
    payload = f'ASM:{escaped}\n'.encode()
    for i in range(0, len(payload), 4096):
        ser.write(payload[i:i+4096])
        time.sleep(0.05)
    try:
        resp = json.loads(ser.readline().decode().strip())
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None
    if not resp.get('ok'):
        return None
    ser.write(b'UPLOAD\n')
    ser.readline()
    ser.write(f'RUN:{cycles},1\n'.encode())
    try:
        return json.loads(ser.readline().decode().strip())
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None


def prompt_user(expected):
    """Ask user what they see on the LCD. Returns True if it matches."""
    print(f'  Expected on LCD: {expected!r}')
    while True:
        ans = input('  What do you see? [type the actual content, or "ok"/"no"]: ').strip()
        if ans.lower() == 'ok':
            return True
        if ans.lower() == 'no' or ans == '':
            return False
        # Accept any user-typed content; pass if it matches expected
        return ans == expected


# Test programs: (name, c_source, expected_lcd_output, eeprom_mode)
TESTS = [
    (
        'flat-tiny: lcd_init + 4 chars (flat mode)',
        '''
void main() {
    i2c_init();
    lcd_init();
    lcd_char('A'); lcd_char('B'); lcd_char('C'); lcd_char('D');
    out(42);
    halt();
}
''',
        'ABCD',
        False,
    ),
    (
        'init-extraction: lcd_init + 4 chars + arithmetic',
        '''
void main() {
    i2c_init();
    lcd_init();
    lcd_char('A'); lcd_char('B'); lcd_char('C'); lcd_char('D');
    unsigned char a; a = 1;
    a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1;
    a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1;
    a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1; a=a+1;
    out(a);
    halt();
}
''',
        'ABCD',
        False,
    ),
    (
        'init-extraction: read RTC temp + display',
        open(os.path.join(PROG_DIR, 'lcd_temp.c')).read(),
        '<temperature>°C  (e.g., "21°C" or "22°C")',
        False,
    ),
    (
        'overlay: clock display HH:MM (overlay mode)',
        open(os.path.join(PROG_DIR, 'overlay_clock.c')).read(),
        'HH:MM   (current time from RTC)',
        True,
    ),
]


def main():
    print('MK1 LCD Output Regression Test')
    print(f'Port: {PORT}')
    print()
    print('This test verifies actual LCD display output, not just I2C completion.')
    print('You must visually confirm what the LCD shows after each test.')
    print()

    ser = serial.Serial(PORT, 115200, timeout=10)
    time.sleep(2)
    ser.reset_input_buffer()

    results = []
    for name, source, expected, eeprom in TESTS:
        print(f'── {name} ──')
        asm, mode = compile_c(source, eeprom=eeprom)
        if asm is None:
            print(f'  [SKIP] compile failed: {mode[:200]}')
            results.append((name, 'SKIP'))
            continue
        print(f'  Compiled in {mode} mode')

        result = upload_and_run(ser, asm)
        if result is None:
            print('  [FAIL] upload/run failed')
            results.append((name, 'FAIL'))
            continue
        print(f'  Run: cyc={result.get("cyc")}, val={result.get("val")}')

        passed = prompt_user(expected)
        results.append((name, 'PASS' if passed else 'FAIL'))
        print(f'  → {"PASS" if passed else "FAIL"}')
        print()

    print('── Summary ──')
    for name, status in results:
        marker = {'PASS': '✓', 'FAIL': '✗', 'SKIP': '–'}[status]
        print(f'  {marker} [{status}] {name}')
    pass_count = sum(1 for _, s in results if s == 'PASS')
    print(f'\nResults: {pass_count}/{len(results)} passed')
    return 0 if pass_count == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
