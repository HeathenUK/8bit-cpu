#!/usr/bin/env python3
"""Robust RUNHZ runner: compile a C source, upload via ESP32, run on
hardware until HLT (or maxoi events captured), and report the result.

Reads serial lines in a loop, skipping heartbeats (`{"hb":...}`), until
a final result line (`{"cyc":..., ...}`) arrives or the timeout fires.
This is what the simple `ser.readline()` approach got wrong — it grabbed
the first heartbeat and reported a KeyError instead of the actual run.

Usage:
    runhz_runner.py PROGRAM.c [--maxoi N] [--cycles N] [--hz N] [--timeout S]

Defaults:
    cycles=2_000_000  (~14s wall at 145 kHz; HLT-detect terminates earlier
                       on programs that halt)
    hz=145000
    maxoi=0          (0 = unlimited; run until cycles or HLT)
    timeout=30s
"""
import argparse, json, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hw_regression as hw
from mk1_upload import compile_c

def runhz_until_done(ser, cycles, hz, maxoi, timeout):
    """Send RUNHZ and read lines until we see a final result line or timeout."""
    hw.full_reset(ser)
    ser.reset_input_buffer()
    ser.timeout = timeout
    ser.write(f'RUNHZ:{cycles},{hz},{maxoi}\n'.encode())
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode().strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            print(f'  (non-JSON line skipped: {line[:80]!r})', file=sys.stderr)
            continue
        if 'hb' in r:
            print(f'  heartbeat: cyc={r["hb"]} oi={r["oi"]}', file=sys.stderr)
            continue
        if 'cyc' in r:
            return r
        print(f'  (unrecognized JSON: {r})', file=sys.stderr)
    return {'error': f'timeout after {timeout}s without final result'}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('program')
    ap.add_argument('--cycles', type=int, default=2_000_000)
    ap.add_argument('--hz', type=int, default=145_000)
    ap.add_argument('--maxoi', type=int, default=0)
    ap.add_argument('--timeout', type=int, default=30)
    ap.add_argument('--eeprom', action='store_true')
    args = ap.parse_args()

    print(f'compiling {args.program} ...', file=sys.stderr)
    asm = compile_c(args.program, optimize=True, eeprom=args.eeprom)
    if asm is None:
        print('compile failed', file=sys.stderr); return 1

    print('opening serial ...', file=sys.stderr)
    ser = hw.open_serial()
    try:
        print('uploading ...', file=sys.stderr)
        if not hw.assemble_upload(ser, asm):
            print('upload failed', file=sys.stderr); return 1
        print(f'RUNHZ:{args.cycles},{args.hz},{args.maxoi} (timeout {args.timeout}s) ...', file=sys.stderr)
        r = runhz_until_done(ser, args.cycles, args.hz, args.maxoi, args.timeout)
    finally:
        ser.close()

    if 'error' in r:
        print(f'ERROR: {r["error"]}'); return 1
    print(f'cyc={r.get("cyc")}  cnt={r.get("cnt")}  khz={r.get("khz")}  halted={r.get("halted")}')
    hist = r.get('hist', [])
    if hist:
        print(f'hist (last {len(hist)}): {hist}')
    return 0

if __name__ == '__main__':
    sys.exit(main())
