"""Stream-PoC runner — uses hw_regression's chip-verified assemble_upload.

Reads stream_poc.asm, uploads with byte-exact chip verification, then
runs the program and prints the captured OI sequence.
"""
import os
import sys
import time
import json

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.chdir(os.path.dirname(os.path.abspath(__file__)))
from hw_regression import open_serial, assemble_upload, runlog


def main():
    asm_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'stream_poc.asm')
    with open(asm_path) as f:
        asm = f.read()

    ser = open_serial()

    print('Uploading stream_poc.asm with chip verify-retry...')
    if not assemble_upload(ser, asm, timeout=15):
        print('FAILED: assemble_upload returned False', file=sys.stderr)
        return 1

    print('Running...')
    result = runlog(ser, cycles=2_000_000, us=3, timeout=15, maxoi=64)
    ser.close()

    print('output:', result)
    hist = result.get('hist', [])
    print('hist (decimal):', hist)
    print('hist (hex):    ', [f'0x{b:02X}' for b in hist])

    # Expected streamed-program output: AA, BB, CA(kernel call), CC, EE.
    expected_substring = [0xAA, 0xBB, 0xCA, 0xCC, 0xEE]
    idx = 0
    for b in hist:
        if idx < len(expected_substring) and b == expected_substring[idx]:
            idx += 1
    if idx == len(expected_substring):
        print('PASS — streamed bytes appear in expected order')
        return 0
    else:
        print(f'FAIL — expected {expected_substring} as subsequence, '
              f'matched only {idx}/{len(expected_substring)}')
        return 1


if __name__ == '__main__':
    sys.exit(main())
