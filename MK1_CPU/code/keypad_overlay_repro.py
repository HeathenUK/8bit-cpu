#!/usr/bin/env python3
"""Run a keypad-using MK1 program in the simulator with scheduled key
press events.

Used to verify that the cross-overlay reload-thunk path for
__keypad_scan is software-correct. Sim returns the right key indices
even though hardware doesn't (separate hardware-only bug — see
WORKLOG entry on keypad-in-overlay).

Default program: programs/keypad_thunk_iso.c — emits the key index
each time keypad_scan() returns a different value. Default schedule
presses (0,1) at cyc 200K, releases at 400K, presses (2,3) at 600K,
releases at 800K. Customize via CLI flags.

Usage:
    python3 keypad_overlay_repro.py [PROGRAM.c] [--max-cycles N]
                                     [--press CYC,ROW,COL]...
                                     [--release CYC,ROW,COL]...

Example:
    python3 keypad_overlay_repro.py \
        programs/keypad_thunk_iso.c \
        --press 100000,1,2 --release 300000,1,2
"""
import argparse, os, sys

REPO_CODE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, REPO_CODE)
os.chdir(REPO_CODE)

import mk1sim
from verify_via_sim import compile_to_asm, esp32_assemble_and_dump


def parse_event(spec, action):
    parts = spec.split(',')
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f'expected CYC,ROW,COL, got {spec!r}')
    return (int(parts[0]), action, int(parts[1]), int(parts[2]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('program', nargs='?',
                    default='../programs/keypad_thunk_iso.c')
    ap.add_argument('--max-cycles', type=int, default=1_500_000)
    ap.add_argument('--press', action='append', default=[],
                    metavar='CYC,ROW,COL',
                    help='Schedule a press at cycle CYC at row,col')
    ap.add_argument('--release', action='append', default=[],
                    metavar='CYC,ROW,COL',
                    help='Schedule a release at cycle CYC at row,col')
    ap.add_argument('--eeprom', action='store_true',
                    help='Compile with --eeprom (sim doesn\'t model EEPROM '
                         'preload, may halt on stage-1 init)')
    args = ap.parse_args()

    asm = compile_to_asm(args.program, eeprom=args.eeprom)
    if asm is None:
        return 1
    pages = esp32_assemble_and_dump(asm)

    cpu = mk1sim.MK1()
    cpu.mem[0] = bytearray(pages['code'])
    cpu.mem[1] = bytearray(pages['data'])
    cpu.mem[2] = bytearray(pages['stack'])
    cpu.mem[3] = bytearray(pages['page3'])

    if args.press or args.release:
        events = []
        events.extend(parse_event(s, 'press') for s in args.press)
        events.extend(parse_event(s, 'release') for s in args.release)
        events.sort(key=lambda e: e[0])
        cpu.keypad_events = events
    else:
        # Default schedule: press (0,1) 200K-400K, press (2,3) 600K-800K.
        cpu.keypad_events = [
            (200_000, 'press', 0, 1),
            (400_000, 'release', 0, 1),
            (600_000, 'press', 2, 3),
            (800_000, 'release', 2, 3),
        ]

    last_oi = 0
    for _ in range(args.max_cycles):
        if not cpu.tick():
            break
        if len(cpu.output_history) > last_oi:
            v = cpu.output_history[-1]
            kp = sorted(cpu.pressed_keys)
            print(f'cyc={cpu.cycle:>7d}  out({v:>3d}=0x{v:02X})  '
                  f'pressed={kp}  '
                  f'via=ddra:{cpu.via_ddra:02X} '
                  f'ora:{cpu.via_ora:02X} '
                  f'ddrb:{cpu.via_ddrb:02X} '
                  f'orb:{cpu.via_orb:02X}')
            last_oi = len(cpu.output_history)

    print()
    print(f'completed at cycle {cpu.cycle}; halted={cpu.halted}')
    print(f'output_history (full): {cpu.output_history}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
