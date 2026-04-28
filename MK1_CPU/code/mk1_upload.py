#!/usr/bin/env python3
"""MK1 upload helper — reliable serial communication for large programs.

Usage:
    python3 mk1_upload.py <program.asm> [--run [cycles]] [--port PORT]
    python3 mk1_upload.py <program.c> [-O] [--eeprom] [--run [cycles]] [--port PORT]

Handles: chunked serial sending, assembly, upload, run with timeout.
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
CHUNK_SIZE = 64
CHUNK_DELAY = 0.01  # 10ms between chunks


def send_chunked(ser, data):
    """Send data in small chunks to avoid UART buffer overflow."""
    for i in range(0, len(data), CHUNK_SIZE):
        ser.write(data[i:i+CHUNK_SIZE])
        time.sleep(CHUNK_DELAY)


def mk1_asm(ser, asm_text):
    """Assemble on ESP32. Returns parsed JSON response."""
    ser.reset_input_buffer()
    escaped = asm_text.replace('\n', '\\n')
    cmd = f'ASM:{escaped}\n'.encode()
    send_chunked(ser, cmd)
    r = ser.readline().decode().strip()
    try:
        return json.loads(r)
    except:
        print(f'ASM error: {r[:200]}', file=sys.stderr)
        return None


def mk1_upload(ser):
    """Upload assembled buffer to MK1."""
    ser.write(b'UPLOAD\n')
    ser.readline()


def mk1_run(ser, cycles=10000000, timeout=120):
    """Run program. Returns parsed JSON response."""
    ser.write(f'RUN:{cycles},1\n'.encode())
    old_timeout = ser.timeout
    ser.timeout = timeout
    r = ser.readline().decode().strip()
    ser.timeout = old_timeout
    try:
        return json.loads(r)
    except:
        print(f'RUN timeout or error: {r[:200]}', file=sys.stderr)
        return None


def compile_c(source_file, optimize=True, eeprom=False):
    """Compile C to assembly. Returns asm text or None."""
    args = ['python3', COMPILER, source_file]
    if optimize:
        args.append('-O')
    if eeprom:
        args.append('--eeprom')
    args.extend(['-o', '/tmp/_mk1_upload.asm'])
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        print(f'Compile error: {r.stderr[:500]}', file=sys.stderr)
        return None
    if r.stderr:
        print(r.stderr.strip(), file=sys.stderr)
    with open('/tmp/_mk1_upload.asm') as f:
        return f.read()


# Hash-cache for skip-asm-if-unchanged. Stored at /tmp/.mk1_asm_cache —
# a single line of `<sha1>:<port>` for the most-recently-uploaded asm.
# When the asm hash matches AND the port matches, we know the ESP32's
# uploadBuf still holds the right bytes (assuming no power-cycle since
# the previous upload), so we can skip the ASM serial transfer (the
# slow step — kilobytes over UART) and go straight to UPLOAD. Saves
# ~0.5–1.5 s per iteration.
#
# Trade: if the ESP32 was power-cycled between runs, uploadBuf is
# whatever was there before. Pass `--no-cache` to bypass, or `make
# reset` then re-run (a reset doesn't clear uploadBuf either, but a
# real power cycle does — and you'll know because the next ASM will
# be fresh).
ASM_CACHE_PATH = '/tmp/.mk1_asm_cache'

def _asm_hash(asm_text):
    import hashlib
    return hashlib.sha1(asm_text.encode()).hexdigest()

def _read_cached_hash(port):
    try:
        with open(ASM_CACHE_PATH) as f:
            line = f.read().strip()
        h, p = line.split(':', 1)
        return h if p == port else None
    except (FileNotFoundError, ValueError):
        return None

def _write_cached_hash(h, port):
    try:
        with open(ASM_CACHE_PATH, 'w') as f:
            f.write(f'{h}:{port}\n')
    except OSError:
        pass


def _read_hex_line(ser):
    """Read serial lines until one looks like hex tokens, skipping diagnostic
    strings (firmware streams "EEPROM shim page 0x..." async during EEPROM
    writes; they sometimes land in the buffer between our DUMP request and
    the response). Returns the matching line or '' on timeout."""
    for _ in range(20):
        line = ser.readline().decode(errors='replace').strip()
        if not line:
            return ''
        toks = line.split()
        if toks and all(len(t) == 2 for t in toks):
            return line
    return ''


def _dump_region(ser, cmd, total, label_offset_base=0):
    """Dump `total` bytes via repeated `cmd:off,BLK` calls. Returns bytearray."""
    BLK = 64
    raw = bytearray()
    for off in range(0, total, BLK):
        ser.reset_input_buffer()
        ser.write(f'{cmd}:{off + label_offset_base},{BLK}\n'.encode())
        line = _read_hex_line(ser)
        for tok in line.split():
            if len(tok) == 2:
                raw.append(int(tok, 16))
    return raw


def _hex_print(name, page, base=0, width=16):
    """Pretty 16-byte-wide hex grid with ASCII gutter."""
    print(f'\n── {name} ({len(page)}B) ──')
    for row in range(0, len(page), width):
        offset = base + row
        chunk = page[row:row+width]
        hex_part = ' '.join(f'{b:02X}' for b in chunk)
        ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        print(f'  {offset:04X}: {hex_part:<{width*3-1}}  |{ascii_part}|')


def dump_bufs(ser, eeprom_size=0):
    """Hex-dump the four 256B SRAM page buffers from the firmware's uploadBuf,
    plus the EEPROM buffer when `eeprom_size > 0`.

    uploadBuf layout: [code(256), data(256), stack(256), page3(256)]. Reads
    via the firmware's DUMP:offset,count command. Use after UPLOAD when you
    suspect an emission-order bug — e.g. the manifest claims overlay X is at
    offset 192 but its bytes actually landed at 198, so `derefp3` reads zeros.
    A glance at the page-2 dump shows the bytes' real position.

    EEPROM buffer dump targets the `assembler.result.eeprom[]` buffer (what
    was *meant* to be written to AT24C32, NOT the live AT24C32 contents).
    Catches alignment issues where eg. an overlay lands at eeprom[0xF0]
    instead of eeprom[0x100] because the 16-byte header pad was skipped.
    """
    raw = _dump_region(ser, 'DUMP', 1024)
    if len(raw) < 1024:
        print(f'  dump short: got {len(raw)} bytes, expected 1024', file=sys.stderr)
        return
    for i, name in enumerate(['code', 'data', 'stack', 'page3']):
        _hex_print(f'{name} (page {i})', raw[i*256:(i+1)*256])
    if eeprom_size > 0:
        # Round up to next 64B block so we cover the full payload.
        ee_total = (eeprom_size + 63) & ~63
        ee_raw = _dump_region(ser, 'DUMP_EE', ee_total)
        if len(ee_raw) >= eeprom_size:
            _hex_print(f'eeprom (assembler buffer, {eeprom_size}B used)',
                       ee_raw[:eeprom_size])
        else:
            print(f'  eeprom dump short: got {len(ee_raw)} bytes, expected {eeprom_size}',
                  file=sys.stderr)
            print(f'  (firmware may need re-flashing for DUMP_EE support)',
                  file=sys.stderr)


def main():
    import argparse
    ap = argparse.ArgumentParser(description='MK1 Upload Helper')
    ap.add_argument('input', help='Assembly (.asm) or C (.c) source file')
    ap.add_argument('-O', '--optimize', action='store_true', help='Enable optimization')
    ap.add_argument('--eeprom', action='store_true', help='EEPROM overlay mode')
    ap.add_argument('--run', nargs='?', const=10000000, type=int, help='Run after upload (optional cycle count)')
    ap.add_argument('--port', default=PORT, help=f'Serial port (default: {PORT})')
    ap.add_argument('--timeout', type=int, default=120, help='Run timeout in seconds')
    ap.add_argument('--no-cache', action='store_true',
                    help='Always re-assemble; bypass the hash cache.')
    ap.add_argument('--dump-bufs', action='store_true',
                    help='After UPLOAD, hex-dump the four assembled SRAM page '
                         'buffers (code/data/stack/page3) from the firmware '
                         'uploadBuf. Useful when chasing emission-order bugs '
                         '(manifest claims an offset but the bytes landed '
                         'somewhere else). Also enabled by MK1_DUMP_BUFS=1.')
    args = ap.parse_args()

    # Compile if C source
    if args.input.endswith('.c'):
        print(f'Compiling {args.input}...')
        asm = compile_c(args.input, optimize=args.optimize, eeprom=args.eeprom)
        if asm is None:
            sys.exit(1)
    else:
        with open(args.input) as f:
            asm = f.read()

    print(f'Connecting to {args.port}...')
    ser = serial.Serial(args.port, BAUD, timeout=30)
    time.sleep(1)
    ser.reset_input_buffer()

    # Hash + cache check — skip ASM if uploadBuf already has these bytes.
    asm_h = _asm_hash(asm)
    cached_h = None if args.no_cache else _read_cached_hash(args.port)
    eeprom_size = 0
    if cached_h == asm_h:
        print(f'Skipping assembly (cache hit, hash={asm_h[:8]}...)')
    else:
        print(f'Assembling ({len(asm)} bytes of source)...')
        r = mk1_asm(ser, asm)
        if r is None or not r.get('ok'):
            if r and r.get('detail'):
                for err in r['detail']:
                    print(f"  Error line {err['line']}: {err['msg']}", file=sys.stderr)
            elif r:
                print(f"  {r}", file=sys.stderr)
            ser.close()
            sys.exit(1)
        print(f'  code={r["code"]}B data={r.get("data",0)}B')
        eeprom_size = r.get('eeprom', 0)
        _write_cached_hash(asm_h, args.port)

    # Optional: hex-dump the four SRAM page buffers from uploadBuf and the
    # assembler's EEPROM buffer for emission-order debugging (see `dump_bufs`
    # docstring). MUST be done BEFORE UPLOAD: writeEepromData internally
    # re-runs the assembler to build I²C shim programs, which zeros
    # `result.eeprom[]`. The SRAM uploadBuf is unaffected.
    if args.dump_bufs or os.environ.get('MK1_DUMP_BUFS') == '1':
        dump_bufs(ser, eeprom_size=eeprom_size)

    # Upload
    print('Uploading...')
    mk1_upload(ser)

    # Run
    if args.run is not None:
        cycles = args.run
        print(f'Running ({cycles} cycles, timeout {args.timeout}s)...')
        r2 = mk1_run(ser, cycles=cycles, timeout=args.timeout)
        if r2:
            print(f'  val={r2["val"]} cap={r2["cap"]} cyc={r2["cyc"]}', end='')
            if r2.get('aborted'):
                print(' (ABORTED)', end='')
            print(f' khz={r2.get("khz","")}')
        else:
            print('  No response (program may still be running)')

    ser.close()


if __name__ == '__main__':
    main()
