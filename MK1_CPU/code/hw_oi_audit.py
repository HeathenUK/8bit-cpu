#!/usr/bin/env python3
"""Hardware OI audit for MK1 C programs.

Compiles each C program, assembles/uploads through the ESP32, runs with
RUNLOG, and records observed OI values. This is intentionally a hardware
audit, not a simulator check.
"""

import argparse
import glob
import fcntl
import json
import os
import re
import subprocess
import sys

import hw_regression

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
PROGRAMS_DIR = os.path.join(ROOT, 'MK1_CPU', 'programs')
COMPILER = os.path.join(ROOT, 'MK1_CPU', 'code', 'mk1cc2.py')


class PortLock:
    def __init__(self, port):
        safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', port)
        self.path = os.path.join('/tmp', f'mk1_hw_oi_{safe}.lock')
        self.fd = None

    def __enter__(self):
        self.fd = open(self.path, 'w')
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(
                f'Another hardware audit is already using this serial port '
                f'(lock: {self.path})'
            )
        self.fd.write(str(os.getpid()))
        self.fd.flush()
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.fd is not None:
            fcntl.flock(self.fd, fcntl.LOCK_UN)
            self.fd.close()
        return False


def tracked_programs():
    try:
        out = subprocess.check_output(['git', 'ls-files'], cwd=ROOT, text=True)
        files = [
            os.path.join(ROOT, p)
            for p in out.splitlines()
            if p.startswith('MK1_CPU/programs/') and p.endswith('.c')
        ]
        if files:
            return sorted(files)
    except Exception:
        pass
    return sorted(glob.glob(os.path.join(PROGRAMS_DIR, '*.c')))


def compile_one(path, optimize=True, eeprom=True):
    asm_path = '/tmp/_hw_oi_audit.asm'
    metrics_path = '/tmp/_hw_oi_audit.json'
    args = ['python3', COMPILER, path, '-o', asm_path, '--metrics-out', metrics_path]
    if optimize:
        args.append('-O')
    if eeprom:
        args.append('--eeprom')
    r = subprocess.run(
        args,
        cwd=os.path.dirname(COMPILER),
        capture_output=True,
        text=True,
        timeout=60,
    )
    metrics = None
    if os.path.exists(metrics_path) and os.path.getsize(metrics_path):
        try:
            with open(metrics_path) as f:
                metrics = json.load(f)
        except Exception:
            metrics = None
    if r.returncode != 0:
        return None, metrics, r.stderr[-2000:]
    with open(asm_path) as f:
        return f.read(), metrics, r.stderr[-2000:]


def placement_warnings(stderr):
    lines = stderr.splitlines() if stderr else []
    unsafe = [l.strip() for l in lines if 'wrapping overlay NOT loaded last' in l]
    safe = [l.strip() for l in lines if 'largest overlay wraps' in l]
    return {
        'unsafe_wrap': bool(unsafe),
        'safe_wrap': bool(safe),
        'wrap_warnings': unsafe + safe,
    }


def out_call_count(path):
    with open(path) as f:
        source = f.read()
    return len(re.findall(r'\bout\s*\(', source))


def run_program(path, args):
    asm, metrics, compile_stderr = compile_one(
        path,
        optimize=not args.no_optimize,
        eeprom=not args.no_eeprom,
    )
    row = {
        'program': os.path.relpath(path, ROOT),
        'out_call_count': out_call_count(path),
        'compile_ok': asm is not None,
        'metrics': metrics,
        'compile_stderr_tail': compile_stderr.splitlines()[-8:] if compile_stderr else [],
    }
    row.update(placement_warnings(compile_stderr))
    if asm is None:
        row.update({'upload_ok': False, 'run_ok': False, 'error': 'compile failed'})
        return row

    ser = hw_regression.open_serial(args.port)
    try:
        upload_ok = hw_regression.assemble_upload(ser, asm, timeout=args.asm_timeout)
        row['upload_ok'] = bool(upload_ok)
        if not upload_ok:
            row.update({'run_ok': False, 'error': 'assemble/upload failed'})
            return row
        result = hw_regression.runlog(
            ser,
            cycles=args.cycles,
            us=args.us,
            timeout=args.run_timeout,
        )
    finally:
        ser.close()

    vals = hw_regression.dedup_consecutive(result.get('vals', []))
    row.update({
        'run_ok': 'error' not in result,
        'oi_seen': bool(vals),
        'oi_count_raw': result.get('cnt'),
        'khz': result.get('khz'),
        'cycles': result.get('cyc'),
        'vals_first80': vals[:80],
        'error': result.get('error'),
    })
    return row


def main():
    ap = argparse.ArgumentParser(description='Run hardware OI audit on MK1 C programs')
    ap.add_argument('--port', default=hw_regression.PORT)
    ap.add_argument('--cycles', type=int, default=1_500_000)
    ap.add_argument('--us', type=int, default=1,
                    help='RUNLOG half-period argument; us=1 is about 250 kHz on current firmware')
    ap.add_argument('--asm-timeout', type=int, default=25)
    ap.add_argument('--run-timeout', type=int, default=35)
    ap.add_argument('--filter', help='substring filter on program path')
    ap.add_argument('--output', default='/tmp/hw_oi_audit.json')
    ap.add_argument('--no-eeprom', action='store_true')
    ap.add_argument('--no-optimize', action='store_true')
    opts = ap.parse_args()

    try:
        lock_ctx = PortLock(opts.port)
        lock_ctx.__enter__()
    except RuntimeError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 2
    try:
        programs = tracked_programs()
        if opts.filter:
            programs = [p for p in programs if opts.filter in os.path.relpath(p, ROOT)]

        results = []
        for i, path in enumerate(programs, 1):
            rel = os.path.relpath(path, ROOT)
            print(f'[{i}/{len(programs)}] {rel}', flush=True)
            row = run_program(path, opts)
            results.append(row)
            vals = row.get('vals_first80') or []
            print(
                f"  compile={row.get('compile_ok')} upload={row.get('upload_ok')} "
                f"oi={row.get('oi_seen')} unsafe_wrap={row.get('unsafe_wrap')} "
                f"khz={row.get('khz')} vals={vals[:12]} "
                f"err={row.get('error')}",
                flush=True,
            )
    finally:
        lock_ctx.__exit__(None, None, None)

    summary = {
        'mode': {
            'optimize': not opts.no_optimize,
            'eeprom': not opts.no_eeprom,
            'cycles': opts.cycles,
            'us': opts.us,
            'port': opts.port,
        },
        'total': len(results),
        'compile_ok': sum(1 for r in results if r.get('compile_ok')),
        'upload_ok': sum(1 for r in results if r.get('upload_ok')),
        'run_ok': sum(1 for r in results if r.get('run_ok')),
        'oi_seen': sum(1 for r in results if r.get('oi_seen')),
        'unsafe_wrap': sum(1 for r in results if r.get('unsafe_wrap')),
        'safe_wrap': sum(1 for r in results if r.get('safe_wrap')),
        'results': results,
    }
    with open(opts.output, 'w') as f:
        json.dump(summary, f, indent=2)
    print(json.dumps({k: summary[k] for k in ('total', 'compile_ok', 'upload_ok', 'run_ok', 'oi_seen', 'unsafe_wrap', 'safe_wrap')}, indent=2))
    print(opts.output)
    return 0 if summary['compile_ok'] == summary['total'] and summary['upload_ok'] == summary['total'] else 1


if __name__ == '__main__':
    sys.exit(main())
