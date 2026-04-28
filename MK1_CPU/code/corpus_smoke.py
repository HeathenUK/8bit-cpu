#!/usr/bin/env python3
"""corpus_smoke.py — compile every C program in ../programs/ and report
size deltas vs a baseline.

Usage:
    python3 corpus_smoke.py            # check all programs against baseline
    python3 corpus_smoke.py --update   # regenerate baseline from current state
    python3 corpus_smoke.py --quick    # skip programs that need --eeprom

Catches the failure mode where a compiler change accidentally regresses
output size on programs that aren't in hw_regression.py's 19-test list.
The 67-program corpus exercises far more codegen paths than the
regression specimens, but currently nothing checks they still compile
cleanly across compiler edits.

The baseline is keyed by relative path under programs/. Each entry
records the flags that successfully compiled the program plus the size
fields that matter (stage1_total, kernel, eeprom_used, etc.). New
programs are added on `--update`; deleted programs are dropped.

Failure modes flagged:
- compile error on a program that previously compiled
- stage-1 / page sizes that differ from baseline (in either direction)
- a program that needed `--eeprom` last time but doesn't now (or vice versa)

Each run takes ~1 minute on the current corpus.
"""

import argparse
import json
import os
import subprocess
import sys

CODE_DIR = os.path.dirname(os.path.abspath(__file__))
PROGRAMS_DIR = os.path.join(CODE_DIR, '..', 'programs')
BASELINE_PATH = os.path.join(CODE_DIR, 'corpus_baseline.json')
COMPILER = os.path.join(CODE_DIR, 'mk1cc2.py')

# Fields we track for size regressions. Picked to be deterministic across
# compiler runs (no timestamps, no peephole-fire counts).
TRACKED_FIELDS = [
    'mode',
    'stage1_total',
    'kernel',
    'page0_used',
    'page1_allocated',
    'page3_used',
    'eeprom_used',
    'overlay_slots',
    'overlay_largest',
    'overlay_region',
]


def list_corpus():
    """Return list of program paths (relative to PROGRAMS_DIR), excluding
    the template directory and any obvious scratch files."""
    out = []
    for f in sorted(os.listdir(PROGRAMS_DIR)):
        if not f.endswith('.c'):
            continue
        full = os.path.join(PROGRAMS_DIR, f)
        if not os.path.isfile(full):
            continue
        out.append(f)
    return out


def try_compile(c_path, flags):
    """Run mk1cc2 with given flags. Returns (ok, metrics_dict_or_None, error_summary)."""
    metrics_path = '/tmp/_smoke_metrics.json'
    asm_path = '/tmp/_smoke.asm'
    try:
        os.unlink(metrics_path)
    except FileNotFoundError:
        pass
    args = ['python3', COMPILER, c_path, '-o', asm_path,
            '--metrics-out', metrics_path] + flags
    r = subprocess.run(args, capture_output=True, text=True, timeout=60)
    if r.returncode != 0:
        return False, None, _extract_error_summary(r.stderr)
    try:
        with open(metrics_path) as f:
            metrics = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        return False, None, f'metrics read failed: {e}'
    return True, metrics, ''


def _extract_error_summary(stderr):
    """Pick the most informative line from compiler stderr — typically a
    `Exception: ...` or `!! ... OVERFLOW: ...` message. Falls back to the
    last non-blank line. Used to build a stable error signature for the
    baseline so cosmetic stderr churn doesn't fire false regressions."""
    lines = [ln for ln in stderr.split('\n') if ln.strip()]
    # Prefer the Exception line (Python's default traceback puts it last).
    for ln in reversed(lines):
        s = ln.strip()
        if s.startswith('Exception:') or s.startswith('!! '):
            return s[:160]
    # Otherwise the last meaningful line.
    return lines[-1].strip()[:160] if lines else 'unknown error'


def compile_program(name):
    """Find the first flag set that successfully compiles `name`. Returns
    (flags_used, metrics) or (None, error_string).

    Probes flag sets in order: -O, -O --eeprom, plain, --eeprom. Most
    programs work with -O; a handful need --eeprom because the partitioner
    can't fit them otherwise."""
    c_path = os.path.join(PROGRAMS_DIR, name)
    candidates = [
        ['-O'],
        ['-O', '--eeprom'],
        [],
        ['--eeprom'],
    ]
    last_err = ''
    for flags in candidates:
        ok, metrics, err = try_compile(c_path, flags)
        if ok:
            return flags, metrics
        last_err = err
    return None, last_err


def diff_metrics(baseline, current):
    """Return list of (field, baseline_value, current_value) for fields that
    changed. Skips fields not in TRACKED_FIELDS."""
    diffs = []
    for field in TRACKED_FIELDS:
        b = baseline.get(field)
        c = current.get(field)
        if b != c:
            diffs.append((field, b, c))
    return diffs


def fmt_size_delta(field, baseline_v, current_v):
    """Pretty-print a single field delta. Highlights byte-count regressions."""
    if isinstance(baseline_v, (int, float)) and isinstance(current_v, (int, float)):
        delta = current_v - baseline_v
        sign = '+' if delta > 0 else ''
        return f'{field}: {baseline_v} → {current_v} ({sign}{delta})'
    return f'{field}: {baseline_v!r} → {current_v!r}'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--update', action='store_true',
                    help='Regenerate baseline from current state. Use after '
                         'an intentional compiler change.')
    ap.add_argument('--quick', action='store_true',
                    help='Skip programs that need --eeprom (faster).')
    ap.add_argument('--baseline', default=BASELINE_PATH,
                    help='Path to baseline JSON (default: %(default)s).')
    args = ap.parse_args()

    programs = list_corpus()
    print(f'Compiling {len(programs)} programs from {PROGRAMS_DIR}...')

    if not args.update and not os.path.exists(args.baseline):
        print(f'No baseline at {args.baseline}; run with --update first.',
              file=sys.stderr)
        sys.exit(2)

    baseline = {}
    if not args.update and os.path.exists(args.baseline):
        with open(args.baseline) as f:
            baseline = json.load(f)

    current = {}
    status_changed = []   # was OK, now errors (or vice versa)
    metric_changed = []   # OK in both, but metrics differ
    new_programs = []
    deleted_programs = []

    for name in programs:
        flags, metrics = compile_program(name)
        if flags is None:
            # Persist errored programs in current snapshot so we detect
            # if a previously-erroring program starts compiling again
            # (or vice versa).
            sig = metrics  # already reduced to a stable summary by _extract_error_summary
            current[name] = {'status': 'error', 'error_sig': sig}
            if not args.update:
                prev = baseline.get(name)
                if prev is None:
                    new_programs.append(name)
                    print(f'  [NEW ] {name} ERROR: {sig[:80]}')
                elif prev.get('status') != 'error':
                    status_changed.append((name, prev, current[name]))
                    print(f'  [REGR] {name} was OK, now ERROR: {sig[:80]}')
                elif prev.get('error_sig') != sig:
                    metric_changed.append((name, [('error_sig', prev.get('error_sig'), sig)]))
                    print(f'  [DIFF] {name} error changed:')
                    print(f'           was: {prev.get("error_sig","")[:80]}')
                    print(f'           now: {sig[:80]}')
                else:
                    pass  # known error, unchanged — quiet
            else:
                print(f'  [SET ] {name} ERROR: {sig[:80]}')
            continue

        if args.quick and '--eeprom' in flags:
            continue
        # Capture only the fields that matter for regression checking.
        snapshot = {'status': 'ok', 'flags': flags}
        snapshot.update({k: metrics.get(k) for k in TRACKED_FIELDS})
        current[name] = snapshot

        if args.update:
            print(f'  [SET ] {name} flags={" ".join(flags) or "(none)"}: '
                  f'stage1={snapshot.get("stage1_total")}B '
                  f'eeprom={snapshot.get("eeprom_used")}B')
            continue

        prev = baseline.get(name)
        if prev is None:
            new_programs.append(name)
            print(f'  [NEW ] {name}: not in baseline (compiled OK)')
            continue
        if prev.get('status') == 'error':
            status_changed.append((name, prev, snapshot))
            print(f'  [FIX!] {name} was ERROR, now compiles OK')
            continue
        if prev.get('flags') != flags:
            metric_changed.append((name, [('flags', prev.get('flags'), flags)]))
            print(f'  [FLAG] {name}: flags changed {prev.get("flags")} → {flags}')
            continue
        diffs = diff_metrics(prev, snapshot)
        if diffs:
            metric_changed.append((name, diffs))
            print(f'  [DIFF] {name}:')
            for d in diffs:
                print(f'           {fmt_size_delta(*d)}')
        else:
            pass  # unchanged — quiet

    # In update mode, also drop programs that were in the old baseline but
    # are no longer in the corpus.
    if args.update:
        for old_name in baseline:
            if old_name not in current:
                deleted_programs.append(old_name)
                print(f'  [DEL ] {old_name}: dropping from baseline')
        with open(args.baseline, 'w') as f:
            json.dump(current, f, indent=2, sort_keys=True)
        print(f'\nWrote baseline: {len(current)} programs, '
              f'{len(deleted_programs)} dropped → {args.baseline}')
        return 0

    # Summary
    print()
    total = len(programs)
    n_changed = len(status_changed) + len(metric_changed)
    n_unchanged = total - n_changed - len(new_programs)
    n_errors = sum(1 for v in current.values() if v.get('status') == 'error')
    print(f'Summary: {total} programs ({n_errors} known to error)')
    print(f'  unchanged:      {n_unchanged}')
    print(f'  status changed: {len(status_changed)}  (OK ↔ error)')
    print(f'  metric changed: {len(metric_changed)}  (size deltas)')
    print(f'  new:            {len(new_programs)}')
    if status_changed or metric_changed or new_programs:
        print(f'  → run with --update if changes are intentional.')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
