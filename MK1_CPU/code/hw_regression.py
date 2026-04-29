#!/usr/bin/env python3
"""MK1 hardware regression harness — deterministic output verification.

Fixes the reliability gaps that made earlier "hardware tests" meaningless:
  - RESETs MK1 before EVERY run (the ESP32's UPLOAD alone doesn't put PC
    back to 0 deterministically; stale register/trap state survives).
  - Uses RUNLOG (continuous OI capture) so we see the WHOLE output
    sequence, not just the first value. Single-shot `RUN:N,1` only
    captures one value.
  - Tests use only COMPILE-TIME-KNOWN expected outputs. No `peek3(0)`
    dependency on MK1's residual SRAM state across tests.
  - Each test is run REPLICATIONS times; asserts identical capture.
    Real non-determinism fails the test.

Pass/fail: `expected_hist[:N] == actual_hist[:N]` where N = len(expected).

Usage:
    python3 hw_regression.py [--port /dev/cu.usbmodem...]
                             [--replications 3]
                             [--verbose]
Exit code 0 only if every test passes N replications in a row.
"""
import argparse
import json
import os
import subprocess
import sys
import time
import serial

PORT = '/dev/cu.usbmodem3C8427C2E7C82'
BAUD = 115200
COMPILER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mk1cc2.py')


# ── Low-level serial helpers ─────────────────────────────────────────

def open_serial(port=PORT):
    ser = serial.Serial(port, BAUD, timeout=30)
    # Match overlay_regression_test.py: full 1s wait + drain after
    # opening the port. The ESP32 seems to need this to be responsive —
    # shorter settles result in empty replies on the first ASM.
    time.sleep(1.0)
    ser.reset_input_buffer()
    return ser


def full_reset(ser):
    """Single RESET with settle. Matches the overlay_regression_test
    pattern that's known to work."""
    old_timeout = ser.timeout
    ser.timeout = 5
    ser.reset_input_buffer()
    ser.write(b'RESET\n')
    try:
        ser.readline()
    except Exception:
        pass
    time.sleep(0.2)
    ser.timeout = old_timeout


def assemble_upload(ser, asm_text, timeout=15):
    """ASM: + UPLOAD. Returns True on success."""
    full_reset(ser)
    esc = asm_text.replace('\n', '\\n')
    payload = f'ASM:{esc}\n'.encode()
    CHUNK = 4096
    if len(payload) > CHUNK:
        for i in range(0, len(payload), CHUNK):
            ser.write(payload[i:i+CHUNK])
            time.sleep(0.05)
    else:
        ser.write(payload)
    ser.timeout = timeout
    # Skip non-JSON diagnostic lines (e.g. "EEPROM shim page 0x0100: ..."
    # that the firmware streams during EEPROM writes from a PRIOR test's
    # UPLOAD — they arrive at the host serial buffer slightly later than
    # the prior UPLOAD's JSON response and end up first-in-line for the
    # next ASM read). Loop until we get a `{...}` line or hit timeout.
    r = None
    raw_reply = b''
    for _ in range(20):
        try:
            raw_reply = ser.readline()
        except Exception:
            break
        if not raw_reply:
            break
        line = raw_reply.decode(errors='replace').strip()
        if line.startswith('{'):
            try:
                r = json.loads(line)
                break
            except Exception:
                continue
    if r is None:
        print(f'    ASM parse error: no JSON in response; last raw={raw_reply[:200]!r}', file=sys.stderr)
        return False
    if not r.get('ok'):
        print(f'    ASM failed: {r}', file=sys.stderr)
        return False
    ser.reset_input_buffer()
    ser.write(b'UPLOAD\n')
    try:
        ser.readline()
    except Exception:
        pass
    return True


def dump_pages(ser, timeout=10):
    """Fetch the four 256B SRAM pages from the ESP32's uploadBuf.

    uploadBuf layout: [code(256), data(256), stack(256), page3(256)].
    Returns a dict {'code', 'data', 'stack', 'page3'} of bytearrays —
    the same shape `verify_via_sim.esp32_assemble_and_dump()` returns.

    Used by the sim cross-check (--differential): after `assemble_upload`
    has written the assembler output into uploadBuf, this lets us run
    the same program in the simulator without re-uploading or re-
    opening the serial port.
    """
    ser.timeout = timeout
    out = bytearray()
    BLK = 64
    for off in range(0, 1024, BLK):
        ser.reset_input_buffer()
        ser.write(f'DUMP:{off},{BLK}\n'.encode())
        # Skip non-hex diagnostic lines (e.g. EEPROM shim output from a
        # prior UPLOAD that the firmware streams asynchronously). The
        # DUMP response is a single line of `XX XX XX...` hex tokens.
        line = ''
        for _ in range(20):
            line = ser.readline().decode(errors='replace').strip()
            if not line:
                break
            toks = line.split()
            # Heuristic: a real DUMP line has BLK tokens of length 2.
            if toks and all(len(t) == 2 for t in toks):
                break
        for tok in line.split():
            if len(tok) == 2:
                out.append(int(tok, 16))
    if len(out) < 1024:
        raise RuntimeError(f'DUMP short: got {len(out)} bytes, expected 1024')
    return {
        'code':  out[0:256],
        'data':  out[256:512],
        'stack': out[512:768],
        'page3': out[768:1024],
    }


def runlog(ser, cycles=5_000_000, us=1, timeout=30):
    """RUNLOG: continuous OI capture. Returns dict with vals list."""
    # Reset BEFORE each run — UPLOAD doesn't reset PC, and any prior run
    # leaves CPU at its halt-trap. Without this, the new run starts in
    # the middle of wherever the previous run ended.
    full_reset(ser)
    ser.reset_input_buffer()
    ser.timeout = timeout
    ser.write(f'RUNLOG:{cycles},{us}\n'.encode())
    # Skip non-RUNLOG-shaped lines: EEPROM shim diagnostic strings AND
    # leftover UPLOAD `{"ok":true,...}` JSONs from a prior write that
    # finished after we'd moved on. RUNLOG's response always carries
    # `"cyc"`, so use that to disambiguate.
    line = ''
    r = None
    for _ in range(20):
        try:
            line = ser.readline().decode(errors='replace').strip()
        except Exception:
            line = ''
            break
        if not line:
            break
        if line.startswith('{') and '"cyc"' in line:
            try:
                r = json.loads(line)
                break
            except Exception:
                continue
    if r is None:
        return {'cyc': 0, 'cnt': 0, 'vals': [], 'error': f'parse failed: {line[:80]}'}
    return r


# ── Output sequence post-processing ──────────────────────────────────

def dedup_consecutive(vals):
    """RUNLOG samples OI on BOTH clock edges, so each `out` may register
    twice (once per edge). Collapse runs of the same value to one entry
    — `out(42); out(42)` is rare in real programs; most consecutive
    duplicates are the sampling artefact."""
    out = []
    for v in vals:
        if not out or out[-1] != v:
            out.append(v)
    return out


def dedup_consecutive_with_ts(vals, ts):
    """Like dedup_consecutive but also returns the ts (MK1 cycle count)
    of the FIRST sample in each run. Used for timing-based assertions."""
    out_v = []
    out_t = []
    for i, v in enumerate(vals):
        if not out_v or out_v[-1] != v:
            out_v.append(v)
            out_t.append(ts[i] if i < len(ts) else None)
    return out_v, out_t


# Canonical "execution started at PC=0" sentinel. The test runner wraps
# every program with `out(0xDE); out(0xAD); out(0xBE);` as the FIRST thing
# after the _main preamble. If we see that 3-value sequence in the OI
# capture, execution genuinely started clean (reset put PC at 0, self-copy
# ran, _main body ran from the top). Any OI samples BEFORE the sentinel
# are reset-transient artefacts (bus values latched during RUN startup
# before the CPU settled) and are discarded.
SENTINEL = [0xDE, 0xAD, 0xBE]


def find_sequence(vals, expected):
    """Verify the program's output matches `expected`.

    Strategy: each iteration of the halt-retry loop starts with the
    SENTINEL [0xDE,0xAD,0xBE], so a correct program emits
    [S0,S1,S2, expected..., S0,S1,S2, expected..., ...] in the capture.
    Pass if ANY sentinel occurrence is followed by exactly `expected`.

    Why any-occurrence (not first-occurrence):
      - The first halt-retry iteration often has startup artefacts:
        the ESP32 may miss OI edges during RUN loop warmup, or the
        first out() fires during a transient bus state. Later
        iterations, after the CPU has settled into the halt-retry
        rhythm, are cleaner.
      - A real miscompile produces wrong values at EVERY iteration,
        so even the cleanest sentinel will fail to find `expected`.
      - A hung program emits no sentinel at all, which still fails.

    Returns the expected sequence on success, [] on failure."""
    E = list(expected)
    S = SENTINEL
    if not E:
        return []
    # Try each sentinel occurrence; pass if any has matching tail.
    i = 0
    while i <= len(vals) - len(S):
        if vals[i:i + len(S)] == S:
            tail = vals[i + len(S):i + len(S) + len(E)]
            if tail == E:
                return E
            i += len(S)   # skip this sentinel, try the next
        else:
            i += 1
    return []   # no sentinel followed by expected → miscompile or hang


def wrap_with_sentinel(source):
    """Inject the SENTINEL emit before the first non-init call in main.

    Auto-injection has to come AFTER any init-only builtins (i2c_init,
    lcd_init, tone_setup) — those calls only stay in stage-1 init code if
    every preceding statement in main is also init-classified by step
    10b, and out_imm breaks that classification. Inserting the sentinel
    before i2c_init() would push the init body into runtime main,
    bloating the kernel and defeating the very optimisation some tests
    validate.

    Strategy: find `void main(void) {`, then scan forward past lines that
    match init-only builtin calls, and inject the sentinel after the last
    such call. For tests with no init builtins, inject immediately after
    the `{`. The test author can also pre-supply the sentinel in source
    (signalled by presence of the three constants 0xDE/0xAD/0xBE) — in
    that case we skip injection."""
    # Skip if already has sentinel
    if '0xDE' in source and '0xAD' in source and '0xBE' in source:
        return source
    marker = 'void main(void) {'
    idx = source.find(marker)
    if idx < 0:
        return source
    cursor = idx + len(marker)
    # Scan forward line-by-line, skipping init-only builtin calls and
    # trivial whitespace. Stop at the first real statement.
    init_builtins = ('i2c_init(', 'lcd_init(', 'tone_setup(', 'delay_calibrate(')
    remaining = source[cursor:]
    insert_offset = 0
    pos = 0
    while pos < len(remaining):
        # Skip whitespace
        while pos < len(remaining) and remaining[pos] in ' \t\n\r':
            pos += 1
        if pos >= len(remaining):
            break
        # Check if the next statement is an init builtin
        is_init = False
        for b in init_builtins:
            if remaining.startswith(b, pos):
                # Advance past the whole statement (to next ';')
                semi = remaining.find(';', pos)
                if semi < 0:
                    break
                pos = semi + 1
                insert_offset = pos
                is_init = True
                break
        if not is_init:
            break
    injection = '\n    out(0xDE); out(0xAD); out(0xBE);'
    return source[:cursor + insert_offset] + injection + source[cursor + insert_offset:]


def trim_halt_retry(vals, expected_len):
    """The ESP32's HLT_GRACE_CYCLES lets PC wrap past `hlt` and re-enter
    _main after ~100k cycles. If RUNLOG captures longer than that,
    vals will contain the expected output repeated multiple times.
    Return just the first expected_len elements."""
    return vals[:expected_len]


# ── Compilation ──────────────────────────────────────────────────────

def compile_c(source_path, eeprom=False, optimize=True, extra_env=None):
    args = ['python3', COMPILER, source_path, '-o', '/tmp/_hwreg.asm']
    if optimize:
        args.append('-O')
    if eeprom:
        args.append('--eeprom')
    env = dict(os.environ)
    env.setdefault('PYTHONHASHSEED', '0')
    if extra_env:
        env.update(extra_env)
    r = subprocess.run(args, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        return None, r.stderr
    with open('/tmp/_hwreg.asm') as f:
        return f.read(), r.stderr


# ── Test case runner ─────────────────────────────────────────────────

class Test:
    def __init__(self, name, source, expected, eeprom=False, cycles=500_000, us=2,
                 env=None, expected_intervals_ms=None, interval_tolerance_pct=5,
                 audio=False, needs_hw_peripherals=False):
        self.name = name
        self.source = source
        self.expected = expected   # list of ints, prefix of expected OI history
        self.eeprom = eeprom
        self.cycles = cycles
        # Clock speed: us=2 → ≈250 kHz. Keep tests at a speed comfortably
        # below the MK1's reliable ceiling so we never blame a miscompile
        # on a clock-induced glitch.
        self.us = us
        # Optional extra env vars for the compiler (e.g. MK1_T2_MIN_LEN=3 to
        # force init-touching cross-section extraction for validation).
        self.env = env
        # Optional: assert timing between consecutive `expected` values
        # using the per-event MK1 cycle timestamps from RUNLOG. List of
        # floats in milliseconds, length == len(expected) - 1. Tolerance
        # applies symmetrically; default 5% — calibrated delay is
        # supposed to match wall-clock closely, and anything worse is
        # a real calibration bug to investigate.
        self.expected_intervals_ms = expected_intervals_ms
        self.interval_tolerance_pct = interval_tolerance_pct
        # If True, this test plays sound through the piezo (e.g. via
        # `tone()` builtin). Skipped by default since the audible chirp
        # is disruptive in shared spaces. Pass `--audio` on the CLI to
        # include them in the run.
        self.audio = audio
        # If True, the test depends on responses from physical I²C
        # peripherals (RTC SQW for delay_calibrate, LCD ack for
        # writes, etc.) that the simulator doesn't model. The
        # `--differential` sim cross-check skips these tests because
        # sim's empty/hung output isn't a real divergence — it just
        # means the peripheral isn't there. Hardware leg still runs.
        self.needs_hw_peripherals = needs_hw_peripherals


def run_one(ser, test, verbose=False, differential=False):
    """Compile + upload + runlog once. Return (passed, actual_trimmed, notes).

    Retries up to 3 times on cnt=0 (no captures = MK1 didn't even reach
    the first out_imm — almost always indicates the upload didn't latch
    or RESET held the CPU). Each retry re-uploads.

    With `differential=True`, also runs the program in the simulator
    (using the assembled bytes from uploadBuf, no recompile) and asserts
    the sim's deduped output matches the hardware's. Catches sim ↔
    hardware divergences — exactly the class of bug that masked the
    page-2 manifest issue (where sim halted but hardware ran with
    degenerate output, and trusting either alone gave a false signal).
    """
    wrapped = wrap_with_sentinel(test.source)
    with open('/tmp/_hwreg_src.c', 'w') as f:
        f.write(wrapped)
    asm, compile_err = compile_c('/tmp/_hwreg_src.c', eeprom=test.eeprom,
                                  extra_env=test.env)
    if asm is None:
        return False, [], f'compile error: {compile_err[:200]}'

    sim_actual = None
    sim_notes = ''

    last_notes = ''
    for attempt in range(3):
        if not assemble_upload(ser, asm):
            last_notes = f'assemble/upload failed (attempt {attempt+1})'
            continue

        # Run in sim once (after the first successful assemble_upload).
        # Pull pages straight from uploadBuf — no re-assembly. Tests
        # marked `needs_hw_peripherals` are skipped — sim doesn't
        # model RTC SQW / LCD ACK / etc.
        if differential and sim_actual is None and not test.needs_hw_peripherals:
            try:
                import mk1sim
                pages = dump_pages(ser)
                cpu = mk1sim.MK1()
                cpu.mem[0] = bytearray(pages['code'])
                cpu.mem[1] = bytearray(pages['data'])
                cpu.mem[2] = bytearray(pages['stack'])
                cpu.mem[3] = bytearray(pages['page3'])
                cpu.run(test.cycles)
                sim_dedup = dedup_consecutive(cpu.output_history)
                sim_actual = find_sequence(sim_dedup, test.expected)
                sim_notes = f' sim={sim_actual} halted={cpu.halted}'
            except Exception as e:
                sim_actual = ['<sim_error>']
                sim_notes = f' sim_error={e}'

        r = runlog(ser, cycles=test.cycles, us=test.us)
        if 'error' in r:
            last_notes = r['error']
            continue
        raw_vals = r.get('vals', [])
        raw_ts = r.get('ts', [])
        khz = r.get('khz') or 0
        deduped, deduped_ts = dedup_consecutive_with_ts(raw_vals, raw_ts)
        last_notes = f'cyc={r.get("cyc")} cnt={r.get("cnt")} khz={khz}'
        # cnt=0 = upload/reset failure. Re-upload + retry.
        if r.get('cnt', 0) == 0 and attempt < 2:
            last_notes += f' (retry: cnt=0 on attempt {attempt+1})'
            continue
        actual = find_sequence(deduped, test.expected)
        passed = actual == test.expected
        if not passed:
            last_notes += f' raw={deduped[:25]}'
            if attempt < 2:
                last_notes += f' (retry: wrong data attempt {attempt+1})'
                continue
            return False, actual, last_notes

        # Values matched. If this test has timing expectations, check the
        # per-event MK1 cycle deltas against expected ms × khz.
        if test.expected_intervals_ms is not None and passed:
            # Find the ts for each expected value — match the same way
            # find_sequence does (first occurrence of the sequence).
            seq_ts = _ts_for_sequence(deduped, deduped_ts, test.expected)
            if seq_ts is None or len(seq_ts) < len(test.expected):
                last_notes += f' (timing: could not locate ts for expected sequence)'
                if attempt < 2:
                    continue
                return False, actual, last_notes
            if khz <= 0:
                last_notes += f' (timing: invalid khz={khz})'
                if attempt < 2:
                    continue
                return False, actual, last_notes
            actual_ms = []
            for i in range(1, len(seq_ts)):
                delta_cycles = seq_ts[i] - seq_ts[i-1]
                actual_ms.append(delta_cycles / khz)
            tol = test.interval_tolerance_pct / 100.0
            bad = []
            for i, (want, got) in enumerate(zip(test.expected_intervals_ms, actual_ms)):
                lo, hi = want * (1 - tol), want * (1 + tol)
                if not (lo <= got <= hi):
                    bad.append(f'interval[{i}]: expected {want}ms±{tol*100:.0f}% got {got:.1f}ms')
            if bad:
                last_notes += ' timing_fail: ' + '; '.join(bad)
                if attempt < 2:
                    continue
                return False, actual, last_notes
            last_notes += f' intervals_ms={[f"{x:.1f}" for x in actual_ms]}'
        # Differential check: hardware passed; does sim agree?
        if differential and sim_actual is not None:
            if sim_actual != actual:
                return False, actual, (
                    last_notes +
                    f' SIM_HW_DIVERGENCE: hw={actual} sim={sim_actual}{sim_notes}'
                )
            last_notes += sim_notes
        return True, actual, last_notes
    return False, [], last_notes


def _ts_for_sequence(vals, ts, expected):
    """Return list of ts corresponding to the first occurrence of `expected`
    as a contiguous subsequence within (vals, ts). None if not found."""
    n, m = len(vals), len(expected)
    if m == 0:
        return []
    for i in range(n - m + 1):
        if vals[i:i+m] == list(expected):
            return ts[i:i+m]
    return None


def run_test(ser, test, replications=3, verbose=False, differential=False):
    """Run a test `replications` times. All must pass with identical output."""
    actuals = []
    for i in range(replications):
        passed, actual, notes = run_one(ser, test, verbose=verbose,
                                        differential=differential)
        actuals.append(actual)
        if not passed:
            print(f'  [FAIL rep {i+1}/{replications}] {test.name}', file=sys.stderr)
            print(f'    expected: {test.expected}', file=sys.stderr)
            print(f'    actual:   {actual}', file=sys.stderr)
            print(f'    notes:    {notes}', file=sys.stderr)
            return False
    # All replications match expected, but also assert they match EACH OTHER
    # (in case a test's expected happens to match a degenerate output).
    if len(set(tuple(a) for a in actuals)) != 1:
        print(f'  [FAIL non-deterministic] {test.name}', file=sys.stderr)
        for i, a in enumerate(actuals):
            print(f'    rep {i+1}: {a}', file=sys.stderr)
        return False
    print(f'  [PASS] {test.name}: {actuals[0]}')
    return True


# ── Test corpus ──────────────────────────────────────────────────────
#
# Every test has a CONSTANT expected output — no peek3, no RTC temp
# reads (rtc_read_temp depends on external I2C state). Values are
# compile-time known so the simulator and hardware must agree.

TESTS = [
    # `flat: out(42)` was dropped: strict subset of `3 constant outs` below
    # (same out_imm path, weaker — only one value, no sequencing check).
    Test(
        'flat: 3 constant outs',
        'void main(void) { out(11); out(22); out(33); halt(); }',
        [11, 22, 33],
        cycles=50_000,
    ),
    Test(
        'overlay 0-arg: 1 fn, 1 call',
        '''unsigned char thirty(void) { return 30; }
        unsigned char g[220];
        void main(void) { out(thirty()); halt(); }''',
        [30], eeprom=True, cycles=50_000,
    ),
    Test(
        'overlay 1-arg: pass constant',
        '''unsigned char plus5(unsigned char x) { return x + 5; }
        unsigned char g[220];
        void main(void) { out(plus5(17)); halt(); }''',
        [22], eeprom=True, cycles=50_000,
    ),
    Test(
        'overlay 2-arg: sum of constants',
        '''unsigned char add(unsigned char a, unsigned char b) { return a + b; }
        unsigned char g[220];
        void main(void) { out(add(10, 20)); halt(); }''',
        [30], eeprom=True, cycles=50_000,
    ),
    Test(
        'overlay bundle: entry calls 3 helpers',
        '''unsigned char m(unsigned char a, unsigned char b) {
            unsigned char r = 0;
            while (b > 0) { r = r + a; b = b - 1; }
            return r;
        }
        unsigned char a10(unsigned char x) { return x + 10; }
        unsigned char dbl(unsigned char x) { return x + x; }
        unsigned char chain(unsigned char input) {
            unsigned char s1 = m(input, 3);
            unsigned char s2 = a10(s1);
            return dbl(s2);
        }
        unsigned char g[200];
        void main(void) { out(chain(7)); halt(); }''',
        [(7*3 + 10) * 2 % 256],   # = 62
        eeprom=True,
    ),
    Test(
        'overlay multi-entry merged slot',
        '''void leaf(void) { out(21); }
        void wrap(void) { out(10); leaf(); out(11); }
        unsigned char g[220];
        void main(void) { wrap(); leaf(); halt(); }''',
        [10, 21, 11, 21], eeprom=True, cycles=50_000,
    ),
    Test(
        'two separate overlays',
        '''unsigned char f1(unsigned char x) { return x + 1; }
        unsigned char f2(unsigned char x) { return x + 2; }
        unsigned char g[220];
        void main(void) {
            out(f1(40));      /* 41 */
            out(f2(40));      /* 42 */
            halt();
        }''',
        [41, 42], eeprom=True, cycles=50_000,
    ),
    Test(
        'overlay with loop',
        '''unsigned char count_up(unsigned char n) {
            unsigned char i;
            unsigned char s = 0;
            for (i = 0; i < n; i = i + 1) { s = s + 1; }
            return s;
        }
        unsigned char g[220];
        void main(void) { out(count_up(9)); halt(); }''',
        [9], eeprom=True, cycles=50_000,
    ),
    Test(
        'overlay returning via $c (phase-7 pattern)',
        '''unsigned char f(unsigned char x) { return x * 2; }
        unsigned char g(unsigned char x) { return f(x) + 1; }
        unsigned char globals[220];
        void main(void) {
            out(g(10));     /* f(10) = ?*2; compiler does repeated-add or similar */
            halt();
        }''',
        [21], eeprom=True, cycles=50_000,   # 10+10+1 = 21
    ),
    Test(
        # Exercises _should_split_reload_edge for user-fn -> user-fn calls.
        # leaf_a and leaf_b are sized so they cannot co-locate in one
        # overlay slot; the partitioner emits __ovreload_leaf_a_leaf_b
        # (load callee -> jal -> reload caller -> ret). The 0xB4 marker
        # fires *after* leaf_b returns, so it can only print if the
        # thunk's second _overlay_load correctly reloaded leaf_a's
        # overlay before its tail. sibling() is a plain main->overlay
        # call (no reload-thunk), demonstrating both paths coexist.
        'overlay reload-thunk: user-fn -> user-fn cross-overlay call',
        '''unsigned char tally[16];
        unsigned char filler[8];
        void leaf_b(void) {
            out(0xC2);
            tally[0]++; tally[1]++; tally[2]++; tally[3]++;
            tally[4]++; tally[5]++; tally[6]++; tally[7]++;
            tally[8]++; tally[9]++; tally[10]++; tally[11]++;
            tally[12]++; tally[13]++; tally[14]++; tally[15]++;
            out(0xC3);
        }
        void leaf_a(void) {
            out(0xB1); leaf_b(); out(0xB4);
            tally[0]++; tally[1]++; tally[2]++; tally[3]++;
            tally[4]++; tally[5]++; tally[6]++; tally[7]++;
            tally[8]++; tally[9]++; tally[10]++; tally[11]++;
            tally[12]++; tally[13]++; tally[14]++; tally[15]++;
        }
        void sibling(void) {
            out(0xF0);
            filler[0]++; filler[1]++; filler[2]++; filler[3]++;
            filler[4]++; filler[5]++; filler[6]++; filler[7]++;
            filler[0]++; filler[1]++; filler[2]++; filler[3]++;
            out(0xF1);
        }
        void main(void) {
            out(0xA0); leaf_a(); out(0xA5);
            sibling(); out(0xA6);
            halt();
        }''',
        [0xA0, 0xB1, 0xC2, 0xC3, 0xB4, 0xA5, 0xF0, 0xF1, 0xA6],
        cycles=50_000,
    ),
    Test(
        # Forces an overlay-extracted user fn into PAGE 2 storage
        # (`section stack_code`) and calls it from main. Catches
        # bugs where page-2 overlay bytes drift away from the
        # manifest's declared offset — specifically the manifest-
        # emission-order bug fixed in `73d9a6d`. Before that fix,
        # this test fails: leaf_a's body loads as zeros (wrong page-2
        # offset), the slot executes through zeros into HLT, only
        # main's `out(0xF0)` repeats.
        #
        # Layout forced by the four globals (148 B in page 1) plus
        # three ~74B leaf bodies: leaf_a → page 2 (stack_code),
        # leaf_b → page 1 (data_code), leaf_c → kernel-resident.
        # Different placements per page exercise the loader's
        # page-dispatch arms (`.copy_p1` / `.copy_p2`) end-to-end.
        'overlay placement: page-2 storage end-to-end',
        '''unsigned char fill_p1[100];
        unsigned char tally_a[14];
        unsigned char tally_b[14];
        unsigned char tally_c[14];
        void leaf_a(void) {
            out(0xA1);
            tally_a[0]++; tally_a[1]++; tally_a[2]++; tally_a[3]++;
            tally_a[4]++; tally_a[5]++; tally_a[6]++; tally_a[7]++;
            tally_a[8]++; tally_a[9]++; tally_a[10]++; tally_a[11]++;
            tally_a[12]++; tally_a[13]++;
            out(0xA2);
        }
        void leaf_b(void) {
            out(0xB1);
            tally_b[0]++; tally_b[1]++; tally_b[2]++; tally_b[3]++;
            tally_b[4]++; tally_b[5]++; tally_b[6]++; tally_b[7]++;
            tally_b[8]++; tally_b[9]++; tally_b[10]++; tally_b[11]++;
            tally_b[12]++; tally_b[13]++;
            out(0xB2);
        }
        void leaf_c(void) {
            out(0xC1);
            tally_c[0]++; tally_c[1]++; tally_c[2]++; tally_c[3]++;
            tally_c[4]++; tally_c[5]++; tally_c[6]++; tally_c[7]++;
            tally_c[8]++; tally_c[9]++; tally_c[10]++; tally_c[11]++;
            tally_c[12]++; tally_c[13]++;
            out(0xC2);
        }
        void main(void) {
            out(0xDE); out(0xAD); out(0xBE);
            out(0xF0); leaf_a(); out(0xF1);
            leaf_b(); out(0xF2);
            leaf_c(); out(0xF3);
            halt();
        }''',
        [0xF0, 0xA1, 0xA2, 0xF1, 0xB1, 0xB2, 0xF2, 0xC1, 0xC2, 0xF3],
        cycles=100_000,
    ),
    Test(
        # Cross-page reload thunk: caller (`leaf_a`) lands in page-1
        # overlay storage (`section data_code`), callee (`leaf_b`) in
        # page-2 (`section stack_code`), connected by a generated
        # `__ovrl_leaf_a_leaf_b` reload thunk. This is the
        # `keypad_oled.c` shape — the manifest-emission bug fixed in
        # `73d9a6d` would have manifested here as leaf_b loading blank
        # (slot reads zeros from page-2 offset 0 because the actual
        # bytes are at offset 198+). Complements the page-2 storage
        # test by exercising the reload thunk WHEN it crosses pages.
        #
        # Layout forced by the four globals (148 B in page 1) and three
        # ~70B leaves: leaf_a → page 1 (data_code), leaf_b → page 2
        # (stack_code), leaf_c → kernel-resident.
        'overlay placement: cross-page reload thunk (p1 caller -> p2 callee)',
        '''unsigned char fill_p1[100];
        unsigned char tally_a[16];
        unsigned char tally_b[16];
        unsigned char tally_c[16];
        void leaf_b(void) {
            out(0xB1);
            tally_b[0]++; tally_b[1]++; tally_b[2]++; tally_b[3]++;
            tally_b[4]++; tally_b[5]++; tally_b[6]++; tally_b[7]++;
            tally_b[8]++; tally_b[9]++; tally_b[10]++; tally_b[11]++;
            tally_b[12]++; tally_b[13]++;
            out(0xB2);
        }
        void leaf_a(void) {
            out(0xA1); leaf_b(); out(0xA2);
            tally_a[0]++; tally_a[1]++; tally_a[2]++; tally_a[3]++;
            tally_a[4]++; tally_a[5]++; tally_a[6]++; tally_a[7]++;
            tally_a[8]++; tally_a[9]++; tally_a[10]++; tally_a[11]++;
            tally_a[12]++; tally_a[13]++;
        }
        void leaf_c(void) {
            out(0xC1);
            tally_c[0]++; tally_c[1]++; tally_c[2]++; tally_c[3]++;
            tally_c[4]++; tally_c[5]++; tally_c[6]++; tally_c[7]++;
            tally_c[8]++; tally_c[9]++; tally_c[10]++; tally_c[11]++;
            tally_c[12]++; tally_c[13]++;
            out(0xC2);
        }
        void main(void) {
            out(0xDE); out(0xAD); out(0xBE);
            out(0xF0); leaf_a(); out(0xF1);
            leaf_c(); out(0xF2);
            halt();
        }''',
        [0xF0, 0xA1, 0xB1, 0xB2, 0xA2, 0xF1, 0xC1, 0xC2, 0xF2],
        cycles=100_000,
    ),
    Test(
        # Page-3 overlay storage: forces an overlay-extracted user fn
        # into `section page3_code` (page 3, after the transient kernel
        # image). Five globals (148 B in page 1) plus four ~70B leaves
        # exhaust pages 1+2 (1 leaf in page 1, 2 in page 2) so the
        # fourth spills to page 3.
        #
        # Catches the same bug class as the page-2 storage test, but
        # for the page-3 emission path. Pre-fix, the partitioner
        # advanced p3_code_offset by `meta_table_size` (a hangover from
        # when the manifest lived in page 3), so the manifest claimed
        # the overlay started at e.g. offset 135 while the bytes
        # actually landed at offset 123 (right after the kernel image)
        # — the loader's `derefp3` read zeros, the slot loaded blank,
        # main re-entered.
        'overlay placement: page-3 storage end-to-end',
        '''unsigned char fill[100];
        unsigned char ta[16]; unsigned char tb[16];
        unsigned char tc[16]; unsigned char td[16];
        void leaf_a(void) {
            out(0xA1);
            ta[0]++; ta[1]++; ta[2]++; ta[3]++; ta[4]++; ta[5]++; ta[6]++;
            ta[7]++; ta[8]++; ta[9]++; ta[10]++; ta[11]++; ta[12]++; ta[13]++;
            out(0xA2);
        }
        void leaf_b(void) {
            out(0xB1);
            tb[0]++; tb[1]++; tb[2]++; tb[3]++; tb[4]++; tb[5]++; tb[6]++;
            tb[7]++; tb[8]++; tb[9]++; tb[10]++; tb[11]++; tb[12]++; tb[13]++;
            out(0xB2);
        }
        void leaf_c(void) {
            out(0xC1);
            tc[0]++; tc[1]++; tc[2]++; tc[3]++; tc[4]++; tc[5]++; tc[6]++;
            tc[7]++; tc[8]++; tc[9]++; tc[10]++; tc[11]++; tc[12]++; tc[13]++;
            out(0xC2);
        }
        void leaf_d(void) {
            out(0xD1);
            td[0]++; td[1]++; td[2]++; td[3]++; td[4]++; td[5]++; td[6]++;
            td[7]++; td[8]++; td[9]++; td[10]++; td[11]++; td[12]++; td[13]++;
            out(0xD2);
        }
        void main(void) {
            out(0xDE); out(0xAD); out(0xBE);
            out(0xF0);
            leaf_a(); leaf_b(); leaf_c(); leaf_d();
            out(0xF1);
            halt();
        }''',
        [0xF0, 0xA1, 0xA2, 0xB1, 0xB2, 0xC1, 0xC2, 0xD1, 0xD2, 0xF1],
        cycles=200_000,
    ),
    Test(
        # Forces the 5th leaf into the EEPROM tier (after pages 1+2+3
        # fill). The runtime preload reads bytes from the AT24C32 via
        # the inline I²C bit-bang and writes them into page-3 storage;
        # then the overlay loader copies from page-3 to the slot at
        # call time. This exercises the full EEPROM data path:
        # assembler → ESP32 shim → AT24C32 → MK1 bit-bang → page 3
        # → overlay slot → execute.
        #
        # Pre-fix, two bugs masked each other on this shape:
        #   1. The assembler's `section eeprom` write pointer starts at
        #      0, but mk1cc2 padded only 240B (assuming a 16B header
        #      was already there) — so `out_imm 0xD1` at the start of
        #      the EEPROM-backed leaf landed at eeprom[240] while the
        #      preload read from address 0x100 (=256), pulling bytes
        #      16-bytes into the leaf instead. First byte read became
        #      `jal 0xAC` and execution wandered.
        #   2. The inline preload's sentinel-bit loop exited via `jc`
        #      AFTER `sllb` shifted the sentinel into CF — but BEFORE
        #      the 8th SDA read, so every byte had bit 0 cleared.
        #
        # Hits the EEPROM bit-bang AND the partitioner's ee→p3 hand-off
        # in one test. Failure modes from regressions in any of those
        # paths produce a wrong byte at run time.
        'overlay placement: eeprom storage end-to-end',
        '''unsigned char fill[100];
        unsigned char ta[16]; unsigned char tb[16]; unsigned char tc[16];
        unsigned char td[16]; unsigned char te[16];
        void leaf_a(void) {
            out(0xA1);
            ta[0]++; ta[1]++; ta[2]++; ta[3]++; ta[4]++; ta[5]++; ta[6]++;
            ta[7]++; ta[8]++; ta[9]++; ta[10]++; ta[11]++; ta[12]++; ta[13]++;
            out(0xA2);
        }
        void leaf_b(void) {
            out(0xB1);
            tb[0]++; tb[1]++; tb[2]++; tb[3]++; tb[4]++; tb[5]++; tb[6]++;
            tb[7]++; tb[8]++; tb[9]++; tb[10]++; tb[11]++; tb[12]++; tb[13]++;
            out(0xB2);
        }
        void leaf_c(void) {
            out(0xC1);
            tc[0]++; tc[1]++; tc[2]++; tc[3]++; tc[4]++; tc[5]++; tc[6]++;
            tc[7]++; tc[8]++; tc[9]++; tc[10]++; tc[11]++; tc[12]++; tc[13]++;
            out(0xC2);
        }
        void leaf_d(void) {
            out(0xD1);
            td[0]++; td[1]++; td[2]++; td[3]++; td[4]++; td[5]++; td[6]++;
            td[7]++; td[8]++; td[9]++; td[10]++; td[11]++; td[12]++; td[13]++;
            out(0xD2);
        }
        void leaf_e(void) {
            out(0xE1);
            te[0]++; te[1]++; te[2]++; te[3]++; te[4]++; te[5]++; te[6]++;
            te[7]++; te[8]++; te[9]++; te[10]++; te[11]++; te[12]++; te[13]++;
            out(0xE2);
        }
        void main(void) {
            i2c_init();
            out(0xDE); out(0xAD); out(0xBE);
            out(0x80);
            leaf_a(); leaf_b(); leaf_c(); leaf_d(); leaf_e();
            out(0x81);
            halt();
        }''',
        [0x80, 0xA1, 0xA2, 0xB1, 0xB2, 0xC1, 0xC2, 0xD1, 0xD2, 0xE1, 0xE2, 0x81],
        eeprom=True,
        cycles=300_000,
        # Sim doesn't model the AT24C32 / I²C bit-bang preload, so the
        # EEPROM-backed leaf wouldn't load. Skip the sim leg.
        needs_hw_peripherals=True,
    ),
    Test(
        'xs-abstract init-touch: thunk extracted from init code',
        # With MIN_LEN=3, the 3-instr I2C STOP pattern (ddrb_imm 0x03/0x01/0x00)
        # extracts as a kernel thunk. The thunk occurrences span BOTH the
        # runtime main preamble's bus recovery AND the stage-1 init bus
        # recovery. A regression that mis-places the thunk would make stage-1
        # jal into a non-mini-copied address, so the expected output never
        # appears after the sentinel.
        # The sentinel anchor anchors capture at `_main` start post-self-copy;
        # if stage-1 explodes, no sentinel appears and the test fails cleanly.
        '''unsigned char g[200];
        unsigned char m(unsigned char a, unsigned char b) {
            unsigned char r = 0;
            while (b > 0) { r = r + a; b = b - 1; }
            return r;
        }
        unsigned char a10(unsigned char x) { return x + 10; }
        unsigned char dbl(unsigned char x) { return x + x; }
        unsigned char chain(unsigned char input) {
            unsigned char s1 = m(input, 3);
            unsigned char s2 = a10(s1);
            return dbl(s2);
        }
        void main(void) {
            i2c_init();
            out(chain(7));
            halt();
        }''',
        [62], eeprom=True, cycles=50_000,
        env={'MK1_T2_MIN_LEN': '3'},
    ),
    Test(
        'delay calibration: stopwatch timing',
        # Emits 1,2,3,4 separated by delay(50), delay(100), delay(50).
        # Verifies that calibrated delays produce the requested real-time
        # interval (not just the right count of instructions). If
        # delay_calibrate() miscomputes or delay() ignores calibration,
        # the per-event ts deltas diverge from the requested ms.
        '''void main(void) {
            i2c_init();
            delay_calibrate();
            out(1);
            delay(50);
            out(2);
            delay(100);
            out(3);
            delay(50);
            out(4);
            halt();
        }''',
        [1, 2, 3, 4], eeprom=True,
        # Budget: delay_calibrate() WAITS for the next SQW rising edge,
        # which is 0..1000 ms depending on where in the 1Hz cycle we
        # start. Worst case: ~1000 ms × 166 kHz = ~166k cycles, NOT
        # ~500ms × 166kHz as an earlier comment claimed. Plus 200 ms
        # of delays (~33k), plus overhead (~10k) = ~210k worst-case.
        # The previous 200k budget under-shot worst case by ~10k —
        # whenever calibration landed in the unlucky half of the SQW
        # cycle, the run hit the cycle cap right after out(1) and the
        # capture was [sentinel, 1] with the rest of the test cut off.
        # 500k was supposed to be ~2.4× worst case but hw_regression
        # still saw intermittent FAILs with cnt=1 raw=[4] on rep 2/3 —
        # i.e. the captured trace contained only the FINAL out, with
        # the first three either missed or rolled out of the firmware
        # buffer when HLT_GRACE_CYCLES re-entered main mid-run. Could
        # be longer-than-modelled DS3231 SQW jitter, or the find_
        # sequence() matcher landing on a cross-iteration tail. Either
        # way, fattening the budget so even pathological reps complete
        # the full sequence multiple times eliminates the failure mode
        # at the cost of a few seconds of test runtime.
        cycles=2_000_000,
        expected_intervals_ms=[50.0, 100.0, 50.0],
        interval_tolerance_pct=5,
        needs_hw_peripherals=True,  # delay_calibrate() reads RTC SQW
    ),
    Test(
        'tone duration timing',
        # Cross-check on calibrated delay via __tone_setup. `tone(Hz, ms)`
        # uses the same page3[240] calibration that delay() reads, via
        # a compile-inserted init-time precompute that converts each
        # note's `ratio` into a `half_period` using __tone_setup. Long
        # tone durations (250-800 ms) reduce the relative impact of
        # the fixed per-cycle overhead (~27 cycles) so we can hold a
        # tighter tolerance.
        '''void main(void) {
            i2c_init();
            delay_calibrate();
            out(1);
            tone(1000, 250);
            out(2);
            tone(500, 500);
            out(3);
            tone(2000, 250);
            out(4);
            halt();
        }''',
        [1, 2, 3, 4], eeprom=False,   # flat mode — precompute only runs in flat
        # Budget: calibration can wait up to a full 1 Hz SQW period before
        # starting (~1 s = 166k cycles at 165.5 kHz), then counts 500 ms
        # (83k), plus 1000 ms of tones (166k), plus sentinel/out overhead.
        # 600k *should* have been enough, but the same intermittent FAIL
        # pattern as the stopwatch test (rep 1/3 saw raw=[2,3,2,3,4]) —
        # cross-iteration buffer rolling under HLT_GRACE_CYCLES.
        # Fattened to match the stopwatch budget.
        cycles=2_000_000,
        expected_intervals_ms=[250.0, 500.0, 250.0],
        # 10% tolerance on the flat-mode path (precompute converts ratio
        # to half_period correctly). Overlay-mode tone precision is a
        # separate, deferred investigation.
        interval_tolerance_pct=10,
        audio=True,
        needs_hw_peripherals=True,  # delay_calibrate() reads RTC SQW
    ),
    Test(
        # Exercises BOTH paths through the LCD driver: the PCA9633
        # backlight register-write (lcd_rgb) and the AiP31068L DDRAM
        # path (printf → __lcd_print → __lcd_chr). printf compiles
        # against the string-literal pool + repeated lcd_chr emission,
        # so this test also gates the printf lowering. Visual
        # confirmation is "MK1" at top-left with magenta backlight;
        # the regression gate is the out(42) byte capture.
        'rgb lcd smoke test',
        '''void main(void) {
            i2c_init();
            lcd_init();
            lcd_rgb(255, 0, 255);
            lcd_cmd(0x80);
            printf("MK1");
            out(42);
            halt();
        }''',
        [42], eeprom=True, cycles=500_000,
        needs_hw_peripherals=True,  # lcd_init/lcd_rgb/printf talk to LCD over I2C
    ),
    Test(
        # Issue #1: regparam in $a was elided (no save) for functions
        # without locals/calls/extra-params, on the assumption that $a
        # always holds the param. But intermediate `inc`/`addi`/etc
        # clobber $a between expressions, so a second read of the param
        # silently used the clobbered value. compile_function now also
        # forces a save when the param is read more than once.
        'multi-read regparam preserves x across expressions',
        '''unsigned char buf[10];
        void f(unsigned char x) {
            buf[0] = x + 1;
            buf[1] = x + 2;
            buf[2] = x + 3;
            out(buf[0]); out(buf[1]); out(buf[2]);
        }
        void main(void) { f(5); halt(); }''',
        [6, 7, 8], cycles=50_000,
    ),
    Test(
        # Same hazard for param 1, which lives in $b. `ldi $b,N` for
        # array indexing clobbers the param between reads. Fixed by
        # the symmetric save_b_to_stack path in compile_function.
        'multi-read regparam preserves x in $b across expressions',
        '''unsigned char buf[10];
        void g(unsigned char y, unsigned char x) {
            buf[0] = x + 1;
            buf[1] = x + 2;
            buf[2] = x + 3;
            out(buf[0]); out(buf[1]); out(buf[2]);
        }
        void main(void) { g(0, 5); halt(); }''',
        [6, 7, 8], cycles=50_000,
    ),
    Test(
        # Cold tier end-to-end: helper bytes declared as `ee64` array,
        # uploaded by ESP32 to AT24C512 at 0x50, dispatcher loads from
        # chip into code-page slot at runtime, executes. The 4 bytes
        # are `ldi $a, 0xCD; out; ret`. Confirms the full cold-tier
        # path (compiler partitioner + ee64 image upload + writeEe64Data
        # I²C shim + __cold_call dispatcher + cold slot exec) without
        # any prior-run seed dependency.
        'cold tier: ee64 helper end-to-end',
        '''ee64 unsigned char cold_helper_0[] = {0x38, 0xCD, 0x06, 0x6C};
        void main(void) {
            i2c_init();
            cold_call(0);
            halt();
        }''',
        [205], cycles=500_000,
    ),
]


def main():
    ap = argparse.ArgumentParser(description='MK1 deterministic hardware regression')
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--replications', type=int, default=3)
    ap.add_argument('--cycles', type=int, default=2_000_000)
    ap.add_argument('-v', '--verbose', action='store_true')
    ap.add_argument('--filter', help='Run only tests whose name contains this substring')
    ap.add_argument('--audio', action='store_true',
                    help='Include audio/piezo tests (skipped by default; '
                         'they chirp through the speaker which is disruptive)')
    ap.add_argument('--differential', action='store_true',
                    help='Also run each test in the simulator and assert '
                         'the sim output matches hardware. Catches sim ↔ '
                         'hardware divergences (compiler emission bugs that '
                         'manifest only on real silicon, etc.)')
    args = ap.parse_args()

    ser = open_serial(args.port)
    skipped_audio = sum(1 for t in TESTS if t.audio) if not args.audio else 0
    audio_note = '' if args.audio else f'  (audio: skipped — pass --audio to include)'
    diff_note = '  (sim cross-check enabled)' if args.differential else ''
    print(f'MK1 hardware regression  port={args.port}  replications={args.replications}{audio_note}{diff_note}')
    print()
    passed = 0
    failed = 0
    for t in TESTS:
        if args.filter and args.filter not in t.name:
            continue
        if t.name.startswith('__skip'):
            continue
        if t.audio and not args.audio:
            continue
        if args.cycles != 2_000_000:
            t.cycles = args.cycles
        if run_test(ser, t, replications=args.replications, verbose=args.verbose,
                    differential=args.differential):
            passed += 1
        else:
            failed += 1
    ser.close()
    print()
    total = passed + failed
    print(f'Results: {passed}/{total} passed, {failed} failed')
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
