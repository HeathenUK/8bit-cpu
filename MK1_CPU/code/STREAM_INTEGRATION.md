# Stream-Execution Integration Plan

Canonical PoC: `stream_poc.asm` (assembled + uploaded by `run_stream_poc.py`).

The PoC verifies on real hardware that an MK1 program can fetch and execute
arbitrary code from the AT24C512 cold tier byte-by-byte, with the slot
permanently shrunk to 3 bytes regardless of streamed function size. This
removes the 256-byte slot ceiling that currently constrains
`__synth_main_body` and `__i2c_stream` for `oled_temp`/`oled_test`.

This document is the implementation guide for wiring the streamer into
`mk1cc2.py`. It splits the work into landed (proven), small-extension
(designed, ready to code), and follow-up (researched, deferred) buckets.

---

## What the PoC verifies on hardware

Tested with `run_stream_poc.py` against a hand-crafted streamed function:

| Feature                        | Verified output                            |
|--------------------------------|--------------------------------------------|
| Linear `out_imm` sequence      | `[0xAA, 0xBB, 0xCC, 0xEE]`                 |
| `jal _emit_kc` (kernel call)   | `0xCA` injected mid-sequence                |
| `j 0x10` (unconditional jump)  | `0xDD/DE/DF` correctly skipped              |
| `ret` (terminate)              | Streamer returns; main continues            |

Streamer state: `vpc_lo` cached at `page3[0xE0]`, slot at `code[252..254]`.

---

## Small extensions to make this production-ready

### (1) Conditional branches — `jz` / `jnz` / `jc` / `jnc`

The flag-preservation problem and its fix, in two paragraphs:

The conditional branch instruction is two bytes: opcode + target. The
streamer must decide *taken vs not-taken* using the flags set by the
*previous* slot exec. But every chip-read (`__i2c_rb`) clobbers flags
internally (`tst`, `decd`, etc.), so by the time the streamer has read
the branch opcode, the flags from the prior instruction are gone.

The MK1 has no flag-save / flag-restore primitives, so we can't snapshot
ZF/CF and replay them. Instead, we exploit the fact that **all
flag-preserving paths ARE flag-preserving**: register moves, `ldp3`,
`derefp3`, `iderefp3`, `mov $a,$pc`, and the conditional-jump
instructions themselves never touch flags. The only flag-clobbering
paths the streamer takes between slot-exec and branch-decision are the
chip reads. So we **pre-fetch** the next instruction's first byte at
the END of each iteration (while flags don't yet matter), classify
it, and stash a per-branch-type handler address at
`page3[next_handler]`. Then immediately after the next iteration's
slot exec, the streamer does:

```
;; inside .post_exec, immediately after slot's `ret` returned:
ldp3 next_handler        ; A = handler addr (preserves flags)
mov $a,$pc                ; jump to handler (preserves flags)
;; one of:
.handle_jz:   jz  .branch_taken  ;  j .branch_not_taken
.handle_jnz:  jnz .branch_taken  ;  j .branch_not_taken
.handle_jc:   jc  .branch_taken  ;  j .branch_not_taken
.handle_jnc:  jnc .branch_taken  ;  j .branch_not_taken
.handle_normal: j .fetch_next     ; no branch — proceed
```

Each handler is 3-4 bytes (jx + j); 4 handlers = ~16 B. Plus the
`ldp3; mov $a,$pc` dispatcher = 4 B. The pre-fetch and classify code
adds ~30 B per iteration.

Why not save flags via `setz; setc`? Because there's no inverse — once
ZF is in `$a`, restoring it to the actual flag register requires a
flag-setting op (e.g. `tst`), which itself sets ZF based on whatever
`$a` is now, not the original ZF. The dispatch-via-`mov $a,$pc`
approach sidesteps this entirely.

**Estimated streamer growth: +50 B kernel-resident.** Plus a 1-byte
page3 slot for `next_handler`. Worth it: every cold-tier helper that
loops or branches becomes streamable.

### (2) 1-byte instruction support

Two clean options, pick one:

**Option A — compiler pads 1B insns to 2B.**  
Every `inc`, `dec`, `mov`, `clr`, `ret`, `push`, `pop`, etc. in a
streamed body gets a NOP (`mov $a,$a`, 0x00) appended. The streamer
stays simple — every fetch is 2 bytes — but EE64 size for streamed
helpers grows by ~30%. EE64 has 64 KB; this is fine.

**Option B — opcode-length table.**  
Pack a 256-bit (= 32 B) bitmap into page 3, where `bit[opcode] = 1`
means 2-byte instruction. The streamer fetches 1 byte, indexes the
table (`ldp3 + bit-shift + and`), and fetches a second byte only for
2-byte ops. ~10 B more streamer code, 32 B more page-3 use, no EE64
inflation.

Option A is simpler to implement and debug. Option B is more
storage-efficient. For first integration, ship Option A; switch to B
later if EE64 budget becomes the binding constraint.

### (3) `jal` to streamed (recursive) target

The PoC handles `jal` only when the target is a code-page address
(kernel helper, user function in main). What if streamed code calls
ANOTHER streamed function?

Recursive streaming would need to save/restore the I²C transaction
state across the call, plus the streamer's own `vpc`. Doable via a
streamer-local stack frame in page 3 (push `vpc` on entry, pop on
return).

In practice, almost no cold helpers in our corpus need this — they
call kernel-resident `__cold_call` for inter-helper dispatch, which
itself isn't streamed. So this can stay deferred until something
actually needs it.

---

## Compiler-side integration

Three changes to `mk1cc2.py`:

### A. Per-helper streaming flag

Each cold helper gets a `streamed` boolean. When set:
- Body bytes are padded (Option A above) or left as-is (Option B).
- Call sites use `jal __stream` with the helper's EE64 offset, not
  `jal __cold_call` with a helper id.

A helper qualifies for streaming if:
- Its body exceeds `COLD_SLOT_BYTES` (won't fit in slot at all), OR
- The auto-cold loop has exhausted Phase 4 splits and the program
  still overflows AND this helper is the slot-floor.

The auto-cold loop slot at the end of `_prepare_compile`'s retry
loop is the natural integration point. Add a step after Phase 4:

```python
# Phase 5: stream-execute the slot-floor helper(s).
if _overflow and ...:
    largest = max(gen.cold_helpers, key=lambda h: h[2])
    largest.streamed = True
    continue  # re-compile
```

### B. Streamer body emission

`_emit_i2c_helpers` adds an emission branch when ANY helper is
streamed. Emit `__stream` (and the `__stream`-specific I²C primitives
if they differ from the dispatcher's). The dispatcher and streamer
can share `__i2c_sb` and `__i2c_rb`.

Streamer is ~150 B kernel-resident (PoC's 219 B includes test
scaffolding). Plus 32 B page-3 metadata. Trade-off vs current ~150 B
dispatcher: net kernel growth ~150 B if BOTH are present, or net
shrink if streamer fully replaces dispatcher.

Replacement is the cleaner path: every cold helper streams, and the
slot disappears. Cost: ~10× cold-call slowdown, no slot-size limit.
For programs where cold-tier is on the cold path (init, infrequent
events), the slowdown is invisible.

### C. Call-site rewriter

The existing `_rewrite_cold_calls` replaces `jal _foo` with
`ldi $b,id; jal __cold_call`. For streamed helpers, replace with:
```
ldi $a, ee64_offset_lo
jal __stream
```

If both modes coexist, dispatch on the helper's `streamed` flag at
rewrite time.

### D. EE64 layout

Streamed helpers don't need `org SLOT_BASE` — there's no slot to fit
into. They can be linked at their EE64 base (= `ee64_alloc` at
emission time). All branches/jumps resolve to EE64 offsets directly.

Caveat: the current emit pass uses `org` to make labels resolve to
the slot address (so the assembler outputs body-relative branch
targets). For streamed helpers, use `org 0` (or equivalent) so
labels resolve to EE64 offsets. The firmware assembler's `org`
behavior in `section ee64` (which snaps `code_size`) needs to be
worked around — emit padding bytes or shuffle section order so the
streamed helper's `org` doesn't truncate uploaded code.

---

## Hardware reliability caveat

The AT24C512 chip-write reliability on this specific board is
sensitive to byte values and positions within a written page. The
streamer PoC's first two bytes (`0xD1 0xAA`) consistently mis-write
to `0xD0 0xA0` (single-bit corruption) at the firmware's current
`PAGE_SZ=8`. `PAGE_SZ=4` clears the streamer issue but breaks the
`caller-arg + retval` regression test. We keep `PAGE_SZ=8` and
accept that streamed programs may need a write-verify pass at
upload time:

```
write_page(addr, bytes)
verify = read_chip(addr, len(bytes))
while verify != bytes:
    write_page(addr, bytes)
    verify = read_chip(addr, len(bytes))
```

This adds ~50 ms per page on retry but makes cold tier byte-exact.
Right move for production. Implementation lives in
`writeEe64Data` in firmware.

---

## What's NOT in scope

- Streaming code that reads/writes its OWN bytes (self-modifying
  streamed helpers). Each fetch overwrites the slot, so any byte
  the helper writes there is lost. Treat as "don't do this".
- Interrupts during streaming (we don't have interrupts).
- Page-boundary crossings within the AT24C512's physical 128-byte
  page (irrelevant — we read sequentially, the chip rolls over
  cleanly across its physical pages).

---

## Test plan when integration lands

1. Compile `oled_temp.c` with streaming enabled for `__i2c_stream`.
   Expect: code-0 fits.
2. Run on hardware. Expect: OLED draws correctly (slower init, but
   functional).
3. Compile `oled_test.c` with streaming enabled for synth chunks.
   Same expectation.
4. `hw_regression.py --replications 1` with streaming feature gated
   off (env var or flag) — expect 25/25.
5. Add a `oled_temp_streamed` and `oled_test_streamed` test to
   hw_regression that explicitly exercises the streamer end-to-end.
