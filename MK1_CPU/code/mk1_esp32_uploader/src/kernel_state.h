// MK1 kernel-state offsets — single source of truth shared with the
// compiler. Mirror of mk1cc2.py:KERNEL_STATE. Add new entries to BOTH
// files; the count is small enough that hand-maintained sync is fine.
//
// Each entry names a memory location reserved for runtime helpers
// (calibration constants, one-shot guards, kernel counters). These
// were previously hardcoded as magic numbers (240/241/242 on page 3)
// at every emission site; Phase 3 of the kernel-state allocator
// refactor moved them into the structured page-2 kstate region.
//
// Page convention (STK + HL bits select page): 00 → page 0 / code,
// 01 → page 1 / data, 10 → page 2 / stack, 11 → page 3. All current
// kernel-state slots live on page 2 and are accessed via deref2 /
// ideref2 from MK1 asm.
//
// Page-2 partition (low to high) — refactored 2026-04-25 to a 16 B
// stack (corpus peak depth = 9 B was deeply over-provisioned at the
// original 64 B), giving the freed 48 B to the page-2 overlay slot:
//   0x00..0xAF   overlay slot     (176 B)
//   0xB0..0xDF   kernel state     ( 48 B; THIS FILE'S DOMAIN)
//   0xE0..0xEF   guard band       ( 16 B; overflow buffer)
//   0xF0..0xFF   stack reserved   ( 16 B; SP grows down from 0xFF)

#pragma once

#define KSTATE_PAGE                   2

#define KSTATE_CALIB_BLOCKS           0xB0   // F/4608 — DS3231 SQW HIGH-phase
                                             // block count, set by __delay_cal,
                                             // consumed by __delay_Nms and
                                             // __tone_setup.

#define KSTATE_TONE_PRECOMP_DONE      0xB1   // one-shot guard: non-zero means
                                             // the note table has been converted
                                             // from ratios to half_periods.

#define KSTATE_OVERLAY_R2C_COUNT      0xB2   // remaining-bytes counter for the
                                             // sequential I2C → code page reads
                                             // in __eeprom_r2c_loop.

#define KSTATE_MANIFEST_BASE          0xC0   // overlay manifest + pages array;
                                             // each manifest entry is 2 B
                                             // (offset, size); pages array is
                                             // 1 B per overlay following the
                                             // manifest.
