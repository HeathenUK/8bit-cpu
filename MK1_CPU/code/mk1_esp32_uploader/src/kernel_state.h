// MK1 kernel-state offsets — single source of truth shared with the
// compiler. Mirror of mk1cc2.py:KERNEL_STATE. Add new entries to BOTH
// files; the count is small enough that hand-maintained sync is fine.
//
// Each entry names a memory location reserved for runtime helpers
// (calibration constants, one-shot guards, kernel counters). These
// were previously hardcoded as magic numbers (240/241/242 on page 3)
// at every emission site; Phase 3 of the kernel-state allocator
// refactor moved them into the structured page-2 kstate region
// (0x80..0xAF) defined by the page-2 partition.
//
// Page convention (STK + HL bits select page): 00 → page 0 / code,
// 01 → page 1 / data, 10 → page 2 / stack, 11 → page 3. All current
// kernel-state slots live on page 2 and are accessed via deref2 /
// ideref2 from MK1 asm.
//
// Page-2 partition (low to high):
//   0x00..0x7F   overlay slot     (128 B; Phase 4 activates)
//   0x80..0xAF   kernel state     ( 48 B; THIS FILE'S DOMAIN)
//   0xB0..0xBF   guard band       ( 16 B; overflow buffer)
//   0xC0..0xFF   stack reserved   ( 64 B; SP grows down from 0xFF)

#pragma once

#define KSTATE_PAGE                   2

#define KSTATE_CALIB_BLOCKS           0x80   // F/4608 — DS3231 SQW HIGH-phase
                                             // block count, set by __delay_cal,
                                             // consumed by __delay_Nms and
                                             // __tone_setup.

#define KSTATE_TONE_PRECOMP_DONE      0x81   // one-shot guard: non-zero means
                                             // the note table has been converted
                                             // from ratios to half_periods.

#define KSTATE_OVERLAY_R2C_COUNT      0x82   // remaining-bytes counter for the
                                             // sequential I2C → code page reads
                                             // in __eeprom_r2c_loop.
