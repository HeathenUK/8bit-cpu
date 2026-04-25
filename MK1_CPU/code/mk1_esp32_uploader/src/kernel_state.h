// MK1 kernel-state offsets — single source of truth shared with the
// compiler. Mirror of mk1cc2.py:KERNEL_STATE. Add new entries to BOTH
// files; the count is small enough that hand-maintained sync is fine.
//
// Each entry names a memory location reserved for runtime helpers
// (calibration constants, one-shot guards, kernel counters). These
// were previously hardcoded as magic numbers (240/241/242) at every
// emission site, with non-collision assumed by inspection.
//
// Page convention: STK + HL bits select page (00 → page 0 / code,
// 01 → page 1 / data, 10 → page 2 / stack, 11 → page 3). All current
// kernel-state slots live on page 3 and are accessed via derefp3 /
// iderefp3 from MK1 asm. The Phase-3 refactor (see WORKLOG.md) will
// migrate these to page 2.

#pragma once

// Page-3 kernel-state offsets.
#define KSTATE_PAGE                   3

#define KSTATE_CALIB_BLOCKS           240   // F/4608 — DS3231 SQW HIGH-phase
                                            // block count, set by __delay_cal,
                                            // consumed by __delay_Nms and
                                            // __tone_setup.

#define KSTATE_TONE_PRECOMP_DONE      241   // one-shot guard: non-zero means
                                            // the note table has been converted
                                            // from ratios to half_periods.

#define KSTATE_OVERLAY_R2C_COUNT      242   // remaining-bytes counter for the
                                            // sequential I2C → code page reads
                                            // in __eeprom_r2c_loop.
