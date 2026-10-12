/*
 * Low-pass taps for the two-stage decimating IQ filter (Q15, unity DC gain).
 * Kaiser-windowed sinc, beta 5.65 (~60 dB stop band), cut-off 0.5 / D.
 */
#pragma once

#include <stdint.h>

#define FIR_STAGE1_DECIMATION 8u
#define FIR_STAGE1_TAPS 32u
#define FIR_STAGE2_MAX_TAPS 256u

/*
 * Stage 1 (decimate by 8, 32 taps): an 8-boxcar squared (15-tap triangle)
 * convolved with an 18-tap Kaiser low pass. The triangle puts double zeros on
 * every multiple of fs/8 - exactly the bands that alias onto 0 Hz after the
 * 8x decimation, including LO leakage at +fs/4 in rotate mode.
 */
void fir_design_stage1(int16_t taps[FIR_STAGE1_TAPS]);
/* Stage 2: decimation d2, `count` taps. */
void fir_design_lowpass(int16_t *taps, uint32_t count, uint32_t decimation);
/* Stage-2 tap count for decimation d2: 18 per output, multiple of 8, at most 256. */
uint32_t fir_stage2_taps(uint32_t d2);
