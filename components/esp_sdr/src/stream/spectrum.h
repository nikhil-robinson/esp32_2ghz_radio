/*
 * On-chip spectrum: every unit is cut into nfft-pair blocks; each block is
 * windowed (Hann), transformed (int16 radix-2 FFT), DC-corrected and its power
 * accumulated (mean or max-hold). One output frame per unit (or later, when
 * the sink is still busy) carries 0.5 dB codes in natural bin order.
 *
 * Single-core mode runs the block pipeline in slices between ring polls.
 * Dual mode runs whole blocks on core 1; core 0 may "assist" by transforming
 * single blocks into a hand-off buffer when that fits before a deadline.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stream.h"
#include "worker.h"

bool spectrum_init(void);
bool spectrum_valid_nfft(uint32_t nfft);
void spectrum_set_dc_mode(uint8_t mode);

/* Run setup (outside the masked section is fine). Seeds result->work_max.
 * Returns the measured cycles of one core-0 block (unpack + FFT). */
uint32_t spectrum_begin(bool dual, bool assist);

/* ---- single-core path --------------------------------------------------- */
void spectrum_accept_unit(const stream_unit_t *unit);
void spectrum_release_bank(unsigned bank);
bool spectrum_bank_idle(unsigned bank);
/* One bounded slice of work; false when idle. */
bool spectrum_work_slice(void);
void spectrum_finish_single(void);

/* ---- dual-core path ----------------------------------------------------- */
void spectrum_assist(void);
uint32_t spectrum_assist_blocks(void);
extern const worker_hooks_t spectrum_worker_hooks;
