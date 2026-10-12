/*
 * Decimated IQ stream: each unit passes a two-stage decimating FIR
 * (8x, then decimation/8) and the output is rounded, saturated to 4/8/16
 * bits and packed into IQ_FRAMEs of up to 1024 payload bytes. Frames carry
 * the decimated sample index of their first sample, so the host sees gaps.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stream.h"
#include "worker.h"

/* Validates the configuration and allocates filters; on failure returns false
 * and sets *detail (2: arguments, 3: frame buffer, 4: filter memory). */
bool iq_stream_begin(uint32_t *detail);

/* ---- single-core path --------------------------------------------------- */
void iq_stream_accept_unit(const stream_unit_t *unit);
bool iq_stream_pending(void);
/* Worst slice seen so far, cycles (scheduling budget). */
uint32_t iq_stream_slice_cost(void);
/* Filter up to 1024 pairs of the pending unit; returns cycles spent. */
uint32_t iq_stream_slice(void);
void iq_stream_release_bank(unsigned bank);

/* Flush, report lost pairs and free the filters. */
void iq_stream_finish(bool dual);
extern const worker_hooks_t iq_stream_worker_hooks;
