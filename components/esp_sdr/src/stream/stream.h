/*
 * Internal interfaces of the continuous stream engine (ring.c) and its
 * processors: spectrum.c (on-chip FFT), iq_stream.c (decimated IQ) and
 * worker.c (second-core runner).
 *
 * A "unit" is the gapless span of samples the dump engine wrote into one bank
 * between two bank switches (~12288 pairs). Units are processed while later
 * units are being captured; a unit whose bank is about to be reused before
 * its processing finished is abandoned (counted), never waited for.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_attr.h"
#include "esp_sdr.h"

/* Code on the bank-switch deadline path must never wait for a flash cache miss. */
#define STREAM_HOT IRAM_ATTR
#define MEMORY_BARRIER() __asm__ volatile("memw" ::: "memory")

/* Shared by all processors for the duration of one run. */
typedef struct {
    const esp_sdr_run_config_t *config;
    const esp_sdr_sink_t *sink;
    esp_sdr_run_result_t *result;
} stream_run_t;
extern stream_run_t g_stream;

/* A located unit handed to a processor. */
typedef struct {
    uint32_t seq;     /* worker queue sequence (bank ownership token) */
    uint32_t bank;
    uint32_t first;   /* ring index of the unit's first pair */
    uint32_t count;   /* pairs */
    uint64_t index;   /* stream index of the first pair (gapless clock) */
    uint8_t gain;     /* gain field (bits 20..27) of the first word */
} stream_unit_t;

/* Busy CPU cycles per core during the run, for STATS frames. */
typedef struct {
    volatile uint32_t core0;
    volatile uint32_t core1;
} stream_load_t;
extern stream_load_t g_load;

/* Run one stream (ring.c). Receive mode and filter are set by the caller. */
void stream_run(const esp_sdr_run_config_t *config, const esp_sdr_sink_t *sink, esp_sdr_run_result_t *result);

/* ---- output queue (tx_queue.c) ----------------------------------------- */
#define TX_QUEUE_SIZE 16384u  /* 7 spectra of 2048 bins, 56 of 256 */
void tx_queue_reset(void);
/* Core 0 producer. False (nothing written) when the frame does not fit. */
bool tx_queue_push(const void *data, uint32_t size);
/* Core 1 producer (publishes with a barrier). Never mixed with core 0 pushes in one run. */
bool tx_queue_push_from_worker(const void *data, uint32_t size);
/* Offer queued bytes to the sink (at most one 64-byte chunk); never waits. */
void tx_queue_pump(void);
bool tx_queue_empty(void);
uint32_t tx_queue_consumed(void);  /* free-running count of bytes sent */
uint32_t tx_queue_fill_permille(void);
