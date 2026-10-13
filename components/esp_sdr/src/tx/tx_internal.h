/* Transmit engine shared by tx.c and the packet link (link.c). */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Mode allows TX and the whole occupied band fits the region. */
esp_err_t tx_check(uint32_t center_khz, uint32_t deviation_khz);
/* Generator on at the current LO (no tuning); returns the tone word with field 0, tone on. */
uint32_t tx_tone_start(uint8_t atten);
void tx_tone_stop(void);

typedef struct {
    uint32_t *table;  /* (coarse, fine) register pairs */
    size_t steps;
    uint32_t cycles;  /* CPU cycles per step */
    unsigned sub;
    int dev_steps;    /* 39.0625 kHz units */
    bool fine;
} tx_job_t;

esp_err_t tx_job_init(tx_job_t *j, uint32_t bitrate, uint32_t deviation_hz, bool gaussian, size_t nbits);
/* `offset`: fine units added to every step (packet centre relative to the LO). */
void tx_job_fill(tx_job_t *j, const uint8_t *bits, size_t nbits, uint32_t on, int offset);
void tx_job_free(tx_job_t *j);
/* One packet, interrupts masked; returns the worst lateness in cycles. */
int32_t send_once(const uint32_t *table, size_t n, uint32_t cycles, uint32_t idle, bool fine);
