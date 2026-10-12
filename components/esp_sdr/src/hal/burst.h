#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_sdr.h"

/* Dump-control rate bits for a rate. */
uint32_t burst_rate_bits(esp_sdr_rate_t rate);
/*
 * One-shot capture of `samples` raw words into the burst window (bank 2).
 * Receive mode must be prepared. Returns ESP_ERR_TIMEOUT if the engine did
 * not finish within 20 ms or left any word unwritten.
 */
esp_err_t burst_capture(esp_sdr_rate_t rate, uint32_t samples, uint32_t *elapsed_us);
uint32_t *burst_buffer(void);
