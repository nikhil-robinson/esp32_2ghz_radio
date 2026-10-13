/* The mode chosen at esp_sdr_init(). */
#pragma once

#include <stdbool.h>
#include "esp_sdr.h"

esp_sdr_mode_t sdr_mode(void);

static inline bool sdr_can_rx(void) { return sdr_mode() == ESP_SDR_MODE_RX || sdr_mode() == ESP_SDR_MODE_TXRX; }
static inline bool sdr_can_tx(void) { return sdr_mode() == ESP_SDR_MODE_TX || sdr_mode() == ESP_SDR_MODE_TXRX; }
