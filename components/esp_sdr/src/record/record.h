/* Stream record encoding (layout in esp_sdr_record.h). */
#pragma once

#include <stdint.h>
#include "esp_attr.h"
#include "esp_sdr_record.h"

/* Write the 8-byte record header; returns the CRC state covering type .. length.
 * In IRAM: the stream worker on core 1 calls it. */
uint32_t record_write_header(uint8_t header[ESP_SDR_RECORD_HEADER_SIZE], uint8_t type, uint8_t tag, uint32_t length);
