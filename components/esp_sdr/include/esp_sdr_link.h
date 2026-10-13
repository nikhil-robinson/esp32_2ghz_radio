/*
 * ESP-SDR two-way packet link (ESP32-S3), after esp_sdr_init(ESP_SDR_MODE_TXRX).
 *
 * nRF24L01+ ShockBurst framing (preamble, 3-5 address bytes in air order,
 * fixed-length payload, CRC 0/1/2 bytes; no ESB packet control field) on one
 * channel, in both directions. Transmitter and on-chip receiver share one LO
 * 3.984 MHz below the channel, so switching direction needs no retune.
 *
 * Both calls mask interrupts on core 0 while they listen and leave the
 * receiver tuned to that LO (call esp_sdr_set_frequency() to move it).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_SDR_LINK_MAX_PAYLOAD 32u

typedef struct {
    uint32_t channel_khz;  /**< carrier; the TX region applies */
    uint32_t bitrate;      /**< 250000, 1000000 or 2000000 */
    uint8_t addr[5];       /**< in air order (first byte sent first) */
    uint8_t addr_len;      /**< 3..5 */
    uint8_t crc_len;       /**< 0, 1 or 2 bytes over address + payload */
    uint8_t atten;         /**< TX attenuation code, 0 = full power */
} esp_sdr_link_t;

typedef struct {
    /** exchange: our last bit -> first bit of the reply; respond: their last bit -> our first bit. ns. */
    uint32_t delay_ns;
    uint32_t crc_failures;  /**< packets for our address rejected before the valid one */
    uint32_t syncs;         /**< sync-word matches (diagnostic) */
    uint32_t edge_errors;   /**< bank switches whose edge was not found: the receiver fell behind */
} esp_sdr_link_info_t;

/**
 * Send `tx` (tx_len bytes), then receive the first packet of rx_len payload
 * bytes with a valid CRC within timeout_us of our last bit.
 * ESP_ERR_TIMEOUT if none arrived; ESP_ERR_INVALID_STATE outside TXRX mode.
 */
esp_err_t esp_sdr_link_exchange(const esp_sdr_link_t *link, const uint8_t *tx, size_t tx_len, uint8_t *rx,
                                size_t rx_len, uint32_t timeout_us, esp_sdr_link_info_t *info);

/**
 * Receive the first packet of rx_len payload bytes with a valid CRC (within
 * timeout_ms), then send `reply` starting turnaround_us after its last bit
 * (or as soon as the transmitter is ready, if that is later: see delay_ns).
 */
esp_err_t esp_sdr_link_respond(const esp_sdr_link_t *link, uint8_t *rx, size_t rx_len, const uint8_t *reply,
                               size_t reply_len, uint32_t turnaround_us, uint32_t timeout_ms,
                               esp_sdr_link_info_t *info);

#ifdef __cplusplus
}
#endif
