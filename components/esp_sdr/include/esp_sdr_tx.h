/*
 * ESP-SDR transmitter (ESP32-S3). Available after esp_sdr_init(ESP_SDR_MODE_TX).
 *
 * The carrier is the PHY tone generator; the CPU modulates its frequency
 * (2-FSK, optionally Gaussian-shaped) from a table built beforehand, with
 * interrupts masked on core 0 for each packet. Interrupts are back on
 * between packets. Every call returns with the carrier off, except
 * esp_sdr_tx_tone().
 *
 * Deviation is in steps of 40 MHz / 1024 = 39.0625 kHz, except Gaussian
 * shaping at 2 Mbps, which uses whole 156.25 kHz steps (CPU time per step).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Tone generator frequency step: 40 MHz / 1024 = 39.0625 kHz, in mHz. */
#define ESP_SDR_TX_STEP_MILLIHZ 39062500u
#define ESP_SDR_TX_MAX_BITS 2048u
#define ESP_SDR_TX_MAX_COUNT 65535u

typedef enum {
    ESP_SDR_REGION_ISM = 0,          /**< 2400.0-2483.5 MHz, licence-free (default) */
    ESP_SDR_REGION_HAM = 1,          /**< 2300-2450 MHz, licensed radio amateurs only */
    ESP_SDR_REGION_UNRESTRICTED = 2, /**< 2210-2780 MHz, shielded / lab use only */
} esp_sdr_region_t;

/** Choose where the transmitter may operate. Anything but ISM logs a warning. */
esp_err_t esp_sdr_tx_set_region(esp_sdr_region_t region);
esp_sdr_region_t esp_sdr_tx_get_region(void);

/**
 * Unmodulated carrier at `khz` until esp_sdr_tx_off(). `atten` is the PHY's
 * attenuation code: 0 = full power, larger = less (not calibrated).
 */
esp_err_t esp_sdr_tx_tone(uint32_t khz, uint8_t atten);
esp_err_t esp_sdr_tx_off(void);

typedef struct {
    uint32_t center_khz;
    uint32_t bitrate;       /**< 250000, 1000000 or 2000000 */
    uint32_t deviation_hz;  /**< rounded to 39.0625 kHz steps, at least one */
    bool gaussian;          /**< BT 0.5 shaping */
    uint8_t atten;
} esp_sdr_fsk_t;

/**
 * Send `nbits` bits (packed, first bit = bit 7 of bits[0]; 1 = +deviation)
 * `count` times with `gap_us` of silence between. ESP_ERR_TIMEOUT if the
 * CPU fell behind the bit clock (the packet went out distorted).
 */
esp_err_t esp_sdr_tx_bits(const esp_sdr_fsk_t *fsk, const uint8_t *bits, size_t nbits,
                          uint16_t count, uint32_t gap_us);

/** BLE ADV_NONCONN_IND on channel 37..39 at 1 Mbps; adv_addr LSB first (random address). */
esp_err_t esp_sdr_tx_ble_adv(uint8_t channel, const uint8_t adv_addr[6], const uint8_t *data, size_t len,
                             uint8_t atten, uint16_t count, uint32_t gap_us);

typedef struct {
    uint8_t addr[5];        /**< in air order (first byte sent first) */
    uint8_t addr_len;       /**< 3..5 */
    uint8_t crc_len;        /**< 0, 1 or 2 bytes */
    uint8_t channel;        /**< 0..125: 2400 + channel MHz */
    uint32_t bitrate;       /**< 250000, 1000000 or 2000000 */
    uint8_t atten;
} esp_sdr_nrf24_t;

/** nRF24L01+ ShockBurst packet (no ESB: no packet control field, no ACK). */
esp_err_t esp_sdr_tx_nrf24(const esp_sdr_nrf24_t *config, const uint8_t *payload, size_t len,
                           uint16_t count, uint32_t gap_us);

#ifdef __cplusplus
}
#endif
