/*
 * Packet framing for the transmitter: pure C, no hardware.
 *
 * Every builder writes the packet as on-air bits, packed MSB first (the first
 * bit sent is bit 7 of out[0]), and returns the number of bits, or 0 if an
 * argument is invalid.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define PACKET_BLE_MAX_DATA 31u
#define PACKET_BLE_MAX_BYTES (1u + 4u + 2u + 6u + PACKET_BLE_MAX_DATA + 3u)
#define PACKET_NRF24_MAX_PAYLOAD 32u
#define PACKET_NRF24_MAX_BYTES (1u + 5u + PACKET_NRF24_MAX_PAYLOAD + 2u)

/* BLE ADV_NONCONN_IND on channel 37..39 (random AdvA, given LSB first as in HCI). */
size_t packet_ble_adv(uint8_t *out, uint8_t channel, const uint8_t adv_addr[6],
                      const uint8_t *data, size_t len);

/* nRF24 ShockBurst without ESB: address in air order, crc_len 0, 1 or 2. */
size_t packet_nrf24(uint8_t *out, const uint8_t *addr, size_t addr_len, unsigned crc_len,
                    const uint8_t *payload, size_t len);

/* Exposed for tests. */
uint32_t packet_ble_crc24(const uint8_t *pdu, size_t len);
uint8_t packet_crc8(const uint8_t *data, size_t len);
uint16_t packet_crc16(const uint8_t *data, size_t len);
