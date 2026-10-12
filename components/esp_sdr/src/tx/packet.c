/* Packet framing (BLE Core Vol 6 Part B 2.1 and 3.1-3.2; nRF24L01+ datasheet 7.3). */
#include "packet.h"

#include <stdbool.h>
#include <string.h>

#define BLE_ACCESS_ADDRESS 0x8E89BED6u
#define BLE_ADV_NONCONN_IND_TXRANDOM 0x42u

typedef struct {
    uint8_t *out;
    size_t bits;
} bit_writer_t;

static void put_bit(bit_writer_t *w, unsigned bit)
{
    uint8_t mask = (uint8_t)(0x80u >> (w->bits & 7u));
    if (bit) {
        w->out[w->bits >> 3] |= mask;
    } else {
        w->out[w->bits >> 3] &= (uint8_t)~mask;
    }
    w->bits++;
}

static void put_byte(bit_writer_t *w, uint8_t v, bool lsb_first)
{
    for (unsigned i = 0; i < 8; i++) {
        put_bit(w, lsb_first ? (v >> i) & 1u : (v >> (7 - i)) & 1u);
    }
}

/* CRC-24 x^24+x^10+x^9+x^6+x^4+x^3+x+1, init 0x555555, data LSB first.
 * Returns the CRC in transmission order: bit 23 is sent first. */
uint32_t packet_ble_crc24(const uint8_t *pdu, size_t len)
{
    uint32_t crc = 0x555555u;
    while (len--) {
        uint8_t v = *pdu++;
        for (unsigned i = 0; i < 8; i++, v >>= 1) {
            unsigned top = (crc >> 23) & 1u;
            crc = (crc << 1) & 0xFFFFFFu;
            if (top != (v & 1u)) {
                crc ^= 0x00065Bu;
            }
        }
    }
    return crc;  /* register bit 23 is sent first */
}

/* Whitening LFSR x^7+x^4+1 seeded with 1 then the channel index, kept
 * bit-reversed in a byte: bit 7 is the output, bit 1 the seed's leading 1. */
static void ble_whiten(uint8_t *data, size_t len, uint8_t channel)
{
    uint8_t lfsr = 2u;
    for (unsigned i = 0; i < 6; i++) {
        lfsr |= (uint8_t)(((channel >> i) & 1u) << (7 - i));
    }
    for (size_t n = 0; n < len; n++) {
        for (unsigned i = 0; i < 8; i++) {
            if (lfsr & 0x80u) {
                lfsr ^= 0x11u;
                data[n] ^= (uint8_t)(1u << i);
            }
            lfsr <<= 1;
        }
    }
}

size_t packet_ble_adv(uint8_t *out, uint8_t channel, const uint8_t adv_addr[6],
                      const uint8_t *data, size_t len)
{
    if (channel < 37 || channel > 39 || len > PACKET_BLE_MAX_DATA || (len && !data)) {
        return 0;
    }
    uint8_t pdu[2 + 6 + PACKET_BLE_MAX_DATA + 3];
    size_t n = 0;
    pdu[n++] = BLE_ADV_NONCONN_IND_TXRANDOM;
    pdu[n++] = (uint8_t)(6 + len);
    memcpy(pdu + n, adv_addr, 6);
    n += 6;
    if (len) {
        memcpy(pdu + n, data, len);
        n += len;
    }
    uint32_t crc = packet_ble_crc24(pdu, n);
    /* CRC bytes in pdu[] use the LSB-first convention like the rest. */
    for (unsigned b = 0; b < 3; b++) {
        uint8_t v = 0;
        for (unsigned i = 0; i < 8; i++) {
            v |= (uint8_t)(((crc >> (23 - 8 * b - i)) & 1u) << i);
        }
        pdu[n++] = v;
    }
    ble_whiten(pdu, n, channel);

    bit_writer_t w = {.out = out};
    put_byte(&w, (BLE_ACCESS_ADDRESS & 1u) ? 0x55u : 0xAAu, true);  /* last preamble bit differs from AA bit 0 */
    for (unsigned i = 0; i < 4; i++) {
        put_byte(&w, (uint8_t)(BLE_ACCESS_ADDRESS >> (8 * i)), true);
    }
    for (size_t i = 0; i < n; i++) {
        put_byte(&w, pdu[i], true);
    }
    return w.bits;
}

uint8_t packet_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFFu;
    while (len--) {
        crc ^= *data++;
        for (unsigned i = 0; i < 8; i++) {
            crc = (uint8_t)((crc & 0x80u) ? ((unsigned)crc << 1) ^ 0x07u : (unsigned)crc << 1);
        }
    }
    return crc;
}

uint16_t packet_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= (uint16_t)(*data++ << 8);
        for (unsigned i = 0; i < 8; i++) {
            crc = (uint16_t)((crc & 0x8000u) ? ((unsigned)crc << 1) ^ 0x1021u : (unsigned)crc << 1);
        }
    }
    return crc;
}

size_t packet_nrf24(uint8_t *out, const uint8_t *addr, size_t addr_len, unsigned crc_len,
                    const uint8_t *payload, size_t len)
{
    if (addr_len < 3 || addr_len > 5 || crc_len > 2 || len > PACKET_NRF24_MAX_PAYLOAD || (len && !payload)) {
        return 0;
    }
    uint8_t body[5 + PACKET_NRF24_MAX_PAYLOAD + 2];
    memcpy(body, addr, addr_len);
    if (len) {
        memcpy(body + addr_len, payload, len);
    }
    size_t n = addr_len + len;
    if (crc_len == 1) {
        body[n] = packet_crc8(body, n);
        n += 1;
    } else if (crc_len == 2) {
        uint16_t crc = packet_crc16(body, n);
        body[n] = (uint8_t)(crc >> 8);
        body[n + 1] = (uint8_t)crc;
        n += 2;
    }
    bit_writer_t w = {.out = out};
    put_byte(&w, (addr[0] & 0x80u) ? 0xAAu : 0x55u, false);  /* last preamble bit differs from the first address bit */
    for (size_t i = 0; i < n; i++) {
        put_byte(&w, body[i], false);
    }
    return w.bits;
}
