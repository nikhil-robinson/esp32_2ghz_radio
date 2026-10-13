/* Receive framing for the packet link (pure C, host-tested): nRF24 ShockBurst. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_MAX_PAYLOAD 32u
#define LINK_MAX_BODY_BYTES (LINK_MAX_PAYLOAD + 2u)

typedef struct {
    uint32_t sync;       /* last 32 air bits before the payload: preamble + address */
    uint16_t body_bits;  /* payload + CRC */
    uint8_t addr[5];
    uint8_t addr_len;
    uint8_t crc_len;
    uint8_t payload_len;
} link_rx_plan_t;

/* False if the framing is not supported (address 3..5 bytes, CRC 0..2, payload 1..32). */
bool link_rx_plan(link_rx_plan_t *p, const uint8_t *addr, size_t addr_len, unsigned crc_len, size_t payload_len);
/* body = payload then CRC, as received (MSB first). True if the CRC checks (always for CRC 0). */
bool link_rx_check(const link_rx_plan_t *p, const uint8_t *body);
