#include "link_frame.h"

#include <string.h>
#include "packet.h"

bool link_rx_plan(link_rx_plan_t *p, const uint8_t *addr, size_t addr_len, unsigned crc_len, size_t payload_len)
{
    if (!addr || addr_len < 3 || addr_len > 5 || crc_len > 2 || !payload_len || payload_len > LINK_MAX_PAYLOAD) {
        return false;
    }
    uint8_t head[6];
    head[0] = (addr[0] & 0x80u) ? 0xAAu : 0x55u;  /* preamble, as packet_nrf24() sends it */
    memcpy(head + 1, addr, addr_len);
    const uint8_t *last = head + 1 + addr_len - 4;
    *p = (link_rx_plan_t){
        .sync = (uint32_t)last[0] << 24 | (uint32_t)last[1] << 16 | (uint32_t)last[2] << 8 | last[3],
        .body_bits = (uint16_t)(8u * (payload_len + crc_len)),
        .addr_len = (uint8_t)addr_len,
        .crc_len = (uint8_t)crc_len,
        .payload_len = (uint8_t)payload_len,
    };
    memcpy(p->addr, addr, addr_len);
    return true;
}

bool link_rx_check(const link_rx_plan_t *p, const uint8_t *body)
{
    if (!p->crc_len) {
        return true;
    }
    uint8_t covered[5 + LINK_MAX_PAYLOAD];
    memcpy(covered, p->addr, p->addr_len);
    memcpy(covered + p->addr_len, body, p->payload_len);
    size_t n = p->addr_len + p->payload_len;
    const uint8_t *crc = body + p->payload_len;
    if (p->crc_len == 1) {
        return crc[0] == packet_crc8(covered, n);
    }
    uint16_t c = packet_crc16(covered, n);
    return crc[0] == (uint8_t)(c >> 8) && crc[1] == (uint8_t)c;
}
