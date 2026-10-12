#include "frame.h"

#define MAGIC_LO (uint8_t)(ESP_SDR_RECORD_MAGIC & 0xffu)
#define MAGIC_HI (uint8_t)(ESP_SDR_RECORD_MAGIC >> 8)

#ifdef ESP_PLATFORM
#include "esp_rom_crc.h"
uint32_t frame_crc32(uint32_t crc, const void *data, size_t size)
{
    return esp_rom_crc32_le(crc, (const uint8_t *)data, (uint32_t)size);
}
#else
uint32_t frame_crc32(uint32_t crc, const void *data, size_t size)
{
    const uint8_t *p = data;
    crc = ~crc;
    while (size--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}
#endif

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

uint32_t frame_write_header(uint8_t header[ESP_SDR_RECORD_HEADER_SIZE], uint8_t type, uint8_t seq, uint32_t length)
{
    header[0] = MAGIC_LO;
    header[1] = MAGIC_HI;
    header[2] = type;
    header[3] = seq;
    for (unsigned i = 0; i < 4; i++) {
        header[4 + i] = (uint8_t)(length >> (8u * i));
    }
    return frame_crc32(0, header + 2, ESP_SDR_RECORD_HEADER_SIZE - 2);
}

void frame_parser_reset(frame_parser_t *parser)
{
    parser->used = 0;
    parser->length = 0;
}

frame_event_t frame_parser_push(frame_parser_t *parser, uint8_t byte)
{
    uint32_t at = parser->used;
    if (at < ESP_SDR_RECORD_HEADER_SIZE) {
        if ((at == 0 && byte != MAGIC_LO) || (at == 1 && byte != MAGIC_HI)) {
            /* A failed second byte may itself start a new magic. */
            parser->used = byte == MAGIC_LO ? 1 : 0;
            parser->header[0] = byte;
            return FRAME_NONE;
        }
        parser->header[at] = byte;
        parser->used = at + 1;
        if (parser->used == ESP_SDR_RECORD_HEADER_SIZE) {
            parser->length = get_u32(parser->header + 4);
            if (parser->length > PROTOCOL_MAX_REQUEST_PAYLOAD) {
                frame_parser_reset(parser);
                return FRAME_TOO_LONG;
            }
        }
        return FRAME_NONE;
    }
    uint32_t body = at - ESP_SDR_RECORD_HEADER_SIZE;
    parser->payload[body] = byte;
    parser->used = at + 1;
    if (body + 1 < parser->length + ESP_SDR_RECORD_CRC_SIZE) {
        return FRAME_NONE;
    }
    uint32_t crc = frame_crc32(0, parser->header + 2, ESP_SDR_RECORD_HEADER_SIZE - 2);
    crc = frame_crc32(crc, parser->payload, parser->length);
    parser->type = parser->header[2];
    parser->seq = parser->header[3];
    parser->used = 0;
    return crc == get_u32(parser->payload + parser->length) ? FRAME_READY : FRAME_BAD_CRC;
}
