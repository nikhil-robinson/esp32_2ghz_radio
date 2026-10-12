/*
 * Firmware frame codec: the record layout of esp_sdr_record.h, used for both
 * requests and responses. Pure C (no hardware) so host tests can build it.
 *
 *   E5 D5 | type | seq | length u32 | payload | crc32 over type..payload
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "protocol.h"

/* Write an 8-byte header; returns the CRC state covering type, seq, length. */
uint32_t frame_write_header(uint8_t header[ESP_SDR_RECORD_HEADER_SIZE], uint8_t type, uint8_t seq, uint32_t length);
/* IEEE CRC-32 (zlib), chainable. */
uint32_t frame_crc32(uint32_t crc, const void *data, size_t size);

typedef enum {
    FRAME_NONE = 0,  /* need more bytes */
    FRAME_READY,     /* type/seq/payload/length hold a request */
    FRAME_BAD_CRC,   /* complete frame, CRC mismatch (dropped) */
    FRAME_TOO_LONG,  /* declared length exceeds PROTOCOL_MAX_REQUEST_PAYLOAD (dropped) */
} frame_event_t;

typedef struct {
    uint8_t header[ESP_SDR_RECORD_HEADER_SIZE];
    uint8_t payload[PROTOCOL_MAX_REQUEST_PAYLOAD + ESP_SDR_RECORD_CRC_SIZE];
    uint32_t used;    /* bytes of the current frame received */
    uint32_t length;  /* payload length of the current frame */
    uint8_t type;
    uint8_t seq;
} frame_parser_t;

void frame_parser_reset(frame_parser_t *parser);
/* Feed one byte. Bytes before a magic are skipped (resynchronisation). */
frame_event_t frame_parser_push(frame_parser_t *parser, uint8_t byte);
static inline bool frame_parser_busy(const frame_parser_t *parser) { return parser->used != 0; }
