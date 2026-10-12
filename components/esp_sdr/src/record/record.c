#include "record.h"

#include "crc32.h"

IRAM_ATTR uint32_t record_write_header(uint8_t header[ESP_SDR_RECORD_HEADER_SIZE], uint8_t type, uint8_t tag,
                                       uint32_t length)
{
    header[0] = (uint8_t)(ESP_SDR_RECORD_MAGIC & 0xffu);
    header[1] = (uint8_t)(ESP_SDR_RECORD_MAGIC >> 8);
    header[2] = type;
    header[3] = tag;
    header[4] = (uint8_t)length;
    header[5] = (uint8_t)(length >> 8);
    header[6] = (uint8_t)(length >> 16);
    header[7] = (uint8_t)(length >> 24);
    return crc32_update(0, header + 2, ESP_SDR_RECORD_HEADER_SIZE - 2);
}
