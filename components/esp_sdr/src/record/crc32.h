/* IEEE CRC-32 (zlib compatible), chainable: crc32_update(crc32_update(0, a), b). ROM on target. */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "esp_rom_crc.h"
static inline uint32_t crc32_update(uint32_t crc, const void *data, size_t size)
{
    return esp_rom_crc32_le(crc, (const uint8_t *)data, (uint32_t)size);
}
#else
uint32_t crc32_update(uint32_t crc, const void *data, size_t size);
#endif
