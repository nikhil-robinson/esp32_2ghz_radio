#include "iq_pack.h"

#define IQ20_MASK 0xfffffu

size_t iq_pack10_size(uint32_t samples)
{
    return ((size_t)samples * 20u + 7u) / 8u;
}

size_t iq_pack10(uint32_t *words, uint32_t samples)
{
    uint8_t *out = (uint8_t *)words;
    for (uint32_t j = 0; j < samples; j += 2, out += 5) {
        /* Read both words before writing: out[0..4] overlaps words[j/...]. */
        uint32_t a = words[j] & IQ20_MASK;
        uint32_t b = j + 1 < samples ? words[j + 1] & IQ20_MASK : 0;
        out[0] = (uint8_t)a;
        out[1] = (uint8_t)(a >> 8);
        out[2] = (uint8_t)((a >> 16) | (b << 4));
        if (j + 1 < samples) {
            out[3] = (uint8_t)(b >> 4);
            out[4] = (uint8_t)(b >> 12);
        }
    }
    return iq_pack10_size(samples);
}

size_t iq_pack8(uint32_t *words, uint32_t samples)
{
    uint8_t *out = (uint8_t *)words;
    for (uint32_t j = 0; j < samples; j++) {
        uint32_t w = words[j];
        out[2 * j] = (uint8_t)(w >> 2);       /* I bits 2..9 */
        out[2 * j + 1] = (uint8_t)(w >> 12);  /* Q bits 12..19 */
    }
    return (size_t)samples * 2u;
}
