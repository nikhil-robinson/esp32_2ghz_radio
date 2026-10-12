#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * In-place packing of raw dump words (I bits 0..9, Q bits 10..19).
 * IQ10: two 20-bit samples per 5 bytes, little-endian bit order; an odd final
 *       sample takes 3 bytes. IQ8: signed I byte then Q byte, the upper eight
 *       bits of each 10-bit component (arithmetic truncation).
 * Both return the packed byte count. Output never overtakes the input.
 */
size_t iq_pack10(uint32_t *words, uint32_t samples);
size_t iq_pack8(uint32_t *words, uint32_t samples);
size_t iq_pack10_size(uint32_t samples);
