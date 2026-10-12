/*
 * Power to 0.5 dB codes without float math in the hot path:
 * code = 2 * 10*log10(p) = 6.0206 * log2(p), taken from the IEEE-754 bits of p
 * via an exponent table and a 7-bit mantissa table, both in 1/16 code units.
 * Maximum error 0.09 code (0.05 dB).
 */
#pragma once

#include <stdint.h>

#define DB_CODE_STEP 2u  /* codes per dB */

extern int16_t db_exponent_q4[256];
extern int16_t db_mantissa_q4[128];

void db_code_init(void);

/* Code of the float whose bits are `bits`, minus `offset_q4`, clamped to 0..255. */
static inline uint8_t db_code(uint32_t bits, int32_t offset_q4)
{
    int32_t c = ((int32_t)db_exponent_q4[(bits >> 23) & 255u] + db_mantissa_q4[(bits >> 16) & 127u] - offset_q4) >> 4;
    return c <= 0 ? 0 : c >= 255 ? 255 : (uint8_t)c;
}

/* Offset that turns a sum of n powers into their mean (n >= 2), in 1/16 codes. */
static inline int32_t db_mean_offset_q4(uint32_t n)
{
    union { float f; uint32_t u; } v = {.f = (float)n};
    return (int32_t)db_exponent_q4[(v.u >> 23) & 255u] + db_mantissa_q4[(v.u >> 16) & 127u] - 8;
}
