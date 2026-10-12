#include "db_code.h"

#include <math.h>

#define DB_PER_OCTAVE_Q4 (16.0f * 6.0206f)

int16_t db_exponent_q4[256];
int16_t db_mantissa_q4[128];

void db_code_init(void)
{
    /* +8 rounds the final >> 4 to nearest. */
    for (unsigned e = 0; e < 256; e++) {
        db_exponent_q4[e] = (int16_t)(lrintf(DB_PER_OCTAVE_Q4 * ((float)e - 127.0f)) + 8);
    }
    /* Mantissa bucket centre. */
    for (unsigned m = 0; m < 128; m++) {
        db_mantissa_q4[m] = (int16_t)lrintf(DB_PER_OCTAVE_Q4 * log2f(1.0f + ((float)m + 0.5f) / 128.0f));
    }
}
