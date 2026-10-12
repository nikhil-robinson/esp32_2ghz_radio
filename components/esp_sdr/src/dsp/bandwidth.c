#include "bandwidth.h"

typedef struct {
    uint8_t code;
    uint8_t mhz;
} curve_point_t;

/* Ordered by falling bandwidth. */
static const curve_point_t CURVE[] = {
    {0, 69}, {4, 51}, {8, 45}, {16, 33}, {24, 25}, {32, 21}, {48, 16}, {60, 13},
};
#define CURVE_POINTS (sizeof(CURVE) / sizeof(CURVE[0]))

uint8_t bandwidth_filter_code(uint32_t mhz)
{
    if (mhz == 0 || mhz >= CURVE[0].mhz) {
        return CURVE[0].code;
    }
    if (mhz <= CURVE[CURVE_POINTS - 1].mhz) {
        return CURVE[CURVE_POINTS - 1].code;
    }
    for (unsigned i = 1; i < CURVE_POINTS; i++) {
        if (mhz >= CURVE[i].mhz) {
            const curve_point_t *hi = &CURVE[i - 1], *lo = &CURVE[i];
            uint32_t span = hi->mhz - lo->mhz;
            /* Linear interpolation of the code, rounded to nearest. */
            return (uint8_t)(hi->code + ((lo->code - hi->code) * (hi->mhz - mhz) + span / 2) / span);
        }
    }
    return CURVE[CURVE_POINTS - 1].code;
}
