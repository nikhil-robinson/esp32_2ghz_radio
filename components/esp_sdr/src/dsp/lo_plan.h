/*
 * Receive LO plan. Below 2210 MHz the PLL cannot lock directly; the receiver
 * then runs the PLL at 6/5 of the wanted LO and divides with the alternate
 * LO path (verified 1842..2209 MHz). Pure functions, host-testable.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define LO_ALTERNATE_MIN_MHZ 1842u
#define LO_ALTERNATE_MAX_MHZ 2209u

typedef struct {
    uint32_t pll_mhz;   /* integer MHz the PLL is asked for */
    int32_t offset_khz; /* fractional part, kHz */
    bool alternate;     /* select the 6/5 divider path */
} lo_plan_t;

static inline lo_plan_t lo_plan(uint32_t mhz)
{
    bool alternate = mhz >= LO_ALTERNATE_MIN_MHZ && mhz <= LO_ALTERNATE_MAX_MHZ;
    uint32_t khz = mhz * (alternate ? 1200u : 1000u);
    return (lo_plan_t){.pll_mhz = khz / 1000u, .offset_khz = (int32_t)(khz % 1000u), .alternate = alternate};
}

/* Wi-Fi channel centres can use the PHY's own channel table (better spurs). */
static inline bool lo_is_wifi_channel(uint32_t mhz)
{
    return (mhz >= 2412u && mhz <= 2472u && (mhz - 2412u) % 5u == 0u) || mhz == 2484u;
}
