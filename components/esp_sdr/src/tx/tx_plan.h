/* Transmit planning: region limits and deviation steps (pure, host-tested). */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_sdr_tx.h"

/* The 6/5 LO divider is receive-only, so TX starts where the direct PLL path does. */
#define TX_MIN_KHZ 2210000u
#define TX_MAX_KHZ 2790000u

/* True if the whole occupied band lo_khz..hi_khz lies inside the region. */
static inline bool tx_region_allows(esp_sdr_region_t region, uint32_t lo_khz, uint32_t hi_khz)
{
    static const uint32_t limits[][2] = {
        [ESP_SDR_REGION_ISM] = {2400000u, 2483500u},
        [ESP_SDR_REGION_HAM] = {2300000u, 2450000u},
        [ESP_SDR_REGION_UNRESTRICTED] = {TX_MIN_KHZ, TX_MAX_KHZ},
    };
    if ((unsigned)region > ESP_SDR_REGION_UNRESTRICTED || lo_khz > hi_khz) {
        return false;
    }
    return lo_khz >= limits[region][0] && hi_khz <= limits[region][1];
}

/* Deviation in tone-field steps: nearest whole step, at least one. */
static inline int tx_deviation_steps(uint32_t deviation_hz)
{
    uint32_t steps = (deviation_hz + ESP_SDR_TX_STEP_HZ / 2u) / ESP_SDR_TX_STEP_HZ;
    return steps ? (int)steps : 1;
}

static inline bool tx_bitrate_supported(uint32_t bitrate)
{
    return bitrate == 250000u || bitrate == 1000000u || bitrate == 2000000u;
}

/* Sub-steps per bit for Gaussian shaping. 4 at 2 Mbps is 30 CPU cycles per
 * write; the loop's measured lateness is 4 cycles. */
static inline unsigned tx_substeps(uint32_t bitrate, bool gaussian)
{
    (void)bitrate;
    return gaussian ? 4u : 1u;
}
