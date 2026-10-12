#pragma once

#include <stdint.h>

/*
 * Baseband filter code for an approximate two-sided bandwidth in MHz.
 * Curve measured on the S3: noise spectrum at 2300 MHz, gain 75, 80 Msps,
 * 12 snapshots per code, median windowed FFTs, approximate -3 dB full width.
 * 0 MHz returns code 0 (widest). Values outside the curve clamp to its ends.
 */
uint8_t bandwidth_filter_code(uint32_t mhz);
