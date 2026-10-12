/*
 * Full-band scan geometry. Each step keeps one clean slice of the captured
 * band ABOVE the LO, [LO + offset, LO + offset + keep), clear of the DC / LO
 * leakage spike at 0 Hz and of the filter roll-off at the band edge:
 * offset = ceil(fs / 20), keep = floor(0.4 fs) (16 Msps: 1 + 6 MHz,
 * 40 Msps: 2 + 16 MHz, 80 Msps: 4 + 32 MHz). Pure, host-testable.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t offset_mhz;  /* slice start above the LO */
    uint32_t keep_mhz;    /* slice width = LO step */
    uint32_t steps;
    uint32_t first_lo_mhz;
} scan_plan_t;

static inline scan_plan_t scan_plan(uint32_t from_mhz, uint32_t to_mhz, uint32_t rate_hz)
{
    scan_plan_t p;
    p.offset_mhz = (rate_hz + 19999999u) / 20000000u;
    p.keep_mhz = (uint32_t)(((uint64_t)rate_hz * 2u) / 5000000u);
    p.first_lo_mhz = from_mhz - p.offset_mhz;
    p.steps = to_mhz > from_mhz ? (to_mhz - from_mhz + p.keep_mhz - 1u) / p.keep_mhz : 0;
    return p;
}

static inline uint32_t scan_lo_mhz(const scan_plan_t *p, uint32_t step)
{
    return p->first_lo_mhz + step * p->keep_mhz;
}

/* Every step's LO inside [min_lo, max_lo]? */
static inline bool scan_plan_valid(const scan_plan_t *p, uint32_t from_mhz, uint32_t min_lo, uint32_t max_lo)
{
    return p->steps > 0 && from_mhz >= min_lo + p->offset_mhz &&
           scan_lo_mhz(p, p->steps - 1u) <= max_lo;
}
