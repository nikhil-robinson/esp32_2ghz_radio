#include "link_rx.h"

#include <string.h>
#include "dump_regs.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "soc/soc.h"

#define LINK_RX_PERIOD 512u     /* pairs per bank visit: 32 us at 16 Msps (worst latency ~2 visits) */
#define MARK 0xA5C33C5Au
#define WIN 64u                 /* marker window around each switch point */
#define START_AHEAD 256u
#define CYCLES_PER_PAIR 15u     /* 240 MHz / 16 Msps */
#define DECIMATED_PER_S 4000000u

void link_rx_init(link_rx_t *e, const link_rx_plan_t *plan, uint32_t bitrate)
{
    memset(e, 0, sizeof(*e));
    e->sync = plan->sync;
    e->body_bits = plan->body_bits;
    e->phases = DECIMATED_PER_S / bitrate;
    e->k = bitrate >= 2000000u ? 0u : 1u;  /* one-pole integration: offline comparison on captures */
    e->collect = -1;
}

static IRAM_ATTR void fill(unsigned bank, uint32_t at)
{
    uint32_t *p = dump_bank(bank);
    for (uint32_t j = 0; j < WIN; j++) {
        p[(at + j) & DUMP_RING_MASK] = MARK;
    }
}

/* First index in [at, at + WIN) that is (want_mark) a marker / (else) written data. */
static IRAM_ATTR uint32_t scan(link_rx_t *e, unsigned bank, uint32_t at, bool want_mark)
{
    const uint32_t *p = dump_bank(bank);
    for (uint32_t j = 0; j < WIN; j++) {
        if ((p[(at + j) & DUMP_RING_MASK] == MARK) == want_mark) {
            return (at + j) & DUMP_RING_MASK;
        }
    }
    e->bad_edges++;
    return (at + WIN) & DUMP_RING_MASK;
}

IRAM_ATTR void link_rx_start(link_rx_t *e)
{
    e->saved_select = REG_READ(DUMP_BANK_SELECT_REG);
    const uint32_t ctrl = DUMP_CTRL_CIRCULAR | DUMP_CTRL_RATE_16M;
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    REG_WRITE(DUMP_BANK_SELECT_REG, (e->saved_select & ~DUMP_BANK_MASK) | 2u);  /* warm-up visit: bank 1 */
    REG_WRITE(DUMP_CTRL_REG, ctrl | DUMP_CTRL_RUN);
    e->T = (REG_READ(DUMP_WRITE_INDEX_REG) + START_AHEAD) & DUMP_RING_MASK;
    fill(0, e->T);
    fill(0, e->T + LINK_RX_PERIOD);
    e->cur = 1;
    e->have_unit = false;
    e->job_left = 0;
}

IRAM_ATTR void link_rx_stop(link_rx_t *e)
{
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_BANK_SELECT_REG, e->saved_select);
}

/* One decimated sample from raw samples 0 and 2 of a group: integrated FM bit. */
#define DEMOD(x0, x2)                                                                      \
    do {                                                                                   \
        int32_t i_ = ((int32_t)((x0) << 22) >> 22) - ((int32_t)((x2) << 22) >> 22);       \
        int32_t q_ = ((int32_t)((x0) << 12) >> 22) - ((int32_t)((x2) << 12) >> 22);       \
        acc += (i_ * pq - q_ * pi) - (acc >> shift);                                       \
        pi = i_;                                                                           \
        pq = q_;                                                                           \
        bit = (uint32_t)(acc > 0);                                                         \
    } while (0)

/*
 * Process the current job; true (job paused) when a frame completed.
 * Searching and collecting are separate loops and all state is in locals:
 * the search loop must stay well under 15 CPU cycles per raw pair (measured).
 */
static IRAM_ATTR bool process(link_rx_t *e)
{
    const uint32_t *p = dump_bank(e->job_bank);
    uint32_t idx = e->job_idx, n = e->job_left;
    int32_t pi = e->pi, pq = e->pq, acc = e->acc;
    const unsigned shift = e->k, phases = e->phases;
    const uint32_t sync = e->sync;
    const uint16_t body_bits = e->body_bits;
    unsigned phase = e->phase;
    int collect = e->collect;
    bool skip = e->skip;
    uint16_t got = e->got;
    uint32_t *reg = e->reg, bit;
    bool ready = false;
    uint32_t ready_idx = 0;

    while (n && !ready) {
        if ((idx & 3) == 0 && n >= 4) {
            uint32_t groups = n >> 2, room = (DUMP_RING_PAIRS - idx) >> 2;
            if (groups > room) {
                groups = room;
            }
            const uint32_t *g = p + idx;
            uint32_t gi = 0;
            if (collect < 0) {
                for (; gi < groups; gi++, g += 4) {
                    DEMOD(g[0], g[2]);
                    uint32_t v = (reg[phase] << 1) | bit;
                    reg[phase] = v;
                    if (v == sync) {
                        /* the matching phases span about half a bit: collect a quarter bit later */
                        unsigned c = phase + phases / 4u;
                        collect = (int)(c >= phases ? c - phases : c);
                        skip = c != phase;  /* its next sample still falls in the last sync bit */
                        got = 0;
                        e->syncs++;
                        if (++phase == phases) {
                            phase = 0;
                        }
                        gi++;
                        g += 4;
                        break;
                    }
                    if (++phase == phases) {
                        phase = 0;
                    }
                }
            } else {
                for (; gi < groups; gi++, g += 4) {
                    DEMOD(g[0], g[2]);
                    if ((int)phase == collect && !(skip && !(skip = false))) {
                        uint8_t *byte = &e->body[got >> 3];
                        *byte = (uint8_t)((*byte << 1) | bit);
                        if (++got == body_bits) {
                            ready = true;
                            ready_idx = idx + 4 * gi;
                        }
                    }
                    if (++phase == phases) {
                        phase = 0;
                    }
                    if (ready) {
                        gi++;
                        break;
                    }
                }
            }
            idx = (idx + 4 * gi) & DUMP_RING_MASK;
            n -= 4 * gi;
            continue;
        }
        /* unaligned head of a unit: one raw pair at a time (rare) */
        unsigned ph = idx & 3;
        if (ph == 0) {
            e->pending_x0 = p[idx];
            e->have_x0 = true;
        } else if (ph == 2 && e->have_x0) {
            DEMOD(e->pending_x0, p[idx]);
            e->have_x0 = false;
            if (collect < 0) {
                uint32_t v = (reg[phase] << 1) | bit;
                reg[phase] = v;
                if (v == sync) {
                    unsigned c = phase + phases / 4u;
                    collect = (int)(c >= phases ? c - phases : c);
                    skip = c != phase;
                    got = 0;
                    e->syncs++;
                }
            } else if ((int)phase == collect && !(skip && !(skip = false))) {
                uint8_t *byte = &e->body[got >> 3];
                *byte = (uint8_t)((*byte << 1) | bit);
                if (++got == body_bits) {
                    ready = true;
                    ready_idx = idx;
                }
            }
            if (++phase == phases) {
                phase = 0;
            }
        }
        idx = (idx + 1) & DUMP_RING_MASK;
        n--;
    }
    e->pi = pi;
    e->pq = pq;
    e->acc = acc;
    e->phase = phase;
    e->got = got;
    e->skip = skip;
    e->job_idx = idx;
    e->job_left = n;
    if (ready) {
        /* Time of the frame's last sample: the engine writes one pair per CYCLES_PER_PAIR. */
        uint32_t now = esp_cpu_get_cycle_count();
        uint32_t w = REG_READ(DUMP_WRITE_INDEX_REG) & DUMP_RING_MASK;
        e->end_cycles = now - ((w - ready_idx) & DUMP_RING_MASK) * CYCLES_PER_PAIR;
        e->collect = -1;  /* a later call searches again */
        memset(e->reg, 0, sizeof(e->reg));
    } else {
        e->collect = collect;
    }
    return ready;
}
#undef DEMOD

IRAM_ATTR bool link_rx_run(link_rx_t *e, uint32_t deadline)
{
    for (;;) {
        if (e->job_left && process(e)) {
            return true;
        }
        uint32_t w;
        do {
            if ((int32_t)(esp_cpu_get_cycle_count() - deadline) > 0) {
                return false;
            }
            w = REG_READ(DUMP_WRITE_INDEX_REG) & DUMP_RING_MASK;
        } while (((w - e->T) & DUMP_RING_MASK) > DUMP_RING_PAIRS / 2);
        const unsigned next = e->cur ^ 1u;
        REG_WRITE(DUMP_BANK_SELECT_REG, (e->saved_select & ~DUMP_BANK_MASK) | (1u << next));
        if (e->have_unit) {  /* `cur` is free: its unit runs from its start window to its end window */
            uint32_t start = scan(e, e->cur, e->start_win, false);
            uint32_t end = scan(e, e->cur, e->T, true);
            e->job_bank = e->cur;
            e->job_idx = start;
            e->job_left = (end - start) & DUMP_RING_MASK;
        }
        fill(e->cur, e->T + LINK_RX_PERIOD);       /* `cur`'s next visit: start window ... */
        fill(e->cur, e->T + 2u * LINK_RX_PERIOD);  /* ... and end window (outside the job's range) */
        e->have_unit = true;
        e->start_win = e->T;
        e->cur = next;
        e->T = (e->T + LINK_RX_PERIOD) & DUMP_RING_MASK;
    }
}
