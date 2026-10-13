/*
 * Real-time packet receiver for the link (IRAM, interrupts masked by the caller).
 *
 * The dump engine runs in circular mode at 16 Msps and ping-pongs banks 0 and 1
 * every LINK_RX_PERIOD pairs; the CPU cannot read the bank being written, so it
 * processes the one just released. Marker words around each switch point give
 * the exact unit edges. Per 4 raw samples (signal near fs/4) samples 0 and 2
 * make one 4 Msps complex sample; the sign of the cross product with the
 * previous one is the FM bit. One 32-bit shift register per sampling phase is
 * matched against the sync word; the matching phase then collects the body.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "link_frame.h"

#define LINK_RX_MAX_PHASES 16u

typedef struct {
    /* plan */
    uint32_t sync;
    uint16_t body_bits;
    unsigned phases;        /* decimated samples per bit: 16 / 4 / 2 */
    /* detector */
    int32_t pi, pq;
    int32_t acc;            /* integrated discriminator: acc += d - acc >> k */
    unsigned k;
    uint32_t reg[LINK_RX_MAX_PHASES];
    unsigned phase;
    uint32_t pending_x0;
    bool have_x0;
    int collect;            /* phase collecting the body, -1 while searching */
    bool skip;              /* drop the collect phase's first sample (still the last sync bit) */
    uint16_t got;
    uint8_t body[LINK_MAX_BODY_BYTES];
    /* engine */
    uint32_t saved_select, T, start_win;
    unsigned cur;
    bool have_unit;
    unsigned job_bank;
    uint32_t job_idx, job_left;
    /* result */
    uint32_t end_cycles;    /* CPU cycle count when the last body bit was sampled */
    uint32_t bad_edges;
    uint32_t syncs;
} link_rx_t;

void link_rx_init(link_rx_t *e, const link_rx_plan_t *plan, uint32_t bitrate);
/* Start the engine (receive path must be prepared). */
void link_rx_start(link_rx_t *e);
/* Run until a complete frame is in e->body (true) or the cycle counter passes `deadline` (false).
 * After a frame, call again to keep searching (e.g. when its CRC failed). */
bool link_rx_run(link_rx_t *e, uint32_t deadline);
void link_rx_stop(link_rx_t *e);
