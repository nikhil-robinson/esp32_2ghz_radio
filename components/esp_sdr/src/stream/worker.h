/*
 * Second core (APP CPU) as a stream processor.
 *
 * ESP-IDF runs single-core (CONFIG_FREERTOS_UNICORE) and leaves core 1
 * stalled; worker_start() boots it into worker_main() with interrupts masked.
 * Core 0 keeps the ring (polling, sentinels, bank switches) and the sink;
 * core 1 processes units. They share internal SRAM (no data cache), so
 * plain volatile words plus memw barriers order every handshake:
 *
 *  - core 0 posts each unit with a sequence number and marks its bank owned
 *    (bank_seq[bank] = seq); core 1 clears that when it is done with the bank;
 *  - before refilling a bank, core 0 revokes it (bank_seq = 0) and waits
 *    while core 1 reads from it (busy == bank + 1);
 *  - core 1 sets busy, then re-checks bank_seq before every read (Dekker),
 *    so it never reads a bank that is being refilled.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stream.h"

#define WORKER_QUEUE_DEPTH 8u

typedef struct {
    volatile uint32_t alive;
    volatile uint32_t run;        /* incremented by core 0 to start a run */
    volatile uint32_t posted;     /* units posted by core 0 */
    volatile uint32_t taken;      /* units finished by core 1 */
    volatile uint32_t end;        /* core 0: no more units */
    volatile uint32_t done;       /* core 1: run finished */
    volatile uint32_t busy;       /* bank + 1 while core 1 reads it */
    volatile uint32_t block_max;  /* longest unit step, cycles */
    volatile uint32_t bank_seq[ESP_SDR_RING_BANKS];
} worker_mailbox_t;
extern worker_mailbox_t g_worker;

/* Processor hooks for one run, all IRAM: one unit; one idle step; final flush. */
typedef struct {
    void (*unit)(const stream_unit_t *unit);
    void (*idle)(void);
    void (*finish)(void);
} worker_hooks_t;

bool worker_start(void);
bool worker_alive(void);

/* ---- core 0 ------------------------------------------------------------- */
void worker_begin_run(const worker_hooks_t *hooks);
/* False when core 1 is WORKER_QUEUE_DEPTH units behind (unit skipped). */
bool worker_post(const stream_unit_t *unit);
void worker_revoke_bank(unsigned bank);
bool worker_bank_idle(unsigned bank);
/* Signal the end and wait for core 1, pumping the sink meanwhile. */
void worker_end_run(void);

/* ---- core 1 ------------------------------------------------------------- */
static inline bool worker_lock_bank(uint32_t bank, uint32_t seq)
{
    g_worker.busy = bank + 1u;
    MEMORY_BARRIER();
    if (g_worker.bank_seq[bank] != seq) {  /* core 0 needs the bank back */
        MEMORY_BARRIER();
        g_worker.busy = 0;
        return false;
    }
    return true;
}

static inline void worker_unlock_bank(void)
{
    MEMORY_BARRIER();
    g_worker.busy = 0;
}

static inline void worker_release_bank(uint32_t bank, uint32_t seq)
{
    if (g_worker.bank_seq[bank] == seq) {
        g_worker.bank_seq[bank] = 0;
    }
    MEMORY_BARRIER();
}
