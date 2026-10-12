/*
 * Continuous capture by bank rotation.
 *
 * In circular mode the dump engine writes pairs at a ring index that advances
 * continuously (mod 16384) into whichever bank is selected. Switching the
 * selected bank while it runs splits the stream into gapless "units", one per
 * bank visit (scheme after h0m3us3r/eSpDR capture.c, 0BSD):
 *
 *  - while the engine fills bank b, bank b+1 gets sentinel words where the
 *    next unit is predicted to start and to end;
 *  - once THRESHOLD pairs are in bank b, the engine is moved to bank b+1;
 *  - the first and last pair of the finished unit are found by binary search
 *    over the sentinel windows, and each unit must start exactly where the
 *    previous one ended - which proves the stream gapless.
 *
 * Interrupts stay masked on core 0 for the whole run (no tick, no Wi-Fi ISR,
 * no driver; CONFIG_ESP_INT_WDT must be off). Between polls core 0 runs short
 * work slices or feeds core 1, and pumps the output queue to the sink.
 */
#include <string.h>
#include "burst.h"
#include "dump_regs.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "iq_stream.h"
#include "sdkconfig.h"
#include "soc/soc.h"
#include "spectrum.h"
#include "stream.h"
#include "worker.h"

#define SENTINEL 0xA5C33C5Au
#define NOT_FOUND 0xffffffffu
#define THRESHOLD DUMP_RING_THRESHOLD
#define START_GUARD 3072u       /* sentinels from a unit's earliest possible start */
#define END_GUARD 4096u         /* sentinels from a unit's earliest possible end */
#define LATE_LIMIT 2000u        /* allowed switch lateness: 2 * LATE_LIMIT < END_GUARD */
#define FIRST_UNIT_START_BACK 1024u
#define FIRST_UNIT_START_GUARD 3072u
#define FIRST_UNIT_END_GUARD 1024u  /* unit 0: whole bank is sentinel; stay clear of its start */
#define MIN_PAIRS THRESHOLD
#define MAX_PAIRS (THRESHOLD + LATE_LIMIT + 64u)
#define SETTLE_PAIRS 16u        /* engine settling after a bank switch */
#define PREPARE_CYCLES 20000u   /* worst-case bank preparation incl. a forced retire */
#define PREPARE_MARGIN_PAIRS 1024u
#define PUMP_HEADROOM_PAIRS 2048u
#define SLICE_MIN_CYCLES 8000u
#define SLICE_MARGIN_PAIRS 1024u
#define ASSIST_MARGIN_PAIRS 256u
#define SINK_SILENCE_US 2000000 /* open-ended spectrum: stop if the sink takes nothing for this long */
#define DRAIN_TIMEOUT_US 500000

_Static_assert(START_GUARD + THRESHOLD <= DUMP_RING_PAIRS, "start window overlaps");
_Static_assert(END_GUARD + THRESHOLD <= DUMP_RING_PAIRS, "end window overlaps unit start");
_Static_assert(2u * LATE_LIMIT + 64u < END_GUARD && LATE_LIMIT + 64u < START_GUARD, "guards");

stream_run_t g_stream;
stream_load_t g_load;
static esp_sdr_workers_t workers = ESP_SDR_WORKERS_DUAL_ASSIST;

void esp_sdr_set_workers(esp_sdr_workers_t w) { workers = w; }
esp_sdr_workers_t esp_sdr_get_workers(void) { return workers; }
bool esp_sdr_core1_alive(void) { return worker_alive(); }
uint32_t esp_sdr_assist_blocks(void) { return spectrum_assist_blocks(); }
void esp_sdr_set_dc_mode(uint8_t mode) { spectrum_set_dc_mode(mode); }

/* ---- sentinel windows --------------------------------------------------- */

/* Fill ring indices [at, at + count) of a bank with sentinels, wrapping.
 * 128-bit PIE stores (eSpDR capture.c): a scalar loop is too slow to prepare
 * ~7 k words between two switches at 80 Msps. */
STREAM_HOT static void fill_sentinels(unsigned bank, unsigned at, unsigned count)
{
    at &= DUMP_RING_MASK;
    while (count) {
        unsigned n = DUMP_RING_PAIRS - at;
        if (n > count) {
            n = count;
        }
        uint32_t *p = dump_bank(bank) + at;
        unsigned head = (-at) & 3u;
        if (head > n) {
            head = n;
        }
        for (unsigned i = 0; i < head; i++) {
            *p++ = SENTINEL;
        }
        unsigned vectors = (n - head) / 4;
        if (vectors) {
            uint32_t value = SENTINEL;
            __asm__ volatile("ee.movi.32.q q6, %[v], 0\n"
                             "ee.movi.32.q q6, %[v], 1\n"
                             "ee.movi.32.q q6, %[v], 2\n"
                             "ee.movi.32.q q6, %[v], 3\n"
                             "loopnez %[n], 1f\n"
                             "ee.vst.128.ip q6, %[p], 16\n"
                             "1:"
                             : [p] "+a"(p)
                             : [v] "a"(value), [n] "a"(vectors)
                             : "memory");
        }
        for (unsigned i = head + vectors * 4; i < n; i++) {
            *p++ = SENTINEL;
        }
        count -= n;
        at = 0;
    }
    MEMORY_BARRIER();
}

/* First written pair in [origin, origin + guard), which holds sentinels up to
 * the unit start and data from there on. */
STREAM_HOT static uint32_t find_first(unsigned bank, unsigned origin, unsigned guard)
{
    const uint32_t *p = dump_bank(bank);
    if (p[origin & DUMP_RING_MASK] != SENTINEL) {
        return NOT_FOUND;  /* started earlier than predicted */
    }
    if (p[(origin + guard - 1) & DUMP_RING_MASK] == SENTINEL) {
        return NOT_FOUND;
    }
    unsigned lo = 0, hi = guard - 1;
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (p[(origin + mid) & DUMP_RING_MASK] == SENTINEL) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return (origin + lo) & DUMP_RING_MASK;
}

/* One past the last written pair in [origin, origin + guard), searching from
 * the write index seen before the switch. */
STREAM_HOT static uint32_t find_end(unsigned bank, unsigned origin, unsigned write_index, unsigned guard)
{
    const uint32_t *p = dump_bank(bank);
    unsigned lo = (write_index - origin) & DUMP_RING_MASK;
    if (lo >= guard || p[(origin + guard - 1) & DUMP_RING_MASK] != SENTINEL) {
        return NOT_FOUND;
    }
    unsigned hi = guard - 1;
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (p[(origin + mid) & DUMP_RING_MASK] == SENTINEL) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    return (origin + lo) & DUMP_RING_MASK;
}

STREAM_HOT static void select_banks(uint32_t saved, unsigned mask)
{
    REG_WRITE(DUMP_BANK_SELECT_REG, (saved & ~DUMP_BANK_MASK) | mask);
}

STREAM_HOT static void fail(esp_sdr_run_result_t *r, esp_sdr_run_status_t status, uint32_t detail)
{
    if (!r->status) {
        r->status = status;
        r->detail = detail;
    }
}

/* ---- run ---------------------------------------------------------------- */

typedef struct {
    bool spectrum, capture, iq, dual, assist;
} run_kind_t;

STREAM_HOT static void reclaim_bank(const run_kind_t *k, unsigned bank)
{
    if (k->dual) {
        worker_revoke_bank(bank);
    } else if (k->spectrum) {
        spectrum_release_bank(bank);
    } else if (k->iq) {
        iq_stream_release_bank(bank);
    }
}

STREAM_HOT static bool bank_idle(const run_kind_t *k, unsigned bank)
{
    if (k->dual) {
        return worker_bank_idle(bank);
    }
    return k->spectrum ? spectrum_bank_idle(bank) : true;
}

STREAM_HOT static void prepare_bank(unsigned bank, uint32_t expected, uint32_t *start_probe, uint32_t *end_probe)
{
    start_probe[bank] = (expected + THRESHOLD) & DUMP_RING_MASK;
    end_probe[bank] = (start_probe[bank] + THRESHOLD) & DUMP_RING_MASK;
    fill_sentinels(bank, start_probe[bank], START_GUARD);
    fill_sentinels(bank, end_probe[bank], END_GUARD);
}

static bool validate(const esp_sdr_run_config_t *c, const esp_sdr_sink_t *sink, const run_kind_t *k,
                     esp_sdr_run_result_t *r)
{
    if (c->rate > ESP_SDR_RATE_16M) {
        fail(r, ESP_SDR_RUN_FAIL_ARG, 0);
        return false;
    }
    if (k->capture && (c->capture_units < 1 || c->capture_units > ESP_SDR_RING_BANKS)) {
        fail(r, ESP_SDR_RUN_FAIL_ARG, 0);
        return false;
    }
    if (k->spectrum && (!spectrum_init() || !spectrum_valid_nfft(c->nfft))) {
        fail(r, ESP_SDR_RUN_FAIL_ARG, 0);
        return false;
    }
    return true;
}

STREAM_HOT void stream_run(const esp_sdr_run_config_t *cfg, const esp_sdr_sink_t *sink, esp_sdr_run_result_t *r)
{
    memset(r, 0, sizeof(*r));
    g_stream.config = cfg;
    g_stream.sink = sink;
    g_stream.result = r;
    g_load.core0 = g_load.core1 = 0;
    tx_queue_reset();

    run_kind_t k = {
        .spectrum = cfg->mode == ESP_SDR_RUN_SPECTRUM,
        .capture = cfg->mode == ESP_SDR_RUN_CAPTURE,
        .iq = cfg->mode == ESP_SDR_RUN_IQ,
    };
    if (!validate(cfg, sink, &k, r)) {
        return;
    }
    if (k.iq) {
        uint32_t detail;
        if (!iq_stream_begin(&detail)) {
            fail(r, ESP_SDR_RUN_FAIL_ARG, detail);
            return;
        }
    }
    k.dual = (k.spectrum || k.iq) && worker_alive() && workers != ESP_SDR_WORKERS_SINGLE;
    k.assist = k.dual && workers == ESP_SDR_WORKERS_DUAL_ASSIST;

    const uint32_t cpp = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000u / esp_sdr_rate_hz(cfg->rate);  /* cycles per pair */
    const uint32_t max_age = (DUMP_RING_PAIRS - 128u) * cpp;
    const uint32_t settle = SETTLE_PAIRS * cpp;
    const uint32_t ctrl = DUMP_CTRL_CIRCULAR | burst_rate_bits(cfg->rate);
    const int64_t duration_us = k.capture ? 0 : (int64_t)cfg->duration_ms * 1000;
    const uint32_t bank_select_saved = REG_READ(DUMP_BANK_SELECT_REG);
    uint32_t start_probe[ESP_SDR_RING_BANKS] = {0}, end_probe[ESP_SDR_RING_BANKS] = {0};

    for (unsigned b = 0; b < ESP_SDR_RING_BANKS; b++) {
        fill_sentinels(b, 0, DUMP_RING_PAIRS);
    }
    /* Warm up after the sentinel fill: spectrum_begin() reads bank 2. */
    uint32_t assist_cost = k.spectrum ? spectrum_begin(k.dual, k.assist) : 0;
    uint32_t assist_budget = assist_cost + assist_cost / 3u;  /* grows to the maximum seen */
    if (k.dual) {
        worker_begin_run(k.spectrum ? &spectrum_worker_hooks : &iq_stream_worker_hooks);
    }
    uint32_t worker_seq = 0, worker_skipped_blocks = 0, tail_seen = 0;
    int64_t last_output = 0;

    unsigned irq = portSET_INTERRUPT_MASK_FROM_ISR();
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    select_banks(bank_select_saved, 1u);
    const int64_t t_start = esp_timer_get_time();
    last_output = t_start;
    REG_WRITE(DUMP_CTRL_REG, ctrl | DUMP_CTRL_RUN);
    uint32_t epoch = esp_cpu_get_cycle_count();
    const uint32_t w0 = REG_READ(DUMP_WRITE_INDEX_REG) & DUMP_RING_MASK;

    /* Unit 0 starts near w0; its exact start is found after the first switch. */
    uint32_t expected = (w0 - 256u) & DUMP_RING_MASK;
    uint64_t index = 0;
    unsigned bank = 0;
    bool prepared = false;

    for (unsigned unit = 0;; unit++) {
        const unsigned next = (bank + 1) % ESP_SDR_RING_BANKS;
        const bool need_next = !k.capture || unit + 1 < cfg->capture_units;

        /* 1. Poll; between polls prepare the next bank, pump output and run
         *    bounded work. The stop decision is made mid-unit so only the
         *    index read sits between the threshold and the bank switch. */
        uint32_t write_index, written;
        bool stop = !need_next, stop_checked = false;
        for (;;) {
            /* Single core: a forced retire may run the longest slice seen (grows during the run). */
            const uint32_t prepare_pairs =
                (!k.dual && r->work_max > PREPARE_CYCLES ? r->work_max : PREPARE_CYCLES) / cpp + PREPARE_MARGIN_PAIRS;
            write_index = REG_READ(DUMP_WRITE_INDEX_REG) & DUMP_RING_MASK;
            written = (write_index - expected) & DUMP_RING_MASK;
            uint32_t age = esp_cpu_get_cycle_count() - epoch;
            if (age > max_age) {
                fail(r, ESP_SDR_RUN_FAIL_AGE, age);
                break;
            }
            if (written >= THRESHOLD) {
                if (need_next && !prepared) {  /* a long slice ran past the deadline */
                    uint32_t tp = esp_cpu_get_cycle_count();
                    reclaim_bank(&k, next);
                    prepare_bank(next, expected, start_probe, end_probe);
                    prepared = true;
                    g_load.core0 += esp_cpu_get_cycle_count() - tp;
                    write_index = REG_READ(DUMP_WRITE_INDEX_REG) & DUMP_RING_MASK;
                    written = (write_index - expected) & DUMP_RING_MASK;
                }
                break;
            }
            if (!stop_checked && written >= THRESHOLD / 2) {
                stop_checked = true;
                if (!stop && duration_us && esp_timer_get_time() - t_start >= duration_us) {
                    stop = true;
                }
                if (tx_queue_empty() || tx_queue_consumed() != tail_seen) {  /* the host keeps up */
                    tail_seen = tx_queue_consumed();
                    last_output = esp_timer_get_time();
                }
                if (!stop && k.spectrum && !duration_us && esp_timer_get_time() - last_output > SINK_SILENCE_US) {
                    stop = true;
                    r->stopped = true;
                }
                if (!stop && !k.capture && sink && sink->stop_requested && sink->stop_requested(sink->ctx)) {
                    stop = true;
                    r->stopped = true;
                }
                continue;
            }
            /* Prepare the next bank as soon as its old unit is retired, and
             * unconditionally with one late-limit of margin left. */
            if (need_next && !prepared &&
                (bank_idle(&k, next) || written + prepare_pairs + LATE_LIMIT >= THRESHOLD)) {
                uint32_t tp = esp_cpu_get_cycle_count();
                reclaim_bank(&k, next);
                prepare_bank(next, expected, start_probe, end_probe);
                prepared = true;
                g_load.core0 += esp_cpu_get_cycle_count() - tp;
                continue;
            }
            if (written + PUMP_HEADROOM_PAIRS < THRESHOLD && !tx_queue_empty()) {
                uint32_t tq = esp_cpu_get_cycle_count();
                tx_queue_pump();
                g_load.core0 += esp_cpu_get_cycle_count() - tq;
            }
            /* Output took time since the last poll: never schedule work
             * against that stale write position. */
            write_index = REG_READ(DUMP_WRITE_INDEX_REG) & DUMP_RING_MASK;
            written = (write_index - expected) & DUMP_RING_MASK;
            if (written >= THRESHOLD) {
                continue;
            }
            /* Start work only if the longest step seen so far still fits. */
            const uint32_t work_limit = prepared || !need_next ? THRESHOLD + LATE_LIMIT / 2
                                                               : THRESHOLD - prepare_pairs - LATE_LIMIT;
            if (k.assist && k.spectrum) {
                if (written + assist_budget / cpp + ASSIST_MARGIN_PAIRS < work_limit) {
                    uint32_t t0 = esp_cpu_get_cycle_count();
                    spectrum_assist();
                    uint32_t dt = esp_cpu_get_cycle_count() - t0;
                    if (dt > assist_budget) {
                        assist_budget = dt;
                    }
                    g_load.core0 += dt;
                }
            }
            if (k.iq && !k.dual && iq_stream_pending()) {
                if (written + iq_stream_slice_cost() / cpp + ASSIST_MARGIN_PAIRS < work_limit) {
                    g_load.core0 += iq_stream_slice();
                }
            }
            if (k.spectrum && !k.dual) {
                uint32_t slice = r->work_max > SLICE_MIN_CYCLES ? r->work_max : SLICE_MIN_CYCLES;
                if (written + slice / cpp + 128u + SLICE_MARGIN_PAIRS < THRESHOLD + LATE_LIMIT / 2) {
                    uint32_t tw = esp_cpu_get_cycle_count();
                    if (spectrum_work_slice()) {
                        g_load.core0 += esp_cpu_get_cycle_count() - tw;
                    }
                }
            }
        }
        if (r->status) {
            break;
        }

        uint32_t late = written - THRESHOLD;
        if (late > r->late_max) {
            r->late_max = late;
        }
        if (late > LATE_LIMIT) {
            fail(r, ESP_SDR_RUN_FAIL_LATE, written);
            break;
        }

        /* 2. Stop or switch. */
        if (stop) {
            REG_WRITE(DUMP_CTRL_REG, ctrl);
            select_banks(bank_select_saved, 0u);
        } else {
            select_banks(bank_select_saved, 1u << next);
            epoch = esp_cpu_get_cycle_count();
        }
        uint32_t t = esp_cpu_get_cycle_count();
        while (esp_cpu_get_cycle_count() - t < settle) {
        }

        /* 3. Locate the finished unit exactly. */
        uint32_t first, end;
        if (unit == 0) {
            first = find_first(bank, (w0 - FIRST_UNIT_START_BACK) & DUMP_RING_MASK, FIRST_UNIT_START_GUARD);
            end = find_end(bank, write_index, write_index, FIRST_UNIT_END_GUARD);
        } else {
            first = find_first(bank, start_probe[bank], START_GUARD);
            end = find_end(bank, end_probe[bank], write_index, END_GUARD);
        }
        if (first == NOT_FOUND || (unit && first != expected)) {
            fail(r, ESP_SDR_RUN_FAIL_START, (first & 0xffffu) | (expected << 16));
            break;
        }
        if (end == NOT_FOUND) {
            fail(r, ESP_SDR_RUN_FAIL_END, write_index);
            break;
        }
        uint32_t count = (end - first) & DUMP_RING_MASK;
        if (count < (unit ? MIN_PAIRS : MIN_PAIRS - 1024u) || count > MAX_PAIRS) {
            fail(r, ESP_SDR_RUN_FAIL_LENGTH, count);
            break;
        }

        const stream_unit_t located = {
            .seq = ++worker_seq, .bank = bank, .first = first, .count = count, .index = index,
            .gain = (uint8_t)(dump_bank(bank)[first] >> 20),
        };
        if (k.capture) {
            r->unit[unit] = (esp_sdr_ring_unit_t){.bank = (uint16_t)bank, .first = (uint16_t)first, .count = count};
        } else if (k.dual) {
            if (!worker_post(&located) && k.spectrum) {
                worker_skipped_blocks += count >> __builtin_ctz(cfg->nfft);  /* core 1 far behind */
            }
        } else if (k.spectrum) {
            spectrum_accept_unit(&located);
        } else if (k.iq) {
            iq_stream_accept_unit(&located);
        }
        r->units++;
        index += count;
        expected = end;
        bank = next;
        prepared = false;
        if (stop) {
            break;
        }
    }

    /* Engine stopped (normally or by failure). Finish queued work. */
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_BANK_SELECT_REG, bank_select_saved);
    r->elapsed_us = (uint64_t)(esp_timer_get_time() - t_start);
    r->pairs = index;
    if (k.dual) {
        worker_end_run();
        r->abandoned += worker_skipped_blocks;
        r->work_max = g_worker.block_max;  /* longest core-1 block */
    }
    if (k.iq) {
        iq_stream_finish(k.dual);
    } else if (k.spectrum && !k.dual) {
        spectrum_finish_single();
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);

    /* Drain with a deadline; the host may have stopped reading. */
    int64_t deadline = esp_timer_get_time() + DRAIN_TIMEOUT_US;
    while (!tx_queue_empty() && esp_timer_get_time() < deadline) {
        tx_queue_pump();
    }
}

void esp_sdr_run_unit(const esp_sdr_run_result_t *result, unsigned u, const uint32_t **head, uint32_t *head_count,
                      const uint32_t **tail, uint32_t *tail_count)
{
    const esp_sdr_ring_unit_t *unit = &result->unit[u];
    const uint32_t *bank = dump_bank(unit->bank);
    uint32_t first_part = DUMP_RING_PAIRS - unit->first;
    if (first_part > unit->count) {
        first_part = unit->count;
    }
    *head = bank + unit->first;
    *head_count = first_part;
    *tail = bank;
    *tail_count = unit->count - first_part;
}
