#include "spectrum.h"

#include <math.h>
#include <string.h>
#include "crc32.h"
#include "db_code.h"
#include "dsps_fft2r.h"
#include "dump_regs.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_sdr_record.h"
#include "esp_timer.h"
#include "record.h"

#define ACCUMULATE_CHUNK 128u   /* bins per accumulation / emission slice */
#define UNPACK_CHUNK 512u       /* pairs per unpack slice */
#define WORKER_UNPACK_CHUNK 256u
#define WORKER_ENCODE_CHUNK 256u
#define WORKER_CRC_CHUNK 1024u
#define STATS_PERIOD_CYCLES 60000000u  /* 250 ms at 240 MHz */
#define FLAG_MAX_HOLD 1u
#define FLAG_ABANDONED 2u
#define FLAG_DROPPED_BEFORE 4u
#define PI_F 3.14159265358979f

/* esp-dsp twiddles; the transform itself is our bias-corrected kernel. */
extern int16_t *dsps_fft_w_table_sc16;
int fft_sc16_rounded(int16_t *data, int n, int16_t *twiddles);
int fft_sc16_rounded_stage(int16_t *data, int n, int16_t *twiddles, unsigned stage);
/* PIE unpack: IQ10 words -> {I<<6, Q<<6} * window >> 15, 8 pairs per group. */
void iq10_unpack_windowed(const uint32_t *src, int16_t *dst, const int16_t *window2, unsigned groups8);

/* ---- tables and buffers ------------------------------------------------- */

#define FRAME_PAYLOAD_HEADER sizeof(esp_sdr_record_spectrum_frame_t)
#define FRAME_BINS_AT (ESP_SDR_RECORD_HEADER_SIZE + FRAME_PAYLOAD_HEADER)

static int16_t window_q15[ESP_SDR_MAX_NFFT];
static int16_t window_pairs[2 * ESP_SDR_MAX_NFFT] __attribute__((aligned(16)));  /* each coefficient twice */
static int16_t fft_buffer[2 * ESP_SDR_MAX_NFFT] __attribute__((aligned(16)));
/* Indexed by FFT output slot (bit-reversed). Mean: float power sums;
 * max-hold: uint32 powers in the same storage. Double-buffered so core 1 can
 * encode one while filling the other. */
static float accumulators[2][ESP_SDR_MAX_NFFT];
static float *accumulator = accumulators[0];
static uint16_t natural_bin[ESP_SDR_MAX_NFFT];  /* FFT slot <-> natural bin (involution) */
static uint8_t frame_out[FRAME_BINS_AT + ESP_SDR_MAX_NFFT + ESP_SDR_RECORD_CRC_SIZE];
static int16_t *assist_buffer;  /* heap: BSS must end below the RF ring */
static unsigned nfft, nfft_log2;
static bool fft_ready;

static uint8_t dc_mode = 1;
static struct {
    int32_t re, im;
    bool ready;
} dc_estimate;

/* ---- run state ---------------------------------------------------------- */

typedef struct {
    bool pending;
    uint32_t first, count, next_block, blocks;
    uint64_t index;
    uint8_t gain;
} pending_unit_t;

enum { STEP_IDLE, STEP_UNPACK, STEP_FFT, STEP_ACCUMULATE };

static struct {
    pending_unit_t unit[ESP_SDR_RING_BANKS];
    unsigned fifo[ESP_SDR_RING_BANKS], fifo_len;  /* banks with pending work, oldest first */
    /* frame being accumulated */
    unsigned frame_units, frame_ffts;
    uint32_t frame_pairs;
    uint64_t frame_index;
    uint8_t frame_gain, frame_flags;
    bool dropped;
    /* block in flight (single core) */
    unsigned step, position, block_bank, fft_stage;
    uint32_t block_at;
    /* frame emission in progress (single core) */
    bool emitting;
    unsigned emit_position;
    uint32_t emit_crc;
    int32_t emit_offset_q4;
} run;

/* Worker-side frame encoder (source = accumulator swapped out). */
static struct {
    uint32_t pending, bin, crc, crc_at, length;
    int32_t offset_q4;
    bool max_hold;
    uint32_t *source;
} encoder;

/* Block cursor of the unit core 1 works on, and core 0's hand-off slot. */
static volatile struct {
    uint32_t seq;        /* unit open for claims, 0 = none */
    uint32_t bank, first, blocks;
    uint32_t next;       /* next block index (claimed with compare-and-set) */
    uint32_t assist_seq; /* core 0 is between claim and hand-off for this unit */
    uint32_t handoff_full;
    uint32_t handoff_seq;
    uint32_t done;       /* blocks accumulated for the open unit */
    uint32_t assist_blocks;
} claim;

static struct {
    bool started;
    uint32_t t0, core0, core1, ffts, pairs, pairs_done;
    uint32_t heap_free, heap_largest;
    uint16_t mode;
} stats;

/* ---- setup -------------------------------------------------------------- */

bool spectrum_init(void)
{
    if (fft_ready) {
        return true;
    }
    static int16_t *twiddles;  /* heap: BSS must end below the RF ring */
    if (!twiddles) {
        twiddles = heap_caps_aligned_alloc(16, ESP_SDR_MAX_NFFT * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    assist_buffer = heap_caps_aligned_alloc(16, 2 * ESP_SDR_MAX_NFFT * sizeof(int16_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    db_code_init();
    fft_ready = twiddles && assist_buffer && dsps_fft2r_init_sc16(twiddles, ESP_SDR_MAX_NFFT) == ESP_OK;
    return fft_ready;
}

bool spectrum_valid_nfft(uint32_t n)
{
    return n >= ESP_SDR_MIN_NFFT && n <= ESP_SDR_MAX_NFFT && !(n & (n - 1));
}

void spectrum_set_dc_mode(uint8_t mode)
{
    dc_mode = mode;
}

/* Hann window and bit-reversal map for size n. Scaling is independent of n:
 * IQ10 << 6 in, the int16 FFT divides by n. */
static void prepare_tables(unsigned n)
{
    if (n == nfft) {
        return;
    }
    unsigned log2n = 0;
    while ((1u << log2n) < n) {
        log2n++;
    }
    for (unsigned i = 0; i < n; i++) {
        window_q15[i] = (int16_t)lrintf(32767.0f * 0.5f * (1.0f - cosf(2.0f * PI_F * (float)i / (float)n)));
        window_pairs[2 * i] = window_pairs[2 * i + 1] = window_q15[i];
        unsigned reversed = 0;
        for (unsigned bit = 0; bit < log2n; bit++) {
            reversed |= ((i >> bit) & 1u) << (log2n - 1 - bit);
        }
        natural_bin[i] = (uint16_t)reversed;
    }
    nfft = n;
    nfft_log2 = log2n;
}

/* ---- block stages ------------------------------------------------------- */

static inline int16_t saturate16(int32_t v)
{
    return v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)v;
}

/* DC handling in the bit-reversed Q15 FFT domain: subtract the (tracked) bin-0
 * value and spread half of it to the two bins where a DC step leaks. */
static inline __attribute__((always_inline)) void remove_dc(int16_t *x)
{
    int32_t dr = x[0], di = x[1];
    if (dc_mode) {
        if (!dc_estimate.ready) {
            dc_estimate.re = dr * 256;
            dc_estimate.im = di * 256;
            dc_estimate.ready = true;
        } else {
            dc_estimate.re += (dr * 256 - dc_estimate.re) >> 6;
            dc_estimate.im += (di * 256 - dc_estimate.im) >> 6;
        }
        dr = (dc_estimate.re + 128) >> 8;
        di = (dc_estimate.im + 128) >> 8;
    }
    x[0] = saturate16(x[0] - dr);
    x[1] = saturate16(x[1] - di);
    for (unsigned k = 0; k < 2; k++) {
        unsigned j = k ? nfft - 1 : nfft / 2;
        x[2 * j] = saturate16(x[2 * j] + dr / 2);
        x[2 * j + 1] = saturate16(x[2 * j + 1] + di / 2);
    }
}

/* Window pairs [from, to) of the block starting at ring index `at` into dst.
 * The PIE kernel reads one vector past its input, so groups touching the
 * ring wrap or the bank end are done in C (bit-identical). */
IRAM_ATTR static void unpack(int16_t *dst, const uint32_t *bank, unsigned at, unsigned from, unsigned to)
{
    while (from < to) {
        unsigned first = (at + from) & DUMP_RING_MASK;
        unsigned available = first + 4 < DUMP_RING_PAIRS ? DUMP_RING_PAIRS - first - 4 : 0;
        unsigned count = to - from < available ? to - from : available;
        unsigned bulk = count & ~7u;
        if (bulk) {
            iq10_unpack_windowed(bank + first, dst + 2 * from, window_pairs + 2 * from, bulk / 8);
            from += bulk;
            continue;
        }
        unsigned end = from + 8 < to ? from + 8 : to;
        for (; from < end; from++) {
            uint32_t w = bank[(at + from) & DUMP_RING_MASK];
            dst[2 * from] = (int16_t)((esp_sdr_word_i(w) * window_q15[from]) >> 9);
            dst[2 * from + 1] = (int16_t)((esp_sdr_word_q(w) * window_q15[from]) >> 9);
        }
    }
}

IRAM_ATTR static void accumulate(const int16_t *x, bool max_hold, unsigned from, unsigned to)
{
    x += 2 * from;
    if (max_hold) {
        uint32_t *peak = (uint32_t *)accumulator;
        for (unsigned k = from; k < to; k++, x += 2) {
            uint32_t power = (uint32_t)(x[0] * x[0]) + (uint32_t)(x[1] * x[1]);  /* <= 2^31 */
            if (power > peak[k]) {
                peak[k] = power;
            }
        }
    } else {
        for (unsigned k = from; k < to; k++, x += 2) {
            accumulator[k] += (float)((uint32_t)(x[0] * x[0]) + (uint32_t)(x[1] * x[1]));
        }
    }
}

static inline uint32_t power_bits(uint32_t stored, bool max_hold)
{
    if (!max_hold) {
        return stored;  /* already float bits */
    }
    union { float f; uint32_t u; } v = {.f = (float)stored};
    return v.u;
}

/* ---- statistics frames -------------------------------------------------- */

IRAM_ATTR static void emit_stats(bool from_worker)
{
    uint32_t now = esp_cpu_get_cycle_count();  /* producer's own clock */
    esp_sdr_run_result_t *r = g_stream.result;
    if (!stats.started) {
        stats.started = true;
        stats.t0 = now;
        stats.core0 = g_load.core0;
        stats.core1 = g_load.core1;
        stats.ffts = r->ffts;
        stats.pairs = stats.pairs_done;
        return;
    }
    uint32_t dt = now - stats.t0;
    if (dt < STATS_PERIOD_CYCLES) {
        return;
    }
    uint32_t per_mille = dt / 1000u, ms = dt / 240000u;
    uint32_t core0 = g_load.core0, core1 = g_load.core1;
    uint32_t ffts = r->ffts - stats.ffts, pairs = stats.pairs_done - stats.pairs;
    uint32_t c0 = (core0 - stats.core0) / per_mille, c1 = (core1 - stats.core1) / per_mille;
    uint32_t coverage = pairs >= 1000u ? (ffts * nfft) / (pairs / 1000u) : 0;
    uint8_t out[ESP_SDR_RECORD_HEADER_SIZE + sizeof(esp_sdr_record_stats_t) + ESP_SDR_RECORD_CRC_SIZE];
    esp_sdr_record_stats_t m = {
        .core0_permille = (uint16_t)(c0 > 1000 ? 1000 : c0),
        .core1_permille = (uint16_t)(c1 > 1000 ? 1000 : c1),
        .coverage_permille = (uint16_t)(coverage > 1000 ? 1000 : coverage),
        .mode = stats.mode,
        .heap_free = stats.heap_free,
        .heap_largest = stats.heap_largest,
        .abandoned = r->abandoned,
        .drops = r->drops,
        .late_max = (uint16_t)(r->late_max > 65535 ? 65535 : r->late_max),
        .queue_permille = (uint16_t)tx_queue_fill_permille(),
        .ffts_per_s = ms ? ffts * 1000u / ms : 0,
    };
    uint32_t crc = record_write_header(out, ESP_SDR_RECORD_STATS, g_stream.config->tag, sizeof(m));
    const uint8_t *mp = (const uint8_t *)&m;
    for (unsigned i = 0; i < sizeof(m); i++) {  /* core 1: no flash memcpy */
        out[ESP_SDR_RECORD_HEADER_SIZE + i] = mp[i];
    }
    crc = crc32_update(crc, out + ESP_SDR_RECORD_HEADER_SIZE, sizeof(m));
    for (unsigned i = 0; i < 4; i++) {
        out[ESP_SDR_RECORD_HEADER_SIZE + sizeof(m) + i] = (uint8_t)(crc >> (8u * i));
    }
    (void)(from_worker ? tx_queue_push_from_worker(out, sizeof(out)) : tx_queue_push(out, sizeof(out)));
    stats.t0 = now;
    stats.core0 = core0;
    stats.core1 = core1;
    stats.ffts = r->ffts;
    stats.pairs = stats.pairs_done;
}

/* ---- frame header ------------------------------------------------------- */

/* Header + payload header for the frame being closed; returns CRC so far. */
STREAM_HOT static uint32_t write_frame_header(void)
{
    esp_sdr_run_result_t *r = g_stream.result;
    const bool max_hold = g_stream.config->max_hold;
    esp_sdr_record_spectrum_frame_t h = {
        .frame = r->frames + r->drops,
        .pair_index = run.frame_index,
        .pairs = run.frame_pairs,
        .ffts = (uint16_t)run.frame_ffts,
        .flags = (uint8_t)((max_hold ? FLAG_MAX_HOLD : 0) | run.frame_flags | (run.dropped ? FLAG_DROPPED_BEFORE : 0)),
        .gain = run.frame_gain,
        .drops = (uint16_t)(r->drops > 65535 ? 65535 : r->drops),
        .nfft_log2 = (uint8_t)nfft_log2,
        .db_step = DB_CODE_STEP,
    };
    uint32_t crc = record_write_header(frame_out, ESP_SDR_RECORD_SPECTRUM_FRAME, g_stream.config->tag,
                                      (uint32_t)(FRAME_PAYLOAD_HEADER + nfft));
    const uint8_t *hp = (const uint8_t *)&h;
    for (unsigned i = 0; i < FRAME_PAYLOAD_HEADER; i++) {
        frame_out[ESP_SDR_RECORD_HEADER_SIZE + i] = hp[i];
    }
    return crc32_update(crc, frame_out + ESP_SDR_RECORD_HEADER_SIZE, FRAME_PAYLOAD_HEADER);
}

STREAM_HOT static int32_t frame_offset_q4(void)
{
    return (!g_stream.config->max_hold && run.frame_ffts >= 2) ? db_mean_offset_q4(run.frame_ffts) : 0;
}

STREAM_HOT static void reset_frame(void)
{
    run.frame_units = run.frame_ffts = 0;
    run.frame_pairs = 0;
    run.frame_flags = 0;
}

STREAM_HOT static void count_push(bool accepted)
{
    if (accepted) {
        g_stream.result->frames++;
        run.dropped = false;
    } else {
        g_stream.result->drops++;
        run.dropped = true;
    }
}

/* ---- single-core path --------------------------------------------------- */

static void emit_chunk(void);

/* Close the accumulating frame; emit_chunk() encodes its bins in slices. */
STREAM_HOT static void close_frame(void)
{
    while (run.emitting) {
        emit_chunk();  /* rare: previous frame not out yet */
    }
    if (!run.frame_ffts) {
        g_stream.result->drops++;
        run.dropped = true;
        reset_frame();
        return;
    }
    run.emit_crc = write_frame_header();
    run.emit_offset_q4 = frame_offset_q4();
    run.emit_position = 0;
    run.emitting = true;
    reset_frame();
}

STREAM_HOT static void emit_chunk(void)
{
    uint8_t *bins = frame_out + FRAME_BINS_AT;
    unsigned end = run.emit_position + ACCUMULATE_CHUNK < nfft ? run.emit_position + ACCUMULATE_CHUNK : nfft;
    uint32_t *stored = (uint32_t *)accumulator;
    const bool max_hold = g_stream.config->max_hold;
    for (unsigned k = run.emit_position; k < end; k++) {
        unsigned slot = natural_bin[k];
        bins[k] = db_code(power_bits(stored[slot], max_hold), run.emit_offset_q4);
        stored[slot] = 0;
    }
    run.emit_crc = crc32_update(run.emit_crc, bins + run.emit_position, end - run.emit_position);
    run.emit_position = end;
    if (end < nfft) {
        return;
    }
    size_t length = FRAME_BINS_AT + nfft;
    memcpy(frame_out + length, &run.emit_crc, ESP_SDR_RECORD_CRC_SIZE);
    count_push(tx_queue_push(frame_out, (uint32_t)(length + ESP_SDR_RECORD_CRC_SIZE)));
    run.emitting = false;
    if (g_stream.config->stats) {
        emit_stats(false);
    }
}

/* Retire the oldest unit before its bank is reused. Once unpacked, a block's
 * FFT owns its input copy and may finish later; frame closure waits for it. */
STREAM_HOT static void retire_oldest_unit(void)
{
    unsigned bank = run.fifo[0];
    pending_unit_t *u = &run.unit[bank];
    if (u->next_block < u->blocks) {
        g_stream.result->abandoned += u->blocks - u->next_block;
        run.frame_flags |= FLAG_ABANDONED;
    }
    if (run.step == STEP_UNPACK && run.block_bank == bank) {
        run.step = STEP_IDLE;
        g_stream.result->abandoned++;
        run.frame_flags |= FLAG_ABANDONED;
    }
    u->pending = false;
    for (unsigned i = 1; i < run.fifo_len; i++) {
        run.fifo[i - 1] = run.fifo[i];
    }
    run.fifo_len--;
    if (!run.frame_units) {
        run.frame_index = u->index;
        run.frame_gain = u->gain;
    }
    run.frame_pairs += u->count;
    stats.pairs_done += u->count;
    /* Emit a fresh frame as soon as the previous output has drained. */
    if (++run.frame_units && run.step == STEP_IDLE && run.frame_ffts && tx_queue_empty()) {
        close_frame();
    }
}

void spectrum_accept_unit(const stream_unit_t *unit)
{
    run.unit[unit->bank] = (pending_unit_t){
        .pending = true, .first = unit->first, .count = unit->count, .next_block = 0,
        .blocks = unit->count >> nfft_log2, .index = unit->index, .gain = unit->gain,
    };
    run.fifo[run.fifo_len++] = unit->bank;
}

STREAM_HOT void spectrum_release_bank(unsigned bank)
{
    while (run.unit[bank].pending) {
        retire_oldest_unit();  /* FIFO order: older units retire first */
    }
}

STREAM_HOT bool spectrum_bank_idle(unsigned bank)
{
    return !run.unit[bank].pending;
}

STREAM_HOT bool spectrum_work_slice(void)
{
    const uint32_t t0 = esp_cpu_get_cycle_count();
    if (run.emitting) {
        emit_chunk();
    } else if (run.step == STEP_FFT) {
        fft_sc16_rounded_stage(fft_buffer, (int)nfft, dsps_fft_w_table_sc16, run.fft_stage);
        if (++run.fft_stage == nfft_log2) {
            remove_dc(fft_buffer);
            run.step = STEP_ACCUMULATE;
            run.position = 0;
        }
    } else if (run.step == STEP_ACCUMULATE) {
        unsigned end = run.position + ACCUMULATE_CHUNK < nfft ? run.position + ACCUMULATE_CHUNK : nfft;
        accumulate(fft_buffer, g_stream.config->max_hold, run.position, end);
        run.position = end;
        if (end == nfft) {
            run.step = STEP_IDLE;
            run.frame_ffts++;
            g_stream.result->ffts++;
            if (tx_queue_empty()) {
                close_frame();
            }
        }
    } else if (run.step == STEP_UNPACK) {
        unsigned end = run.position + UNPACK_CHUNK < nfft ? run.position + UNPACK_CHUNK : nfft;
        unpack(fft_buffer, dump_bank(run.block_bank), run.block_at, run.position, end);
        run.position = end;
        if (end == nfft) {
            run.step = STEP_FFT;
            run.fft_stage = 0;
        }
    } else {
        if (!run.fifo_len) {
            return false;
        }
        unsigned bank = run.fifo[0];
        pending_unit_t *u = &run.unit[bank];
        if (u->next_block >= u->blocks) {
            retire_oldest_unit();
        } else {
            run.block_bank = bank;
            run.block_at = u->first + (u->next_block << nfft_log2);
            u->next_block++;
            run.step = STEP_UNPACK;
            run.position = 0;
        }
    }
    uint32_t dt = esp_cpu_get_cycle_count() - t0;
    if (dt > g_stream.result->work_max) {
        g_stream.result->work_max = dt;
    }
    return true;
}

void spectrum_finish_single(void)
{
    while (spectrum_work_slice()) {
        tx_queue_pump();
    }
    if (run.frame_units) {
        close_frame();
        while (run.emitting) {
            emit_chunk();
        }
    }
}

/* ---- dual-core path ----------------------------------------------------- */

static inline bool compare_and_set(volatile uint32_t *address, uint32_t expected, uint32_t desired)
{
    uint32_t value = desired;
    __asm__ volatile("wsr %1, scompare1\n"
                     "s32c1i %0, %2, 0\n"
                     : "+a"(value)
                     : "a"(expected), "a"(address)
                     : "memory");
    return value == expected;
}

IRAM_ATTR static bool claim_block(uint32_t *block)
{
    for (;;) {
        uint32_t b = claim.next;
        if (b >= claim.blocks) {
            return false;
        }
        if (compare_and_set(&claim.next, b, b + 1)) {
            *block = b;
            return true;
        }
    }
}

/* One slice of the pending frame: 256 bins, then 1 KiB of CRC, then the push. */
IRAM_ATTR static void worker_encode_step(void)
{
    if (!encoder.pending) {
        return;
    }
    if (encoder.bin < nfft) {
        unsigned end = encoder.bin + WORKER_ENCODE_CHUNK < nfft ? encoder.bin + WORKER_ENCODE_CHUNK : nfft;
        uint8_t *bins = frame_out + FRAME_BINS_AT;
        for (unsigned k = encoder.bin; k < end; k++) {
            bins[natural_bin[k]] = db_code(power_bits(encoder.source[k], encoder.max_hold), encoder.offset_q4);
            encoder.source[k] = 0;
        }
        encoder.bin = end;
        return;
    }
    if (encoder.crc_at < encoder.length) {
        uint32_t n = encoder.length - encoder.crc_at;
        if (n > WORKER_CRC_CHUNK) {
            n = WORKER_CRC_CHUNK;
        }
        encoder.crc = crc32_update(encoder.crc, frame_out + encoder.crc_at, n);
        encoder.crc_at += n;
        return;
    }
    for (unsigned i = 0; i < 4; i++) {
        frame_out[encoder.length + i] = (uint8_t)(encoder.crc >> (8u * i));
    }
    count_push(tx_queue_push_from_worker(frame_out, encoder.length + ESP_SDR_RECORD_CRC_SIZE));
    encoder.pending = 0;
    if (g_stream.config->stats) {
        emit_stats(true);
    }
}

IRAM_ATTR static void worker_close_frame(void)
{
    while (encoder.pending) {
        worker_encode_step();  /* frame_out still holds the previous frame */
    }
    (void)write_frame_header();
    encoder.source = (uint32_t *)accumulator;
    encoder.offset_q4 = frame_offset_q4();
    encoder.max_hold = g_stream.config->max_hold;
    encoder.bin = 0;
    encoder.crc = 0;
    encoder.crc_at = 2;  /* CRC covers type .. payload */
    encoder.length = FRAME_BINS_AT + nfft;
    encoder.pending = 1;
    accumulator = accumulator == accumulators[0] ? accumulators[1] : accumulators[0];  /* already zeroed */
    reset_frame();
}

IRAM_ATTR static void take_handoff(void)
{
    remove_dc(assist_buffer);
    accumulate(assist_buffer, g_stream.config->max_hold, 0, nfft);
    run.frame_ffts++;
    g_stream.result->ffts++;
    claim.done++;
    MEMORY_BARRIER();
    claim.handoff_full = 0;
}

IRAM_ATTR static void worker_unit(const stream_unit_t *u)
{
    const uint32_t unit_start = esp_cpu_get_cycle_count();
    const uint32_t blocks = u->count >> nfft_log2;
    claim.bank = u->bank;
    claim.first = u->first;
    claim.blocks = blocks;
    claim.next = 0;
    claim.done = 0;
    MEMORY_BARRIER();
    claim.seq = u->seq;  /* open for claims (core 0 too) */
    MEMORY_BARRIER();
    bool revoked = false;
    for (;;) {
        if (claim.handoff_full) {
            take_handoff();
        }
        uint32_t block;
        if (revoked || !claim_block(&block)) {
            break;
        }
        const uint32_t t0 = esp_cpu_get_cycle_count();
        const uint32_t *bank = dump_bank(u->bank);
        unsigned at = u->first + (block << nfft_log2);
        for (unsigned s = 0; s < nfft && !revoked; s += WORKER_UNPACK_CHUNK) {
            if (!worker_lock_bank(u->bank, u->seq)) {
                revoked = true;  /* core 0 needs the bank */
                break;
            }
            unpack(fft_buffer, bank, at, s, s + WORKER_UNPACK_CHUNK);
            worker_unlock_bank();
        }
        if (revoked) {
            claim.next = claim.blocks;  /* stop further claims; the partial block is dropped */
            continue;
        }
        fft_sc16_rounded(fft_buffer, (int)nfft, dsps_fft_w_table_sc16);
        remove_dc(fft_buffer);
        accumulate(fft_buffer, g_stream.config->max_hold, 0, nfft);
        run.frame_ffts++;
        g_stream.result->ffts++;
        claim.done++;
        uint32_t dt = esp_cpu_get_cycle_count() - t0;
        if (dt > g_worker.block_max) {
            g_worker.block_max = dt;
        }
    }
    /* Close: no more claims; collect core 0's block still in flight. */
    claim.next = claim.blocks;
    MEMORY_BARRIER();
    while (claim.assist_seq == u->seq || (claim.handoff_full && claim.handoff_seq == u->seq)) {
        if (claim.handoff_full) {
            take_handoff();
        }
    }
    claim.seq = 0;
    MEMORY_BARRIER();
    if (claim.done < blocks) {
        g_stream.result->abandoned += blocks - claim.done;
        run.frame_flags |= FLAG_ABANDONED;
    }
    worker_release_bank(u->bank, u->seq);
    if (!run.frame_units) {
        run.frame_index = u->index;
        run.frame_gain = u->gain;
    }
    run.frame_pairs += u->count;
    stats.pairs_done += u->count;
    if (++run.frame_units && run.frame_ffts && !encoder.pending && tx_queue_empty()) {
        worker_close_frame();
    }
    g_load.core1 += esp_cpu_get_cycle_count() - unit_start;
}

IRAM_ATTR static void worker_finish(void)
{
    if (run.frame_units) {
        worker_close_frame();
    }
    while (encoder.pending) {
        worker_encode_step();
    }
}

const worker_hooks_t spectrum_worker_hooks = {
    .unit = worker_unit,
    .idle = worker_encode_step,
    .finish = worker_finish,
};

/* Core 0: transform one block of core 1's open unit into the hand-off buffer.
 * Called only when it fits before the next ring deadline. */
IRAM_ATTR void spectrum_assist(void)
{
    if (claim.handoff_full) {
        return;
    }
    uint32_t seq = claim.seq;
    if (!seq) {
        return;
    }
    claim.assist_seq = seq;
    MEMORY_BARRIER();
    uint32_t block;
    if (claim.seq != seq || !claim_block(&block)) {
        claim.assist_seq = 0;
        return;
    }
    unsigned bank = claim.bank;
    if (g_worker.bank_seq[bank] != seq) {  /* revoked meanwhile (by core 0 itself) */
        claim.assist_seq = 0;
        return;
    }
    unpack(assist_buffer, dump_bank(bank), claim.first + (block << nfft_log2), 0, nfft);
    fft_sc16_rounded(assist_buffer, (int)nfft, dsps_fft_w_table_sc16);
    claim.handoff_seq = seq;
    MEMORY_BARRIER();
    claim.handoff_full = 1;
    claim.assist_blocks++;
    MEMORY_BARRIER();
    claim.assist_seq = 0;
}

uint32_t spectrum_assist_blocks(void)
{
    return claim.assist_blocks;
}

/* ---- run setup ---------------------------------------------------------- */

uint32_t spectrum_begin(bool dual, bool assist)
{
    memset(&run, 0, sizeof(run));
    memset(&stats, 0, sizeof(stats));
    prepare_tables(g_stream.config->nfft);
    dc_estimate.ready = false;

    /* Warm caches and tables on bank 2 (sentinels only) and measure the stages,
     * so the first slices are scheduled against real costs. */
    const uint32_t assist_start = esp_cpu_get_cycle_count();
    unpack(fft_buffer, dump_bank(2), 0, 0, nfft);
    uint32_t longest = 0;
    for (unsigned stage = 0; stage < nfft_log2; stage++) {
        uint32_t t = esp_cpu_get_cycle_count();
        fft_sc16_rounded_stage(fft_buffer, (int)nfft, dsps_fft_w_table_sc16, stage);
        uint32_t dt = esp_cpu_get_cycle_count() - t;
        if (dt > longest) {
            longest = dt;
        }
    }
    const uint32_t assist_cost = esp_cpu_get_cycle_count() - assist_start;
    uint32_t t = esp_cpu_get_cycle_count();
    accumulate(fft_buffer, false, 0, ACCUMULATE_CHUNK < nfft ? ACCUMULATE_CHUNK : nfft);
    uint32_t dt = esp_cpu_get_cycle_count() - t;
    if (dt > longest) {
        longest = dt;
    }
    memset(accumulators, 0, sizeof(accumulators));
    accumulator = accumulators[0];
    encoder.pending = 0;
    dc_estimate.ready = false;
    /* +15 % for cache and bus variation. */
    g_stream.result->work_max = longest + longest / 7u;

    stats.heap_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    stats.heap_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    stats.mode = (uint16_t)((dual ? 1 : 0) | (dual && assist ? 2 : 0));
    claim.seq = claim.assist_seq = claim.handoff_full = claim.assist_blocks = 0;
    return assist_cost;
}
