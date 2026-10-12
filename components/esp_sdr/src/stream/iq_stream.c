#include "iq_stream.h"

#include <stdlib.h>
#include <string.h>
#include "crc32.h"
#include "dump_regs.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_sdr_record.h"
#include "fir_design.h"
#include "record.h"

#define FRAME_PAYLOAD_BYTES 1024u
#define FRAME_PAYLOAD_HEADER sizeof(esp_sdr_record_iq_frame_t)
#define FRAME_DATA_AT (ESP_SDR_RECORD_HEADER_SIZE + FRAME_PAYLOAD_HEADER)
#define FEED_CHUNK 512u            /* pairs per FIR feed */
#define SLICE_PAIRS 1024u          /* single-core slice */
#define FINISH_SLICE_PAIRS 4096u
#define INITIAL_SLICE_COST 8000u   /* cycles, until measured */
#define STAGE1_SHIFT 17u           /* ring I<<6 -> stage 1 I*16 */
#define STAGE2_SHIFT 14u           /* -> stage 2 I*32 */
#define FLAG_GAP 1u
#define FLAG_DROPPED_BEFORE 2u
#define NO_PAIR UINT64_MAX

/* PIE kernels (fir_kernels.S). Windows start 16-byte aligned. */
typedef struct {
    const int16_t *xi, *xq, *taps;
    uint32_t groups, outputs, step, shift;
    int16_t *oi, *oq;
} fir_job_t;
void iq10_unpack_split(const uint32_t *src, int16_t *di, int16_t *dq, unsigned groups8);
void fir_dot_split(const fir_job_t *job);
void fir_dot_decim8_taps32(const fir_job_t *job);
void iq_rotate_quarter_fs(int16_t *wi, int16_t *wq, unsigned groups8, const int16_t *masks);

/* z * j^n (shift by +fs/4): m1 = {1,0,-1,0}, m2 = {0,-1,0,1}, m3 = -m2. */
static const int16_t ROTATE_MASKS[24] __attribute__((aligned(16))) = {
    1, 0, -1, 0, 1, 0, -1, 0, 0, -1, 0, 1, 0, -1, 0, 1, 0, 1, 0, -1, 0, 1, 0, -1};

/* One FIR stage with L samples of history, continuous across slices and units. */
typedef struct {
    uint32_t decimation, taps, filled, capacity, shift;
    uint64_t base;      /* input index of wi[0] */
    uint64_t next_end;  /* last input index of the next output */
    int16_t *h, *wi, *wq;
} fir_stage_t;

static fir_stage_t stage1, stage2;
static int16_t *out_i, *out_q;
static bool rotate;

static struct {
    bool pending;
    unsigned bank;
    uint32_t first, count, position;
    uint64_t index;          /* stream index of the unit's first pair */
    uint64_t next_pair;      /* index the filter continues from */
    uint32_t decimation, log2_decimation, bits, shift;
    uint64_t out_index;      /* decimated index of the next output sample */
    uint64_t frame_index;    /* decimated index of the frame's first sample */
    uint32_t fill, frame, lost_pairs;
    uint8_t flags, gain;
    uint32_t slice_cost;
    uint64_t cycles, pairs_done;
    uint8_t *out;            /* frame buffer, heap */
} iq;

/* ---- filter stages ------------------------------------------------------ */

static void *dma_alloc(size_t size)
{
    return heap_caps_aligned_alloc(16, size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
}

static bool stage_init(fir_stage_t *s, uint32_t decimation, uint32_t taps, uint32_t shift, uint32_t input_max)
{
    s->decimation = decimation;
    s->taps = taps;
    s->shift = shift;
    s->capacity = 2u * taps + input_max + 16u;
    s->h = dma_alloc(2u * taps);
    s->wi = dma_alloc(2u * s->capacity);
    s->wq = dma_alloc(2u * s->capacity);
    if (!s->h || !s->wi || !s->wq) {
        return false;
    }
    if (decimation == FIR_STAGE1_DECIMATION && taps == FIR_STAGE1_TAPS) {
        fir_design_stage1(s->h);
    } else {
        fir_design_lowpass(s->h, taps, decimation);
    }
    return true;
}

static void stage_free(fir_stage_t *s)
{
    free(s->h);
    free(s->wi);
    free(s->wq);
    s->h = s->wi = s->wq = NULL;
}

static void stage_reset(fir_stage_t *s, uint64_t start)
{
    memset(s->wi, 0, 2u * s->taps);
    memset(s->wq, 0, 2u * s->taps);
    s->filled = s->taps;
    s->base = start - s->taps;
    s->next_end = start + s->decimation - 1u;
}

/* All complete outputs to oi/oq; returns their count; drops consumed history. */
IRAM_ATTR static uint32_t stage_run(fir_stage_t *s, int16_t *oi, int16_t *oq)
{
    uint32_t outputs = 0;
    if (s->next_end < s->base + s->filled) {
        outputs = (uint32_t)(s->base + s->filled - 1u - s->next_end) / s->decimation + 1u;
        uint32_t offset = (uint32_t)(s->next_end + 1u - s->taps - s->base);
        fir_job_t job = {s->wi + offset, s->wq + offset, s->h, s->taps / 8u, outputs, 2u * s->decimation,
                         s->shift, oi, oq};
        if (s->decimation == FIR_STAGE1_DECIMATION && s->taps == FIR_STAGE1_TAPS) {
            fir_dot_decim8_taps32(&job);
        } else {
            fir_dot_split(&job);
        }
        s->next_end += (uint64_t)outputs * s->decimation;
    }
    /* Drop consumed samples only when the rest (< taps) does not overlap them,
     * so a plain memcpy suffices; otherwise filled stays below 2 * taps. */
    uint32_t keep = (uint32_t)(s->next_end + 1u - s->taps - s->base);  /* multiple of 8 */
    if (keep && keep >= s->filled - keep) {
        memcpy(s->wi, s->wi + keep, 2u * (s->filled - keep));
        memcpy(s->wq, s->wq + keep, 2u * (s->filled - keep));
        s->base += keep;
        s->filled -= keep;
    }
    return outputs;
}

static inline void stage1_push_one(const uint32_t *bank, uint32_t at)
{
    uint32_t w = bank[at];
    int16_t i = (int16_t)(esp_sdr_word_i(w) * 64), q = (int16_t)(esp_sdr_word_q(w) * 64);
    if (rotate) {
        switch ((uint32_t)(stage1.base + stage1.filled) & 3u) {  /* z * j^n */
        case 1: { int16_t t = i; i = (int16_t)-q; q = t; break; }
        case 2: i = (int16_t)-i; q = (int16_t)-q; break;
        case 3: { int16_t t = i; i = q; q = (int16_t)-t; break; }
        default: break;
        }
    }
    stage1.wi[stage1.filled] = i;
    stage1.wq[stage1.filled] = q;
    stage1.filled++;
}

/* ---- output frames ------------------------------------------------------ */

IRAM_ATTR static void flush_frame(void)
{
    if (!iq.fill) {
        return;
    }
    unsigned bytes_per_sample = iq.bits == 16 ? 4u : iq.bits == 8 ? 2u : 1u;
    esp_sdr_record_iq_frame_t h = {
        .frame = iq.frame, .sample_index = iq.frame_index, .samples = (uint16_t)(iq.fill / bytes_per_sample),
        .bits = (uint8_t)iq.bits, .flags = iq.flags, .decimation = (uint16_t)iq.decimation,
        .gain = iq.gain, .shift = (uint8_t)iq.shift,
    };
    uint32_t length = (uint32_t)FRAME_PAYLOAD_HEADER + iq.fill;
    uint32_t crc = record_write_header(iq.out, ESP_SDR_RECORD_IQ_FRAME, g_stream.config->tag, length);
    const uint8_t *hp = (const uint8_t *)&h;
    for (unsigned i = 0; i < FRAME_PAYLOAD_HEADER; i++) {
        iq.out[ESP_SDR_RECORD_HEADER_SIZE + i] = hp[i];
    }
    crc = crc32_update(crc, iq.out + ESP_SDR_RECORD_HEADER_SIZE, length);
    uint8_t *tail = iq.out + ESP_SDR_RECORD_HEADER_SIZE + length;
    for (unsigned i = 0; i < 4; i++) {
        tail[i] = (uint8_t)(crc >> (8u * i));
    }
    if (tx_queue_push(iq.out, ESP_SDR_RECORD_HEADER_SIZE + length + ESP_SDR_RECORD_CRC_SIZE)) {
        g_stream.result->frames++;
        iq.frame++;
        iq.flags = 0;
    } else {
        g_stream.result->drops++;
        iq.flags |= FLAG_DROPPED_BEFORE;
    }
    iq.frame_index = iq.out_index;
    iq.fill = 0;
}

IRAM_ATTR static inline void put_sample(int32_t i, int32_t q)
{
    if (iq.shift) {  /* round to nearest: a plain shift floors and leaves a -0.5 LSB DC */
        int32_t half = 1 << (iq.shift - 1);
        i = (i + half) >> iq.shift;
        q = (q + half) >> iq.shift;
    }
    int32_t limit = (1 << (iq.bits - 1)) - 1;
    i = i > limit ? limit : i < -limit - 1 ? -limit - 1 : i;
    q = q > limit ? limit : q < -limit - 1 ? -limit - 1 : q;
    uint8_t *o = iq.out + FRAME_DATA_AT + iq.fill;
    if (iq.bits == 8) {
        o[0] = (uint8_t)i;
        o[1] = (uint8_t)q;
        iq.fill += 2;
    } else if (iq.bits == 16) {
        o[0] = (uint8_t)i;
        o[1] = (uint8_t)(i >> 8);
        o[2] = (uint8_t)q;
        o[3] = (uint8_t)(q >> 8);
        iq.fill += 4;
    } else {
        o[0] = (uint8_t)((i & 15) | (q << 4));
        iq.fill += 1;
    }
    iq.out_index++;
    if (iq.fill >= FRAME_PAYLOAD_BYTES) {
        flush_frame();
    }
}

/* `count` contiguous ring pairs from `at` (no wrap) through both stages. */
IRAM_ATTR static void feed(const uint32_t *bank, uint32_t at, uint32_t count)
{
    while (count) {
        uint32_t take = count > FEED_CHUNK ? FEED_CHUNK : count, done = 0;
        while ((stage1.filled & 7u) && done < take) {
            stage1_push_one(bank, at + done++);
        }
        uint32_t groups = (take - done) / 8u;
        if (groups) {
            iq10_unpack_split(bank + at + done, stage1.wi + stage1.filled, stage1.wq + stage1.filled, groups);
            if (rotate) {
                iq_rotate_quarter_fs(stage1.wi + stage1.filled, stage1.wq + stage1.filled, groups, ROTATE_MASKS);
            }
            stage1.filled += 8u * groups;
            done += 8u * groups;
        }
        while (done < take) {
            stage1_push_one(bank, at + done++);
        }
        stage2.filled += stage_run(&stage1, stage2.wi + stage2.filled, stage2.wq + stage2.filled);
        uint32_t outputs = stage_run(&stage2, out_i, out_q);
        for (uint32_t k = 0; k < outputs; k++) {
            put_sample(out_i[k], out_q[k]);
        }
        at += take;
        count -= take;
    }
}

/* A unit that does not continue the previous one restarts the filters on the
 * decimation grid. Returns false when the unit is too short to contribute. */
STREAM_HOT static bool align_unit(uint32_t *first, uint32_t *count, uint64_t *index)
{
    if (*index == iq.next_pair) {
        return true;
    }
    flush_frame();
    uint64_t skip = (iq.decimation - (*index & (iq.decimation - 1))) & (iq.decimation - 1);
    if (skip >= *count) {
        iq.next_pair = NO_PAIR;
        return false;
    }
    *first = (*first + (uint32_t)skip) & DUMP_RING_MASK;
    *count -= (uint32_t)skip;
    *index += skip;
    iq.out_index = iq.frame_index = *index >> iq.log2_decimation;
    stage_reset(&stage1, *index);
    stage_reset(&stage2, *index / FIR_STAGE1_DECIMATION);
    if (iq.frame) {
        iq.flags |= FLAG_GAP;
    }
    return true;
}

/* ---- setup -------------------------------------------------------------- */

static void free_filters(void)
{
    stage_free(&stage1);
    stage_free(&stage2);
    free(out_i);
    free(out_q);
    out_i = out_q = NULL;
}

bool iq_stream_begin(uint32_t *detail)
{
    const esp_sdr_run_config_t *c = g_stream.config;
    uint32_t d = c->iq_decimation, log2d = 0;
    while ((1u << log2d) < d) {
        log2d++;
    }
    if (d < 64u || d > 1024u || (1u << log2d) != d || (c->iq_bits != 4 && c->iq_bits != 8 && c->iq_bits != 16) ||
        c->iq_shift > 24) {
        *detail = 2;
        return false;
    }
    uint8_t *out = iq.out;
    memset(&iq, 0, sizeof(iq));
    iq.out = out;
    iq.decimation = d;
    iq.log2_decimation = log2d;
    iq.bits = c->iq_bits;
    iq.shift = c->iq_shift;
    iq.next_pair = NO_PAIR;
    iq.slice_cost = INITIAL_SLICE_COST;
    rotate = c->iq_rotate;
    if (!iq.out) {
        iq.out = heap_caps_malloc(FRAME_DATA_AT + FRAME_PAYLOAD_BYTES + ESP_SDR_RECORD_CRC_SIZE,
                                  MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!iq.out) {
            *detail = 3;
            return false;
        }
    }
    uint32_t d2 = d / FIR_STAGE1_DECIMATION;
    out_i = dma_alloc(2u * 80u);
    out_q = dma_alloc(2u * 80u);
    if (!out_i || !out_q || !stage_init(&stage1, FIR_STAGE1_DECIMATION, FIR_STAGE1_TAPS, STAGE1_SHIFT, FEED_CHUNK) ||
        !stage_init(&stage2, d2, fir_stage2_taps(d2), STAGE2_SHIFT, FEED_CHUNK / FIR_STAGE1_DECIMATION)) {
        free_filters();
        *detail = 4;
        return false;
    }
    return true;
}

/* ---- single-core path --------------------------------------------------- */

STREAM_HOT static void abandon_pending(void)
{
    if (!iq.pending) {
        return;
    }
    iq.lost_pairs += iq.count - iq.position;
    g_stream.result->abandoned++;
    iq.pending = false;
    iq.next_pair = NO_PAIR;
}

void iq_stream_accept_unit(const stream_unit_t *unit)
{
    abandon_pending();
    uint32_t first = unit->first, count = unit->count;
    uint64_t index = unit->index;
    if (!align_unit(&first, &count, &index)) {
        return;
    }
    iq.gain = (uint8_t)(dump_bank(unit->bank)[first] >> 20);
    iq.bank = unit->bank;
    iq.first = first;
    iq.count = count;
    iq.position = 0;
    iq.index = index;
    iq.pending = true;
}

STREAM_HOT bool iq_stream_pending(void)
{
    return iq.pending;
}

STREAM_HOT uint32_t iq_stream_slice_cost(void)
{
    return iq.slice_cost;
}

/* Kept out of IRAM (full); single-core fallback only. */
static void process(uint32_t pairs)
{
    const uint32_t *bank = dump_bank(iq.bank);
    uint32_t left = iq.count - iq.position;
    if (pairs > left) {
        pairs = left;
    }
    uint32_t at = (iq.first + iq.position) & DUMP_RING_MASK;
    for (uint32_t k = 0; k < pairs;) {
        uint32_t a = (at + k) & DUMP_RING_MASK, m = pairs - k;
        if (m > DUMP_RING_PAIRS - a) {
            m = DUMP_RING_PAIRS - a;
        }
        feed(bank, a, m);
        k += m;
    }
    iq.position += pairs;
    iq.pairs_done += pairs;
    if (iq.position >= iq.count) {
        iq.pending = false;
        iq.next_pair = iq.index + iq.count;
    }
}

uint32_t iq_stream_slice(void)
{
    if (!iq.pending) {
        return 0;
    }
    uint32_t t0 = esp_cpu_get_cycle_count();
    process(SLICE_PAIRS);
    uint32_t dt = esp_cpu_get_cycle_count() - t0;
    iq.cycles += dt;
    if (dt > iq.slice_cost) {
        iq.slice_cost = dt;
    }
    if (dt > g_stream.result->work_max) {
        g_stream.result->work_max = dt;
    }
    return dt;
}

STREAM_HOT void iq_stream_release_bank(unsigned bank)
{
    if (iq.pending && iq.bank == bank) {
        abandon_pending();
    }
}

void iq_stream_finish(bool dual)
{
    esp_sdr_run_result_t *r = g_stream.result;
    if (!dual) {
        while (iq.pending) {
            process(FINISH_SLICE_PAIRS);
            tx_queue_pump();
        }
    }
    flush_frame();
    r->lost_pairs = iq.lost_pairs;
    /* Single core: average cost in cycles per pair x100; dual: not measured. */
    r->work_max = dual || !iq.pairs_done ? 0 : (uint32_t)(iq.cycles * 100u / iq.pairs_done);
    free_filters();
}

/* ---- core 1 ------------------------------------------------------------- */

IRAM_ATTR static void worker_unit(const stream_unit_t *u)
{
    const uint32_t start = esp_cpu_get_cycle_count();
    uint32_t first = u->first, count = u->count;
    uint64_t index = u->index;
    if (align_unit(&first, &count, &index)) {
        iq.gain = u->gain;
        const uint32_t *bank = dump_bank(u->bank);
        uint32_t k = 0;
        bool revoked = false;
        while (k < count) {
            uint32_t a = (first + k) & DUMP_RING_MASK, m = count - k;
            if (m > FEED_CHUNK) {
                m = FEED_CHUNK;
            }
            if (m > DUMP_RING_PAIRS - a) {
                m = DUMP_RING_PAIRS - a;
            }
            if (!worker_lock_bank(u->bank, u->seq)) {
                revoked = true;
                break;
            }
            feed(bank, a, m);
            worker_unlock_bank();
            k += m;
        }
        if (revoked) {
            iq.lost_pairs += count - k;
            g_stream.result->abandoned++;
            iq.next_pair = NO_PAIR;
        } else {
            iq.next_pair = index + count;
        }
    }
    worker_release_bank(u->bank, u->seq);
    g_load.core1 += esp_cpu_get_cycle_count() - start;
}

IRAM_ATTR static void worker_nothing(void)
{
}

const worker_hooks_t iq_stream_worker_hooks = {
    .unit = worker_unit,
    .idle = worker_nothing,
    .finish = worker_nothing,
};
