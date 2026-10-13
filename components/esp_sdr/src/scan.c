/*
 * Full-band scan: tune, burst-capture (dwell times), FFT with a mean or peak
 * detector, keep the clean slice above the LO, optionally flatten the in-step
 * filter ripple and interpolate over the receiver's own clock spurs.
 */
#include <stdlib.h>
#include <string.h>
#include "burst.h"
#include "db_code.h"
#include "esp_heap_caps.h"
#include "esp_sdr.h"
#include "mode.h"
#include "esp_timer.h"
#include "rf.h"
#include "scan_plan.h"
#include "spectrum.h"

#define SPUR_GRID_HZ 8000000.0    /* gcd of the 40 MHz crystal and 16 MHz sample clock */
#define SPUR_WINDOW_HZ 20000.0    /* + up to one bin around each grid point */
#define SPUR_MIN_EXCESS 12        /* codes (6 dB) above the neighbouring floor */
#define SPUR_NEIGHBOURS 4         /* bins each side used for the floor */
#define FLATTEN_SIGNAL_CODES 12   /* bins 6 dB above their step's median are not floor */

/* In-step response, cached per configuration (index = FFT bin, 0..nfft/2). */
static struct {
    bool valid;
    esp_sdr_rate_t rate;
    uint16_t nfft;
    uint8_t gain;
    int filter;
    int8_t *correction;  /* codes to subtract; heap (BSS must end below the dump banks) */
} shape;

typedef struct {
    const esp_sdr_scan_config_t *config;
    scan_plan_t plan;
    uint32_t rate_hz;
    double bin_hz;
    uint32_t blocks;
    uint32_t samples;
    float *power;    /* nfft, combined over dwell */
    float *capture;  /* nfft, one capture */
    uint8_t *codes;  /* nfft / 2 */
} scan_t;

static uint8_t median_code(const uint8_t *codes, uint16_t n)
{
    uint16_t histogram[256] = {0};
    for (uint16_t i = 0; i < n; i++) {
        histogram[codes[i]]++;
    }
    uint32_t seen = 0;
    for (unsigned c = 0; c < 256; c++) {
        seen += histogram[c];
        if (2 * seen >= n) {
            return (uint8_t)c;
        }
    }
    return 0;
}

/* Bins of the step whose absolute frequency lies in [lo_hz, hi_hz): ascending. */
static uint16_t slice(uint32_t lo_mhz, double bin_hz, uint32_t nfft, double lo_hz, double hi_hz, uint32_t *first_bin)
{
    double base = (double)lo_mhz * 1e6;
    uint32_t a = (uint32_t)((lo_hz - base) / bin_hz + 0.999999);
    uint32_t b = (uint32_t)((hi_hz - base) / bin_hz + 0.999999);
    if (b > nfft / 2) {
        b = nfft / 2;  /* positive frequencies only */
    }
    *first_bin = a;
    return (uint16_t)(b > a ? b - a : 0);
}

/* Tune to step `step` and measure; returns bins (codes in s->codes). */
static esp_err_t measure_step(scan_t *s, uint32_t step, bool peak, uint8_t dwell, uint32_t *lo_out,
                              uint32_t *first_out, uint16_t *bins_out)
{
    const esp_sdr_scan_config_t *c = s->config;
    const uint32_t lo = scan_lo_mhz(&s->plan, step);
    const double seg_lo = ((double)c->from_mhz + (double)step * s->plan.keep_mhz) * 1e6;
    double seg_hi = seg_lo + s->plan.keep_mhz * 1e6;
    if (seg_hi > (double)c->to_mhz * 1e6) {
        seg_hi = (double)c->to_mhz * 1e6;
    }
    rf_set_frequency(lo);
    for (uint8_t d = 0; d < dwell; d++) {
        rf_filter_apply();
        uint32_t capture_us;
        esp_err_t err = burst_capture(c->rate, s->samples, &capture_us);
        rf_filter_restore();
        if (err != ESP_OK) {
            return err;
        }
        float *into = d ? s->capture : s->power;
        if (!spectrum_burst_power(burst_buffer(), c->nfft, s->blocks, peak, into)) {
            return ESP_FAIL;
        }
        for (uint32_t k = 0; d && k < c->nfft; k++) {
            if (peak) {
                s->power[k] = s->capture[k] > s->power[k] ? s->capture[k] : s->power[k];
            } else {
                s->power[k] += s->capture[k];
            }
        }
    }
    const float scale = peak ? 1.0f : 1.0f / (float)dwell;
    uint32_t first;
    uint16_t bins = slice(lo, s->bin_hz, c->nfft, seg_lo, seg_hi, &first);
    for (uint16_t i = 0; i < bins; i++) {
        union { float f; uint32_t u; } v = {.f = s->power[first + i] * scale};
        s->codes[i] = db_code(v.u, 0);
    }
    *lo_out = lo;
    *first_out = first;
    *bins_out = bins;
    return ESP_OK;
}

/* Learn the in-step response from one mean-detector sweep: per FFT bin, the
 * average of (code - step median) over all steps, skipping signal bins. */
static esp_err_t learn_shape(scan_t *s)
{
    const uint32_t half = s->config->nfft / 2;
    if (!shape.correction && !(shape.correction = heap_caps_malloc(ESP_SDR_MAX_NFFT / 2, MALLOC_CAP_INTERNAL))) {
        return ESP_ERR_NO_MEM;
    }
    int32_t *sum = calloc(half, sizeof(int32_t));
    uint16_t *count = calloc(half, sizeof(uint16_t));
    if (!sum || !count) {
        free(sum);
        free(count);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = ESP_OK;
    for (uint32_t step = 0; step < s->plan.steps && err == ESP_OK; step++) {
        uint32_t lo, first;
        uint16_t bins;
        err = measure_step(s, step, false, 1, &lo, &first, &bins);
        if (err != ESP_OK || bins < half / 4) {
            continue;  /* a short final step says little about the shape */
        }
        int median = median_code(s->codes, bins);
        for (uint16_t i = 0; i < bins; i++) {
            int r = (int)s->codes[i] - median;
            if (r < FLATTEN_SIGNAL_CODES) {
                sum[first + i] += r;
                count[first + i]++;
            }
        }
    }
    if (err == ESP_OK) {
        int32_t total = 0, used = 0;
        for (uint32_t k = 0; k < half; k++) {
            if (count[k]) {
                total += sum[k] / count[k];
                used++;
            }
        }
        int32_t mean = used ? total / used : 0;
        for (uint32_t k = 0; k < half; k++) {
            int32_t v = count[k] ? sum[k] / count[k] - mean : 0;
            shape.correction[k] = (int8_t)(v > 127 ? 127 : v < -127 ? -127 : v);
        }
        shape.valid = true;
        shape.rate = s->config->rate;
        shape.nfft = s->config->nfft;
        shape.gain = s->config->gain_index;
        shape.filter = rf_filter();
    }
    free(sum);
    free(count);
    return err;
}

static void flatten(uint8_t *codes, uint16_t bins, uint32_t first)
{
    for (uint16_t i = 0; i < bins; i++) {
        int v = (int)codes[i] - shape.correction[first + i];
        codes[i] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
    }
}

/* Replace narrow peaks on the n x 8 MHz grid by their neighbours' floor. */
static uint8_t suppress_spurs(uint8_t *codes, uint16_t bins, double start_hz, double bin_hz)
{
    uint8_t replaced = 0;
    const double window = SPUR_WINDOW_HZ + bin_hz;
    double grid = (double)(uint64_t)(start_hz / SPUR_GRID_HZ) * SPUR_GRID_HZ;
    for (; grid < start_hz + bins * bin_hz + window; grid += SPUR_GRID_HZ) {
        int lo = (int)((grid - window - start_hz) / bin_hz + 0.999999);
        int hi = (int)((grid + window - start_hz) / bin_hz);
        if (hi < 0 || lo >= bins) {
            continue;
        }
        lo = lo < 0 ? 0 : lo;
        hi = hi >= bins ? bins - 1 : hi;
        /* Floor: median of the neighbours outside the window. */
        uint8_t around[2 * SPUR_NEIGHBOURS];
        uint16_t n = 0;
        for (int k = lo - SPUR_NEIGHBOURS; k < lo; k++) {
            if (k >= 0) {
                around[n++] = codes[k];
            }
        }
        for (int k = hi + 1; k <= hi + SPUR_NEIGHBOURS; k++) {
            if (k < bins) {
                around[n++] = codes[k];
            }
        }
        if (!n) {
            continue;
        }
        uint8_t floor = median_code(around, n);
        bool spur = false;
        for (int k = lo; k <= hi; k++) {
            spur = spur || codes[k] >= floor + SPUR_MIN_EXCESS;
        }
        for (int k = lo; spur && k <= hi; k++) {
            codes[k] = floor;
            replaced++;
        }
    }
    return replaced;
}

esp_err_t esp_sdr_scan(const esp_sdr_scan_config_t *c, esp_sdr_scan_cb_t callback, void *ctx)
{
    if (!sdr_can_rx()) {
        return ESP_ERR_INVALID_STATE;
    }
    scan_t s = {.config = c, .rate_hz = esp_sdr_rate_hz(c->rate)};
    s.plan = scan_plan(c->from_mhz, c->to_mhz, s.rate_hz);
    if (!s.rate_hz || !callback || !spectrum_valid_nfft(c->nfft) || !c->averages || !c->dwell ||
        c->detector > ESP_SDR_DETECTOR_PEAK ||
        !scan_plan_valid(&s.plan, c->from_mhz, ESP_SDR_MIN_MHZ, ESP_SDR_MAX_MHZ)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->gain_index != ESP_SDR_SCAN_GAIN_AGC && c->gain_index > rf_gain_max()) {
        return ESP_ERR_INVALID_ARG;
    }
    s.blocks = ESP_SDR_MAX_BURST_SAMPLES / c->nfft;
    if (s.blocks > c->averages) {
        s.blocks = c->averages;
    }
    s.samples = s.blocks * c->nfft < ESP_SDR_MIN_BURST_SAMPLES ? ESP_SDR_MIN_BURST_SAMPLES : s.blocks * c->nfft;
    s.bin_hz = (double)s.rate_hz / c->nfft;
    /* Heap for the duration of one sweep only (internal SRAM is tight). */
    s.power = heap_caps_malloc(c->nfft * sizeof(float), MALLOC_CAP_INTERNAL);
    s.capture = c->dwell > 1 ? heap_caps_malloc(c->nfft * sizeof(float), MALLOC_CAP_INTERNAL) : NULL;
    s.codes = heap_caps_malloc(c->nfft / 2, MALLOC_CAP_INTERNAL);
    esp_err_t result = (!s.power || !s.codes || (c->dwell > 1 && !s.capture)) ? ESP_ERR_NO_MEM : ESP_OK;

    const uint32_t saved_mhz = rf_frequency();
    const bool saved_manual = rf_gain_manual();
    const uint8_t saved_index = rf_gain_index();
    if (result == ESP_OK) {
        rf_set_gain(c->gain_index != ESP_SDR_SCAN_GAIN_AGC, c->gain_index == ESP_SDR_SCAN_GAIN_AGC ? 0 : c->gain_index);
    }
    if (result == ESP_OK && c->flatten &&
        !(shape.valid && shape.rate == c->rate && shape.nfft == c->nfft && shape.gain == c->gain_index &&
          shape.filter == rf_filter())) {
        result = learn_shape(&s);
    }
    for (uint32_t step = 0; step < s.plan.steps && result == ESP_OK; step++) {
        const int64_t t0 = esp_timer_get_time();
        uint32_t lo, first;
        uint16_t bins;
        result = measure_step(&s, step, c->detector == ESP_SDR_DETECTOR_PEAK, c->dwell, &lo, &first, &bins);
        if (result != ESP_OK) {
            break;
        }
        const double start_hz = (double)lo * 1e6 + first * s.bin_hz;
        if (c->flatten) {
            flatten(s.codes, bins, first);
        }
        uint8_t spurs = c->suppress_spurs ? suppress_spurs(s.codes, bins, start_hz, s.bin_hz) : 0;
        const esp_sdr_scan_segment_t segment = {
            .step = step, .steps = s.plan.steps, .lo_mhz = lo, .start_hz = (uint32_t)(start_hz + 0.5),
            .bin_millihz = (uint32_t)(s.bin_hz * 1000.0 + 0.5), .bins = bins, .gain_index = c->gain_index,
            .ffts = (uint8_t)s.blocks, .spurs = spurs, .elapsed_us = (uint32_t)(esp_timer_get_time() - t0),
            .codes = s.codes,
        };
        if (!callback(&segment, ctx)) {
            result = ESP_ERR_INVALID_STATE;
        }
    }
    free(s.power);
    free(s.capture);
    free(s.codes);
    rf_set_gain(saved_manual, saved_index);
    rf_set_frequency(saved_mhz);
    return result;
}
