/* Full-band scan: tune, burst-capture, average FFTs, keep the clean slice. */
#include <stdlib.h>
#include "burst.h"
#include "db_code.h"
#include "esp_heap_caps.h"
#include "esp_sdr.h"
#include "esp_timer.h"
#include "rf.h"
#include "scan_plan.h"
#include "spectrum.h"


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

esp_err_t esp_sdr_scan(const esp_sdr_scan_config_t *c, esp_sdr_scan_cb_t callback, void *ctx)
{
    const uint32_t rate_hz = esp_sdr_rate_hz(c->rate);
    const scan_plan_t plan = scan_plan(c->from_mhz, c->to_mhz, rate_hz);
    if (!rate_hz || !callback || !spectrum_valid_nfft(c->nfft) || !c->averages ||
        !scan_plan_valid(&plan, c->from_mhz, ESP_SDR_MIN_MHZ, ESP_SDR_MAX_MHZ)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (c->gain_index != ESP_SDR_SCAN_GAIN_AGC && c->gain_index > rf_gain_max()) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Heap for the duration of one sweep only (internal SRAM is tight). */
    float *power = heap_caps_malloc(c->nfft * sizeof(float), MALLOC_CAP_INTERNAL);
    uint8_t *codes = heap_caps_malloc(c->nfft / 2, MALLOC_CAP_INTERNAL);
    if (!power || !codes) {
        free(power);
        free(codes);
        return ESP_ERR_NO_MEM;
    }
    uint32_t blocks = ESP_SDR_MAX_BURST_SAMPLES / c->nfft;
    if (blocks > c->averages) {
        blocks = c->averages;
    }
    const uint32_t samples = blocks * c->nfft < ESP_SDR_MIN_BURST_SAMPLES ? ESP_SDR_MIN_BURST_SAMPLES : blocks * c->nfft;
    const double bin_hz = (double)rate_hz / c->nfft;
    const uint32_t saved_mhz = rf_frequency();
    const bool saved_manual = rf_gain_manual();
    const uint8_t saved_index = rf_gain_index();
    rf_set_gain(c->gain_index != ESP_SDR_SCAN_GAIN_AGC, c->gain_index == ESP_SDR_SCAN_GAIN_AGC ? 0 : c->gain_index);

    esp_err_t result = ESP_OK;
    for (uint32_t step = 0; step < plan.steps && result == ESP_OK; step++) {
        const int64_t t0 = esp_timer_get_time();
        const uint32_t lo = scan_lo_mhz(&plan, step);
        const double seg_lo = ((double)c->from_mhz + (double)step * plan.keep_mhz) * 1e6;
        double seg_hi = seg_lo + plan.keep_mhz * 1e6;
        if (seg_hi > (double)c->to_mhz * 1e6) {
            seg_hi = (double)c->to_mhz * 1e6;
        }
        rf_set_frequency(lo);
        rf_filter_apply();
        uint32_t capture_us;
        esp_err_t err = burst_capture(c->rate, samples, &capture_us);
        rf_filter_restore();
        if (err != ESP_OK || !spectrum_burst_power(burst_buffer(), c->nfft, blocks, power)) {
            result = err != ESP_OK ? err : ESP_FAIL;
            break;
        }
        uint32_t first;
        uint16_t bins = slice(lo, bin_hz, c->nfft, seg_lo, seg_hi, &first);
        for (uint16_t i = 0; i < bins; i++) {
            union { float f; uint32_t u; } v = {.f = power[first + i]};
            codes[i] = db_code(v.u, 0);
        }
        const esp_sdr_scan_segment_t segment = {
            .step = step, .steps = plan.steps, .lo_mhz = lo,
            .start_hz = (uint32_t)((double)lo * 1e6 + first * bin_hz + 0.5),
            .bin_millihz = (uint32_t)(bin_hz * 1000.0 + 0.5), .bins = bins,
            .gain_index = c->gain_index, .ffts = (uint8_t)blocks,
            .elapsed_us = (uint32_t)(esp_timer_get_time() - t0), .codes = codes,
        };
        if (!callback(&segment, ctx)) {
            result = ESP_ERR_INVALID_STATE;
        }
    }
    free(power);
    free(codes);
    rf_set_gain(saved_manual, saved_index);
    rf_set_frequency(saved_mhz);
    return result;
}
