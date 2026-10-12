/*
 * Thin C++ view of esp_sdr.h: same calls, scoped names, no exceptions, no heap.
 *
 *   esp_sdr::Radio radio;
 *   ESP_ERROR_CHECK(radio.begin());
 *   radio.tune(2440);
 *   esp_sdr::Capture c = radio.capture(esp_sdr::Rate::Msps16, 4096);
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include "esp_sdr.h"

namespace esp_sdr {

enum class Mode : uint8_t { Rx = ESP_SDR_MODE_RX, Tx = ESP_SDR_MODE_TX };
enum class Rate : uint8_t { Msps80 = ESP_SDR_RATE_80M, Msps40 = ESP_SDR_RATE_40M, Msps16 = ESP_SDR_RATE_16M };
enum class Workers : uint8_t {
    Single = ESP_SDR_WORKERS_SINGLE,
    DualAssist = ESP_SDR_WORKERS_DUAL_ASSIST,
    Dual = ESP_SDR_WORKERS_DUAL,
};

/** Result of a burst capture: raw words valid until the next capture or stream. */
struct Capture {
    esp_err_t error = ESP_FAIL;
    const uint32_t *words = nullptr;
    uint32_t samples = 0;
    uint32_t elapsed_us = 0;

    explicit operator bool() const { return error == ESP_OK; }
    int16_t i(uint32_t n) const { return esp_sdr_word_i(words[n]); }
    int16_t q(uint32_t n) const { return esp_sdr_word_q(words[n]); }
};

using RunConfig = esp_sdr_run_config_t;
using RunResult = esp_sdr_run_result_t;
using Sink = esp_sdr_sink_t;
using Gain = esp_sdr_gain_t;
using Filter = esp_sdr_filter_t;

class Radio {
public:
    esp_err_t begin(Mode mode = Mode::Rx) { return esp_sdr_init(static_cast<esp_sdr_mode_t>(mode)); }

    esp_err_t tune(uint32_t mhz) { return esp_sdr_set_frequency(mhz); }
    uint32_t frequency() const { return esp_sdr_get_frequency(); }
    void set_pll_offset(int32_t khz) { esp_sdr_set_pll_offset(khz); }

    esp_err_t set_gain(uint8_t index) { return esp_sdr_set_gain(true, index); }
    esp_err_t set_auto_gain() { return esp_sdr_set_gain(false, 0); }
    Gain gain() const
    {
        Gain g{};
        esp_sdr_get_gain(&g);
        return g;
    }

    esp_err_t set_bandwidth(uint32_t mhz) { return esp_sdr_set_bandwidth(mhz); }
    esp_err_t set_filter(int code) { return esp_sdr_set_filter(code); }
    Filter filter() const
    {
        Filter f{};
        esp_sdr_get_filter(&f);
        return f;
    }

    Capture capture(Rate rate, uint32_t samples)
    {
        Capture c;
        c.samples = samples;
        c.error = esp_sdr_capture(static_cast<esp_sdr_rate_t>(rate), samples, &c.words, &c.elapsed_us);
        return c;
    }

    RunResult run(const RunConfig &config, const Sink *sink = nullptr)
    {
        RunResult r{};
        esp_sdr_run(&config, sink, &r);
        return r;
    }

    void set_workers(Workers w) { esp_sdr_set_workers(static_cast<esp_sdr_workers_t>(w)); }
    bool core1_alive() const { return esp_sdr_core1_alive(); }
    void set_dc_mode(uint8_t mode) { esp_sdr_set_dc_mode(mode); }

    static uint32_t rate_hz(Rate rate) { return esp_sdr_rate_hz(static_cast<esp_sdr_rate_t>(rate)); }
};

}  // namespace esp_sdr
