/* Public API: receiver setup, tuning, gain/filter, burst capture. */
#include "esp_sdr.h"

#include "bandwidth.h"
#include "burst.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "iq_pack.h"
#include "mode.h"
#include "nvs_flash.h"
#include "rf.h"
#include "spectrum.h"
#include "stream.h"
#include "worker.h"

#define TAG "esp_sdr"
#define INIT_CHANNEL 1

static esp_sdr_mode_t mode = ESP_SDR_MODE_RX;
static bool initialised;

esp_sdr_mode_t sdr_mode(void) { return initialised ? mode : (esp_sdr_mode_t)-1; }
esp_sdr_mode_t esp_sdr_get_mode(void) { return mode; }

#define RX_ONLY()                               \
    do {                                        \
        if (sdr_mode() != ESP_SDR_MODE_RX) {    \
            return ESP_ERR_INVALID_STATE;       \
        }                                       \
    } while (0)

uint32_t esp_sdr_rate_hz(esp_sdr_rate_t rate)
{
    switch (rate) {
    case ESP_SDR_RATE_80M: return 80000000u;
    case ESP_SDR_RATE_40M: return 40000000u;
    case ESP_SDR_RATE_16M: return 16000000u;
    }
    return 0;
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase");
        err = nvs_flash_init();
    }
    return err;
}

/* The Wi-Fi driver owns PHY initialisation and calibration. Bring it up in
 * NULL mode (no networking) with power save off, and park it on a channel. */
static esp_err_t init_phy(void)
{
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&config), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_NULL), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "power save");
    ESP_RETURN_ON_ERROR(esp_wifi_set_promiscuous(true), TAG, "promiscuous");
    return esp_wifi_set_channel(INIT_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

esp_err_t esp_sdr_init(esp_sdr_mode_t m)
{
    if (initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (m != ESP_SDR_MODE_RX && m != ESP_SDR_MODE_TX) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(init_nvs(), TAG, "nvs");
    ESP_RETURN_ON_ERROR(init_phy(), TAG, "phy");
    mode = m;
    initialised = true;
    if (m == ESP_SDR_MODE_TX) {
        return ESP_OK;  /* no receive path, no second core, no ring */
    }
    rf_prepare_rx();
    if (spectrum_init()) {
        worker_start();  /* without core 1, streams run single-core */
    }
    return ESP_OK;
}

esp_err_t esp_sdr_set_frequency(uint32_t mhz)
{
    RX_ONLY();
    if (mhz < ESP_SDR_MIN_MHZ || mhz > ESP_SDR_MAX_MHZ) {
        return ESP_ERR_INVALID_ARG;
    }
    rf_set_frequency(mhz);
    return ESP_OK;
}

uint32_t esp_sdr_get_frequency(void) { return rf_frequency(); }
void esp_sdr_set_crystal_ppb(int32_t ppb) { rf_set_crystal_ppb(ppb); }
int32_t esp_sdr_get_crystal_ppb(void) { return rf_crystal_ppb(); }
void esp_sdr_set_pll_offset(int32_t khz)
{
    if (sdr_mode() == ESP_SDR_MODE_RX) {
        rf_set_pll_offset(khz);
    }
}

esp_err_t esp_sdr_set_gain(bool manual, uint8_t index)
{
    RX_ONLY();
    if (manual && index > rf_gain_max()) {
        return ESP_ERR_INVALID_ARG;
    }
    rf_set_gain(manual, index);
    return ESP_OK;
}

void esp_sdr_get_gain(esp_sdr_gain_t *gain)
{
    gain->manual = rf_gain_manual();
    gain->index = rf_gain_index();
    gain->max_index = rf_gain_max();
    gain->forced = rf_gain_forced();
}

esp_err_t esp_sdr_set_filter(int code)
{
    RX_ONLY();
    if (code < -1 || code > 63) {
        return ESP_ERR_INVALID_ARG;
    }
    rf_set_filter(code);
    return ESP_OK;
}

esp_err_t esp_sdr_set_bandwidth(uint32_t mhz)
{
    RX_ONLY();
    if (mhz && (mhz < ESP_SDR_MIN_BANDWIDTH_MHZ || mhz > ESP_SDR_MAX_BANDWIDTH_MHZ)) {
        return ESP_ERR_INVALID_ARG;
    }
    rf_set_filter(bandwidth_filter_code(mhz));
    return ESP_OK;
}

void esp_sdr_get_filter(esp_sdr_filter_t *filter)
{
    filter->code = (int8_t)rf_filter();
    rf_filter_active(&filter->active_i, &filter->active_q);
}

esp_err_t esp_sdr_capture(esp_sdr_rate_t rate, uint32_t samples, const uint32_t **words, uint32_t *elapsed_us)
{
    RX_ONLY();
    if (rate > ESP_SDR_RATE_16M || samples < ESP_SDR_MIN_BURST_SAMPLES || samples > ESP_SDR_MAX_BURST_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }
    rf_prepare_rx();
    rf_filter_apply();
    esp_err_t err = burst_capture(rate, samples, elapsed_us);
    rf_filter_restore();
    *words = burst_buffer();
    return err;
}

size_t esp_sdr_pack_iq10(uint32_t *words, uint32_t samples) { return iq_pack10(words, samples); }
size_t esp_sdr_pack_iq8(uint32_t *words, uint32_t samples) { return iq_pack8(words, samples); }

void esp_sdr_run(const esp_sdr_run_config_t *config, const esp_sdr_sink_t *sink, esp_sdr_run_result_t *result)
{
    if (sdr_mode() != ESP_SDR_MODE_RX) {
        *result = (esp_sdr_run_result_t){.status = ESP_SDR_RUN_FAIL_ARG};
        return;
    }
    rf_prepare_rx();
    rf_filter_apply();
    stream_run(config, sink, result);
    rf_filter_restore();
}
