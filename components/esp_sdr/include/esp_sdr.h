/*
 * ESP-SDR: the ESP32-S3 2.4 GHz radio as a software-defined receiver.
 *
 * Radio only: the library never touches USB, UART or networking. Call
 * esp_sdr_init(ESP_SDR_MODE_RX) once, then tune, set gain/filter and capture. All calls are
 * for one task; they are not reentrant. Streams (esp_sdr_run) mask interrupts
 * on core 0 for their whole duration and hand their output records
 * (esp_sdr_record.h) to an application-provided sink.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_sdr_record.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Receive LO range verified on hardware. */
#define ESP_SDR_MIN_MHZ 1840u
#define ESP_SDR_MAX_MHZ 2790u
/** Largest burst capture, in IQ samples. */
#define ESP_SDR_MAX_BURST_SAMPLES 16380u
#define ESP_SDR_MIN_BURST_SAMPLES 256u
/** Spectrum FFT sizes: 256, 512, 1024, 2048. */
#define ESP_SDR_MIN_NFFT 256u
#define ESP_SDR_MAX_NFFT 2048u
/** Continuous ring geometry. */
#define ESP_SDR_RING_BANKS 3u
#define ESP_SDR_RING_PAIRS 16384u
#define ESP_SDR_RING_THRESHOLD 12288u
/** IQ stream: 16 Msps input only, decimation 32..1024 (power of two). Gap-free output is
 *  bounded by the ~800 KB/s USB-Serial-JTAG link: 500 kS/s at 4 bits, 250 kS/s at 8, 125 kS/s at 16. */
#define ESP_SDR_IQ_MIN_DECIMATION 32u
#define ESP_SDR_IQ_MAX_DECIMATION 1024u
/** Approximate two-sided baseband bandwidth range of the S3 filter, MHz. */
#define ESP_SDR_MIN_BANDWIDTH_MHZ 13u
#define ESP_SDR_MAX_BANDWIDTH_MHZ 69u

typedef enum {
    ESP_SDR_RATE_80M = 0,
    ESP_SDR_RATE_40M = 1,
    ESP_SDR_RATE_16M = 2,
} esp_sdr_rate_t;

typedef enum {
    ESP_SDR_WORKERS_SINGLE = 0,      /**< core 0 only */
    ESP_SDR_WORKERS_DUAL_ASSIST = 1, /**< core 1 processes, core 0 helps between bank switches (default) */
    ESP_SDR_WORKERS_DUAL = 2,        /**< core 1 processes alone */
} esp_sdr_workers_t;

typedef struct {
    bool manual;      /**< false: hardware AGC */
    uint8_t index;    /**< PHY gain index when manual */
    uint8_t max_index;/**< largest calibrated index at the current frequency */
    bool forced;      /**< PHY reports a forced gain */
} esp_sdr_gain_t;

typedef struct {
    int8_t code;      /**< requested filter code, -1 = PHY automatic */
    uint8_t active_i; /**< filter code currently in the I register */
    uint8_t active_q; /**< filter code currently in the Q register */
} esp_sdr_filter_t;

/**
 * One raw dump word: I in bits 0..9, Q in bits 10..19 (10-bit two's complement),
 * gain in bits 20..27. The hardware's raw Q is inverted (its spectrum is
 * mirrored, measured with a fixed carrier and a swept LO); these accessors
 * return the corrected sample I + jQ. Spectrum and IQ stream outputs are
 * already corrected.
 */
static inline int16_t esp_sdr_word_i(uint32_t w) { return (int16_t)((int32_t)(w << 22) >> 22); }
static inline int16_t esp_sdr_word_q(uint32_t w) { return (int16_t)-((int32_t)(w << 12) >> 22); }
/** Raw (uncorrected) Q, as stored by the hardware. */
static inline int16_t esp_sdr_word_raw_q(uint32_t w) { return (int16_t)((int32_t)(w << 12) >> 22); }

/* ---- setup ------------------------------------------------------------- */

typedef enum {
    ESP_SDR_MODE_RX = 0,  /**< receiver: capture, streams, scan */
    ESP_SDR_MODE_TX = 1,  /**< transmitter (esp_sdr_tx.h); receiver calls return ESP_ERR_INVALID_STATE */
} esp_sdr_mode_t;

/**
 * Bring up NVS and the Wi-Fi PHY (no networking) in one mode, once per boot.
 * RX also prepares the receiver and starts the second core. Calls of the
 * other mode return ESP_ERR_INVALID_STATE; to switch, restart the chip.
 */
esp_err_t esp_sdr_init(esp_sdr_mode_t mode);
esp_sdr_mode_t esp_sdr_get_mode(void);
uint32_t esp_sdr_rate_hz(esp_sdr_rate_t rate);

/* ---- receiver ---------------------------------------------------------- */

esp_err_t esp_sdr_set_frequency(uint32_t mhz);
uint32_t esp_sdr_get_frequency(void);
/**
 * Crystal calibration for both receive and transmit tuning: the board's
 * measured frequency error in parts per billion (+ = everything comes out
 * high). Applied from the next tune; esp_sdr_set_frequency() retunes now.
 * Transmitting heats the crystal by up to ~5 ppm/min (measured), so
 * calibrate in the duty cycle you will use.
 */
void esp_sdr_set_crystal_ppb(int32_t ppb);
int32_t esp_sdr_get_crystal_ppb(void);
/** Fine LO offset in kHz; retunes immediately. */
void esp_sdr_set_pll_offset(int32_t khz);
esp_err_t esp_sdr_set_gain(bool manual, uint8_t index);
void esp_sdr_get_gain(esp_sdr_gain_t *gain);
/** -1 = PHY automatic, else 0..63. */
esp_err_t esp_sdr_set_filter(int code);
/** 0 = widest, else ESP_SDR_MIN..MAX_BANDWIDTH_MHZ. */
esp_err_t esp_sdr_set_bandwidth(uint32_t mhz);
void esp_sdr_get_filter(esp_sdr_filter_t *filter);

/* ---- burst capture ----------------------------------------------------- */

/**
 * Capture `samples` raw words at `rate` into the radio's dump window.
 * On success *words points at them; valid until the next capture or stream.
 */
esp_err_t esp_sdr_capture(esp_sdr_rate_t rate, uint32_t samples, const uint32_t **words, uint32_t *elapsed_us);
/** Pack raw words in place: IQ10 = two 20-bit samples per 5 bytes; IQ8 = signed I, Q bytes. Returns bytes. */
size_t esp_sdr_pack_iq10(uint32_t *words, uint32_t samples);
size_t esp_sdr_pack_iq8(uint32_t *words, uint32_t samples);

/* ---- continuous streams ------------------------------------------------ */

typedef enum {
    ESP_SDR_RUN_STATS = 0,    /**< gapless run, counters only */
    ESP_SDR_RUN_CAPTURE = 1,  /**< gapless capture of 1..3 units, read with esp_sdr_run_unit() */
    ESP_SDR_RUN_SPECTRUM = 2, /**< on-chip FFT frames */
    ESP_SDR_RUN_IQ = 3,       /**< decimated IQ frames */
} esp_sdr_run_mode_t;

typedef enum {
    ESP_SDR_RUN_OK = 0,
    ESP_SDR_RUN_FAIL_ARG,     /**< invalid configuration (detail: which check; IQ: 2 args, 3/4 memory, 5 rate) */
    ESP_SDR_RUN_FAIL_LATE,    /**< bank switched too late (detail: pairs written) */
    ESP_SDR_RUN_FAIL_AGE,     /**< bank older than the ring allows (detail: cycles) */
    ESP_SDR_RUN_FAIL_START,   /**< unit did not start where the previous ended */
    ESP_SDR_RUN_FAIL_END,     /**< unit end not inside its sentinel window */
    ESP_SDR_RUN_FAIL_LENGTH,  /**< implausible unit length (detail: pairs) */
} esp_sdr_run_status_t;

typedef struct {
    esp_sdr_run_mode_t mode;
    esp_sdr_rate_t rate;
    uint32_t duration_ms;   /**< 0: until sink->stop_requested() (CAPTURE ignores it) */
    uint8_t tag;            /**< copied into every record header */
    uint8_t capture_units;  /**< CAPTURE: 1..ESP_SDR_RING_BANKS */
    uint16_t nfft;          /**< SPECTRUM */
    bool max_hold;          /**< SPECTRUM: per-bin maximum instead of mean */
    bool stats;             /**< SPECTRUM: emit STATS frames */
    uint16_t iq_decimation; /**< IQ: 32..1024, power of two; rate must be ESP_SDR_RATE_16M */
    uint8_t iq_bits;        /**< IQ: 4, 8 or 16 */
    uint8_t iq_shift;       /**< IQ: rounding right shift of the 15-bit FIR output, 0..24 */
    bool iq_rotate;         /**< IQ: shift by +fs/4 before filtering (LO tuned fs/4 below) */
} esp_sdr_run_config_t;

typedef struct {
    uint16_t bank;
    uint16_t first;
    uint32_t count;
} esp_sdr_ring_unit_t;

typedef struct {
    esp_sdr_run_status_t status;
    uint32_t detail;
    uint32_t units;
    uint64_t pairs;          /**< samples covered, gapless */
    uint64_t elapsed_us;
    uint32_t late_max;       /**< worst bank-switch lateness, pairs */
    uint32_t work_max;       /**< longest processing step, CPU cycles */
    uint32_t frames;
    uint32_t drops;          /**< output frames lost to a full queue */
    uint32_t abandoned;      /**< FFT blocks / IQ units not processed in time */
    uint32_t ffts;
    uint32_t lost_pairs;     /**< IQ: input pairs not filtered */
    bool stopped;            /**< ended by sink->stop_requested() or a sink that stopped draining */
    esp_sdr_ring_unit_t unit[ESP_SDR_RING_BANKS];
} esp_sdr_run_result_t;

/**
 * Where a stream's records go. Both callbacks run on core 0 with interrupts
 * masked, between deadline-critical ring polls: they must be in IRAM
 * (IRAM_ATTR), must not block, and must not use FreeRTOS or drivers.
 */
typedef struct {
    /** Take up to `size` bytes now; return how many were taken (0 is fine). */
    size_t (*write)(void *ctx, const uint8_t *data, size_t size);
    /** Optional: true to end the run (checked about once per bank switch). */
    bool (*stop_requested)(void *ctx);
    void *ctx;
} esp_sdr_sink_t;

/**
 * Run a stream to completion. Records go to `sink` (may be NULL for
 * STATS / CAPTURE). An open-ended run (duration 0) needs stop_requested;
 * a SPECTRUM run whose sink stops draining for 2 s also ends.
 */
void esp_sdr_run(const esp_sdr_run_config_t *config, const esp_sdr_sink_t *sink, esp_sdr_run_result_t *result);
/** CAPTURE results: unit u as up to two contiguous word spans (the ring may wrap). */
void esp_sdr_run_unit(const esp_sdr_run_result_t *result, unsigned u,
                      const uint32_t **head, uint32_t *head_count,
                      const uint32_t **tail, uint32_t *tail_count);

/* ---- full-band scan ---------------------------------------------------- */

#define ESP_SDR_SCAN_GAIN_AGC 255u

typedef enum {
    ESP_SDR_DETECTOR_MEAN = 0,  /**< average power: noise floor, steady signals */
    ESP_SDR_DETECTOR_PEAK = 1,  /**< per-bin maximum: catches short bursts (Wi-Fi, BLE) */
} esp_sdr_detector_t;

typedef struct {
    uint32_t from_mhz;      /**< >= ESP_SDR_MIN_MHZ + offset (1/2/4 MHz at 16/40/80 Msps) */
    uint32_t to_mhz;        /**< last step's LO must be <= ESP_SDR_MAX_MHZ */
    esp_sdr_rate_t rate;    /**< step width: 6 / 16 / 32 MHz at 16 / 40 / 80 Msps */
    uint16_t nfft;          /**< 256..2048: resolution = rate / nfft */
    uint8_t averages;       /**< FFTs averaged per step (capped by 16380 / nfft) */
    uint8_t gain_index;     /**< fixed gain for comparable steps; ESP_SDR_SCAN_GAIN_AGC = hardware AGC */
    esp_sdr_detector_t detector;
    uint8_t dwell;          /**< captures per step (>= 1); more dwell catches more bursts */
    bool flatten;           /**< remove the in-step filter ripple (self-calibrated from the noise floor) */
    bool suppress_spurs;    /**< interpolate over the receiver's own n x 8 MHz clock harmonics */
} esp_sdr_scan_config_t;

typedef struct {
    uint32_t step;          /**< 0 .. steps - 1 */
    uint32_t steps;
    uint32_t lo_mhz;
    uint32_t start_hz;      /**< absolute frequency of codes[0] */
    uint32_t bin_millihz;   /**< spacing of codes, mHz */
    uint16_t bins;
    uint8_t gain_index;
    uint8_t ffts;           /**< FFTs per capture */
    uint8_t spurs;          /**< bins interpolated over (suppress_spurs) */
    uint32_t elapsed_us;    /**< time spent on this step */
    const uint8_t *codes;   /**< 0.5 dB codes relative to full scale, ascending frequency */
} esp_sdr_scan_segment_t;

/** Return false to abort the scan. Called on the caller's task; may block. */
typedef bool (*esp_sdr_scan_cb_t)(const esp_sdr_scan_segment_t *segment, void *ctx);

/**
 * One sweep from from_mhz to to_mhz in LO steps; each step's slice above the
 * LO (clear of DC and filter edges) is delivered in ascending frequency order.
 * Restores frequency and gain afterwards. ESP_ERR_INVALID_ARG if the plan
 * leaves the verified LO range; ESP_ERR_INVALID_STATE if aborted.
 */
esp_err_t esp_sdr_scan(const esp_sdr_scan_config_t *config, esp_sdr_scan_cb_t callback, void *ctx);

void esp_sdr_set_workers(esp_sdr_workers_t workers);
esp_sdr_workers_t esp_sdr_get_workers(void);
bool esp_sdr_core1_alive(void);
/** Spectrum blocks core 0 handed to core 1 in the last run. */
uint32_t esp_sdr_assist_blocks(void);
/** 0 = notch bin 0, 1 = slow DC tracker (default). */
void esp_sdr_set_dc_mode(uint8_t mode);

#ifdef __cplusplus
}
#endif
