/*
 * Host requests -> esp_sdr calls -> reply frames (protocol in esp_sdr_protocol.h).
 * Every request is answered: ACK or a typed reply, and every capture or
 * stream ends with RUN_DONE.
 */
#include "requests.h"

#include "esp_heap_caps.h"
#include "esp_sdr.h"
#include "esp_sdr_link.h"
#include "esp_sdr_tx.h"
#include "esp_timer.h"
#include "frame.h"
#include "link.h"
#include "protocol.h"
#include "firmware_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_control.h"
#include "mode_switch.h"
#include "soc/soc_caps.h"
#include <string.h>

#define MAX_DURATION_MS 86400000u
#define MAX_REPEATS 1000u
#define MIN_UART_BAUD 115200u
#define MAX_UART_BAUD 3000000u
#define IQ_AUTO_SHIFT 255u
#define FIR_OUTPUT_BITS 15u
/* Measured USB-Serial-JTAG ceiling is ~800 KB/s; keep IQ streams gap-free below it. */
#define LINK_STREAM_BYTES_PER_S 720000u

typedef struct {
    uint8_t seq;
    uint8_t type;
} reply_to_t;

static void ack(reply_to_t to, uint8_t error)
{
    msg_ack_t m = {.error = error, .request = to.type};
    link_send_frame(RSP_ACK, to.seq, &m, sizeof(m));
}

static uint8_t error_of(esp_err_t err)
{
    switch (err) {
    case ESP_OK: return WIRE_ERR_NONE;
    case ESP_ERR_INVALID_ARG: return WIRE_ERR_BAD_ARGUMENT;
    case ESP_ERR_TIMEOUT: return WIRE_ERR_TIMEOUT;
    case ESP_ERR_NO_MEM: return WIRE_ERR_NO_MEMORY;
    case ESP_ERR_NOT_SUPPORTED: return WIRE_ERR_UNSUPPORTED;
    case ESP_ERR_INVALID_STATE: return WIRE_ERR_WRONG_MODE;
    default: return WIRE_ERR_HARDWARE;
    }
}

static void run_done(reply_to_t to, const esp_sdr_run_result_t *r, uint8_t error)
{
    msg_run_done_t m = {.error = error};
    if (r) {
        m.status = (uint8_t)r->status;
        m.stopped_by_host = r->stopped;
        m.detail = r->detail;
        m.units = r->units;
        m.late_max = r->late_max;
        m.pairs = r->pairs;
        m.elapsed_us = r->elapsed_us;
        m.work_max = r->work_max;
        m.frames = r->frames;
        m.drops = r->drops;
        m.abandoned = r->abandoned;
        m.ffts = r->ffts;
        m.lost_pairs = r->lost_pairs;
    }
    link_send_frame(RSP_RUN_DONE, to.seq, &m, sizeof(m));
}

/* ---- identity and link -------------------------------------------------- */

static void get_info(reply_to_t to)
{
    msg_info_t m = {
        .protocol = PROTOCOL_VERSION,
        .min_mhz = ESP_SDR_MIN_MHZ,
        .max_mhz = ESP_SDR_MAX_MHZ,
        .bw_min_mhz = ESP_SDR_MIN_BANDWIDTH_MHZ,
        .bw_max_mhz = ESP_SDR_MAX_BANDWIDTH_MHZ,
        .rates = (1u << ESP_SDR_RATE_80M) | (1u << ESP_SDR_RATE_40M) | (1u << ESP_SDR_RATE_16M),
        .max_samples = ESP_SDR_MAX_BURST_SAMPLES,
        .max_nfft = ESP_SDR_MAX_NFFT,
        .mode = (uint8_t)esp_sdr_get_mode(),
        .region = (uint8_t)esp_sdr_tx_get_region(),
    };
    esp_sdr_gain_t gain;
    esp_sdr_get_gain(&gain);
    m.gain_max = gain.max_index;
    static const char version[] = FIRMWARE_VERSION_JSON;
    link_send_frame_split(RSP_INFO, to.seq, &m, sizeof(m), version, sizeof(version) - 1);
}

static void set_baud(reply_to_t to, const msg_set_baud_t *q)
{
    if (!link_uart_active()) {
        ack(to, WIRE_ERR_UNSUPPORTED);
        return;
    }
    if (q->baud < MIN_UART_BAUD || q->baud > MAX_UART_BAUD) {
        ack(to, WIRE_ERR_BAD_ARGUMENT);
        return;
    }
    ack(to, WIRE_ERR_NONE);  /* at the old rate */
    link_set_uart_baud(q->baud);
}

/* ---- receiver settings -------------------------------------------------- */

static void get_gain(reply_to_t to)
{
    esp_sdr_gain_t g;
    esp_sdr_get_gain(&g);
    msg_gain_t m = {.manual = g.manual, .index = g.index, .max_index = g.max_index, .forced = g.forced};
    link_send_frame(RSP_GAIN, to.seq, &m, sizeof(m));
}

static void get_filter(reply_to_t to)
{
    esp_sdr_filter_t f;
    esp_sdr_get_filter(&f);
    msg_filter_t m = {.code = f.code, .active_i = f.active_i, .active_q = f.active_q};
    link_send_frame(RSP_FILTER, to.seq, &m, sizeof(m));
}

static void get_workers(reply_to_t to)
{
    msg_workers_t m = {.mode = (uint8_t)esp_sdr_get_workers(), .core1_alive = esp_sdr_core1_alive(),
                               .assist_blocks = esp_sdr_assist_blocks()};
    link_send_frame(RSP_WORKERS, to.seq, &m, sizeof(m));
}

static void get_memory(reply_to_t to)
{
    msg_memory_t m = {
        .heap_free = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT),
        .heap_min_free = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
        .banks = ESP_SDR_RING_BANKS,
        .threshold = ESP_SDR_RING_THRESHOLD,
    };
    link_send_frame(RSP_MEMORY, to.seq, &m, sizeof(m));
}

/* ---- burst capture ------------------------------------------------------ */

static void capture(reply_to_t to, const msg_capture_t *q)
{
    if (q->rate > ESP_SDR_RATE_16M || q->format > WIRE_FORMAT_IQ8 || !q->repeats ||
        q->repeats > MAX_REPEATS || q->samples < ESP_SDR_MIN_BURST_SAMPLES ||
        q->samples > ESP_SDR_MAX_BURST_SAMPLES) {
        run_done(to, NULL, WIRE_ERR_BAD_ARGUMENT);
        return;
    }
    for (uint32_t i = 0; i < q->repeats; i++) {
        const uint32_t *words;
        uint32_t elapsed_us;
        esp_err_t err = esp_sdr_capture((esp_sdr_rate_t)q->rate, q->samples, &words, &elapsed_us);
        if (err != ESP_OK) {
            run_done(to, NULL, error_of(err));
            return;
        }
        uint32_t *buffer = (uint32_t *)words;  /* the dump window; packed in place */
        size_t bytes = q->format == WIRE_FORMAT_IQ10 ? esp_sdr_pack_iq10(buffer, q->samples)
                     : q->format == WIRE_FORMAT_IQ8  ? esp_sdr_pack_iq8(buffer, q->samples)
                                                             : (size_t)q->samples * 4u;
        msg_data_t m = {.samples = q->samples, .elapsed_us = elapsed_us, .format = q->format,
                                .rate = q->rate};
        if (!link_send_frame_split(RSP_DATA, to.seq, &m, sizeof(m), buffer, bytes)) {
            return;  /* host gone: nothing more can be delivered */
        }
        if (i + 1 < q->repeats) {
            vTaskDelay(1);
        }
    }
    run_done(to, NULL, WIRE_ERR_NONE);
}

/* ---- streams ------------------------------------------------------------ */

static void send_ring_data(reply_to_t to, const esp_sdr_run_result_t *r, esp_sdr_rate_t rate)
{
    uint32_t counts[ESP_SDR_RING_BANKS] = {0};
    const uint32_t *span[2 * ESP_SDR_RING_BANKS];
    uint32_t span_words[2 * ESP_SDR_RING_BANKS];
    uint32_t total = 0;
    for (unsigned u = 0; u < r->units; u++) {
        esp_sdr_run_unit(r, u, &span[2 * u], &span_words[2 * u], &span[2 * u + 1], &span_words[2 * u + 1]);
        counts[u] = r->unit[u].count;
        total += r->unit[u].count;
    }
    msg_ring_data_t m = {.rate_hz = esp_sdr_rate_hz(rate), .units = r->units,
                                 .count0 = counts[0], .count1 = counts[1], .count2 = counts[2]};
    /* Header, message, then the spans back to back, one CRC over all of it. */
    uint8_t header[ESP_SDR_RECORD_HEADER_SIZE];
    uint32_t crc = frame_write_header(header, RSP_RING_DATA, to.seq, (uint32_t)sizeof(m) + total * 4u);
    crc = frame_crc32(crc, &m, sizeof(m));
    for (unsigned s = 0; s < 2 * r->units; s++) {
        crc = frame_crc32(crc, span[s], span_words[s] * 4u);
    }
    bool ok = link_send(header, sizeof(header)) && link_send(&m, sizeof(m));
    for (unsigned s = 0; ok && s < 2 * r->units; s++) {
        ok = link_send(span[s], span_words[s] * 4u);
    }
    if (ok) {
        link_send(&crc, sizeof(crc));
    }
}

static bool valid_duration(uint32_t ms)
{
    /* Open-ended runs need a stop byte from the host; only USB reads it reliably while masked. */
    return ms <= MAX_DURATION_MS && (ms || !link_uart_active());
}

static void run_stream(reply_to_t to, esp_sdr_run_config_t *c)
{
    esp_sdr_run_result_t r;
    c->tag = to.seq;
    esp_sdr_run(c, link_stream_sink(), &r);
    link_discard_input();  /* the stop request must not reach the parser */
    if (c->mode == ESP_SDR_RUN_CAPTURE && r.status == ESP_SDR_RUN_OK) {
        send_ring_data(to, &r, c->rate);
    }
    run_done(to, &r, WIRE_ERR_NONE);
}

static void ring_stats(reply_to_t to, const msg_ring_stats_t *q)
{
    if (q->rate > ESP_SDR_RATE_16M || !valid_duration(q->duration_ms)) {
        run_done(to, NULL, WIRE_ERR_BAD_ARGUMENT);
        return;
    }
    esp_sdr_run_config_t c = {.mode = ESP_SDR_RUN_STATS, .rate = (esp_sdr_rate_t)q->rate,
                              .duration_ms = q->duration_ms};
    run_stream(to, &c);
}

static void ring_capture(reply_to_t to, const msg_ring_capture_t *q)
{
    if (q->rate > ESP_SDR_RATE_16M || !q->units || q->units > ESP_SDR_RING_BANKS) {
        run_done(to, NULL, WIRE_ERR_BAD_ARGUMENT);
        return;
    }
    esp_sdr_run_config_t c = {.mode = ESP_SDR_RUN_CAPTURE, .rate = (esp_sdr_rate_t)q->rate,
                              .capture_units = q->units};
    run_stream(to, &c);
}

static void spectrum(reply_to_t to, const msg_spectrum_t *q)
{
    if (link_uart_active()) {
        run_done(to, NULL, WIRE_ERR_UNSUPPORTED);  /* frames need USB throughput */
        return;
    }
    if (q->rate > ESP_SDR_RATE_16M || q->max_hold > 1 || q->stats > 1 || !valid_duration(q->duration_ms) ||
        q->nfft < ESP_SDR_MIN_NFFT || q->nfft > ESP_SDR_MAX_NFFT || (q->nfft & (q->nfft - 1))) {
        run_done(to, NULL, WIRE_ERR_BAD_ARGUMENT);
        return;
    }
    esp_sdr_run_config_t c = {.mode = ESP_SDR_RUN_SPECTRUM, .rate = (esp_sdr_rate_t)q->rate,
                              .duration_ms = q->duration_ms, .nfft = q->nfft, .max_hold = q->max_hold,
                              .stats = q->stats};
    run_stream(to, &c);
}

static void iq_stream(reply_to_t to, const msg_iq_stream_t *q)
{
    if (link_uart_active()) {
        run_done(to, NULL, WIRE_ERR_UNSUPPORTED);
        return;
    }
    if (q->rate != ESP_SDR_RATE_16M || q->rotate > 1 || !valid_duration(q->duration_ms)) {
        run_done(to, NULL, WIRE_ERR_BAD_ARGUMENT);  /* IQ streams take 16 Msps input only */
        return;
    }
    if (q->decimation && q->bits) {
        uint32_t bytes_per_pair = q->bits == 16 ? 4u : q->bits == 8 ? 2u : 1u;
        if (16000000u / q->decimation * bytes_per_pair > LINK_STREAM_BYTES_PER_S) {
            run_done(to, NULL, WIRE_ERR_UNSUPPORTED);  /* more than the link can carry */
            return;
        }
    }
    /* Default shift scales the 15-bit FIR output to the requested width. */
    uint8_t shift = q->shift != IQ_AUTO_SHIFT ? q->shift : q->bits < FIR_OUTPUT_BITS ? FIR_OUTPUT_BITS - q->bits : 0;
    esp_sdr_run_config_t c = {.mode = ESP_SDR_RUN_IQ, .rate = (esp_sdr_rate_t)q->rate,
                              .duration_ms = q->duration_ms, .iq_decimation = q->decimation, .iq_bits = q->bits,
                              .iq_shift = shift, .iq_rotate = q->rotate};
    run_stream(to, &c);  /* the library validates decimation/bits/shift (RUN_FAIL_ARG) */
}

/* ---- full-band scan ----------------------------------------------------- */

typedef struct {
    reply_to_t to;
    uint32_t sweep;
} scan_context_t;

static bool send_segment(const esp_sdr_scan_segment_t *seg, void *ctx)
{
    scan_context_t *s = ctx;
    msg_scan_segment_t m = {
        .sweep = s->sweep, .step = (uint16_t)seg->step, .steps = (uint16_t)seg->steps,
        .lo_mhz = (uint16_t)seg->lo_mhz, .bins = seg->bins, .start_hz = seg->start_hz,
        .bin_millihz = seg->bin_millihz, .gain = seg->gain_index, .ffts = seg->ffts, .db_step = 2,
        .spurs = seg->spurs,
        .elapsed_us = seg->elapsed_us,
    };
    return link_send_frame_split(RSP_SCAN_SEGMENT, s->to.seq, &m, sizeof(m), seg->codes, seg->bins) &&
           !link_stop_requested();
}

static void scan(reply_to_t to, const msg_scan_t *q)
{
    if (q->rate > ESP_SDR_RATE_16M || !q->averages || q->detector > 1 || q->flags > 3) {
        run_done(to, NULL, WIRE_ERR_BAD_ARGUMENT);
        return;
    }
    const esp_sdr_scan_config_t c = {.from_mhz = q->from_mhz, .to_mhz = q->to_mhz, .rate = (esp_sdr_rate_t)q->rate,
                                     .nfft = q->nfft, .averages = q->averages, .gain_index = q->gain,
                                     .detector = (esp_sdr_detector_t)q->detector, .dwell = q->dwell ? q->dwell : 1,
                                     .flatten = q->flags & 1, .suppress_spurs = (q->flags & 2) != 0};
    scan_context_t ctx = {.to = to};
    esp_err_t err = ESP_OK;
    bool stopped = false;
    for (; !q->sweeps || ctx.sweep < q->sweeps; ctx.sweep++) {
        err = esp_sdr_scan(&c, send_segment, &ctx);
        if (err == ESP_ERR_INVALID_STATE) {  /* aborted by the host */
            stopped = true;
            err = ESP_OK;
            break;
        }
        if (err != ESP_OK) {
            break;
        }
    }
    if (stopped) {
        link_discard_input();
    }
    esp_sdr_run_result_t r = {.units = ctx.sweep, .stopped = stopped};  /* units = completed sweeps */
    run_done(to, &r, error_of(err));
}

/* ---- transmitter ------------------------------------------------------- */

static void tx_bits(reply_to_t to, const msg_tx_bits_t *q, const uint8_t *body, size_t body_len)
{
    if (!q->nbits || (q->nbits + 7u) / 8u != body_len || q->gaussian > 1) {
        ack(to, WIRE_ERR_BAD_ARGUMENT);
        return;
    }
    const esp_sdr_fsk_t fsk = {.center_khz = q->center_khz, .bitrate = q->bitrate, .deviation_hz = q->deviation_hz,
                               .gaussian = q->gaussian, .atten = q->atten};
    ack(to, error_of(esp_sdr_tx_bits(&fsk, body, q->nbits, q->count, q->gap_us)));
}

static void tx_ble(reply_to_t to, const msg_tx_ble_t *q, const uint8_t *body, size_t body_len)
{
    const uint8_t adv[6] = {q->adv0, q->adv1, q->adv2, q->adv3, q->adv4, q->adv5};
    ack(to, error_of(esp_sdr_tx_ble_adv(q->channel, adv, body, body_len, q->atten, q->count, q->gap_us)));
}

static void tx_nrf24(reply_to_t to, const msg_tx_nrf24_t *q, const uint8_t *body, size_t body_len)
{
    const esp_sdr_nrf24_t c = {.addr = {q->addr0, q->addr1, q->addr2, q->addr3, q->addr4}, .addr_len = q->addr_len,
                               .crc_len = q->crc_len, .channel = q->channel, .bitrate = q->bitrate,
                               .atten = q->atten};
    ack(to, error_of(esp_sdr_tx_nrf24(&c, body, body_len, q->count, q->gap_us)));
}

/* ---- two-way link ----------------------------------------------------- */

#define LINK_FROM(q)                                                                                        \
    {.channel_khz = (q)->channel_khz, .bitrate = (q)->bitrate, .addr = {(q)->addr0, (q)->addr1, (q)->addr2,  \
     (q)->addr3, (q)->addr4}, .addr_len = (q)->addr_len, .crc_len = (q)->crc_len, .atten = (q)->atten}
#define LINK_RESPOND_SLICE_MS 50u  /* interrupts come back between slices */

static void link_exchange(reply_to_t to, const msg_link_exchange_t *q, const uint8_t *body, size_t body_len)
{
    const esp_sdr_link_t l = LINK_FROM(q);
    uint8_t rx[ESP_SDR_LINK_MAX_PAYLOAD] = {0};
    esp_sdr_link_info_t info = {0};
    esp_err_t err = q->rx_len > sizeof(rx) ? ESP_ERR_INVALID_ARG
                  : esp_sdr_link_exchange(&l, body, body_len, rx, q->rx_len, q->timeout_us, &info);
    msg_link_result_t m = {.error = error_of(err), .len = err == ESP_OK ? q->rx_len : 0, .delay_ns = info.delay_ns,
                           .crc_failures = info.crc_failures, .syncs = info.syncs,
                           .edge_errors = info.edge_errors};
    link_send_frame_split(RSP_LINK_RESULT, to.seq, &m, sizeof(m), rx, m.len);
}

static void link_respond(reply_to_t to, const msg_link_respond_t *q, const uint8_t *body, size_t body_len)
{
    const esp_sdr_link_t l = LINK_FROM(q);
    uint8_t rx[ESP_SDR_LINK_MAX_PAYLOAD] = {0};
    msg_link_summary_t m = {.delay_min_ns = UINT32_MAX};
    esp_err_t err = q->rx_len > sizeof(rx) ? ESP_ERR_INVALID_ARG : ESP_OK;
    int64_t end = esp_timer_get_time() + (int64_t)q->duration_ms * 1000;
    while (err == ESP_OK && m.served < q->count && esp_timer_get_time() < end && !link_stop_requested()) {
        esp_sdr_link_info_t info = {0};
        esp_err_t e = esp_sdr_link_respond(&l, rx, q->rx_len, body, body_len, q->turnaround_us,
                                           LINK_RESPOND_SLICE_MS, &info);
        m.crc_failures += info.crc_failures;
        m.syncs += info.syncs;
        m.edge_errors += info.edge_errors;
        if (e == ESP_OK) {
            m.served++;
            m.len = q->rx_len;
            m.delay_sum_us += info.delay_ns / 1000u;
            m.delay_min_ns = info.delay_ns < m.delay_min_ns ? info.delay_ns : m.delay_min_ns;
            m.delay_max_ns = info.delay_ns > m.delay_max_ns ? info.delay_ns : m.delay_max_ns;
        } else if (e != ESP_ERR_TIMEOUT) {
            err = e;
        }
    }
    link_discard_input();
    m.error = error_of(err);
    link_send_frame_split(RSP_LINK_SUMMARY, to.seq, &m, sizeof(m), rx, m.len);
}

/* ---- GPIO --------------------------------------------------------------- */

static void get_gpio(reply_to_t to)
{
    uint8_t pairs[2 * SOC_GPIO_PIN_COUNT];
    size_t n = gpio_control_list(pairs, SOC_GPIO_PIN_COUNT);
    link_send_frame(RSP_GPIO, to.seq, pairs, 2 * n);
}

/* ---- dispatch ----------------------------------------------------------- */

#define PAYLOAD(type) ((const type *)request->payload)
#define BODY(type) (request->payload + sizeof(type)), (request->length - sizeof(type))
#define NEEDS_AT_LEAST(type)                          \
    do {                                              \
        if (request->length < sizeof(type)) {         \
            ack(to, WIRE_ERR_BAD_LENGTH);     \
            return;                                   \
        }                                             \
    } while (0)
#define NEEDS(type)                                   \
    do {                                              \
        if (request->length != sizeof(type)) {        \
            ack(to, WIRE_ERR_BAD_LENGTH);     \
            return;                                   \
        }                                             \
    } while (0)

void requests_handle(const link_request_t *request)
{
    const reply_to_t to = {.seq = request->seq, .type = request->type};
    switch (request->type) {
    case REQ_GET_INFO:
        get_info(to);
        return;
    case REQ_RELEASE:
        link_release();
        ack(to, WIRE_ERR_NONE);
        return;
    case REQ_SET_BAUD:
        NEEDS(msg_set_baud_t);
        set_baud(to, PAYLOAD(msg_set_baud_t));
        return;
    case REQ_SET_MODE: {
        NEEDS(msg_set_mode_t);
        uint8_t mode = PAYLOAD(msg_set_mode_t)->mode;
        if (mode > WIRE_MODE_TXRX) {
            ack(to, WIRE_ERR_BAD_ARGUMENT);
            return;
        }
        ack(to, WIRE_ERR_NONE);
        mode_switch_restart((esp_sdr_mode_t)mode);  /* does not return */
        return;
    }
    case REQ_TX_REGION:
        NEEDS(msg_tx_region_t);
        ack(to, error_of(esp_sdr_tx_set_region((esp_sdr_region_t)PAYLOAD(msg_tx_region_t)->region)));
        return;
    case REQ_TX_TONE:
        NEEDS(msg_tx_tone_t);
        ack(to, error_of(esp_sdr_tx_tone(PAYLOAD(msg_tx_tone_t)->khz, PAYLOAD(msg_tx_tone_t)->atten)));
        return;
    case REQ_TX_OFF:
        ack(to, error_of(esp_sdr_tx_off()));
        return;
    case REQ_TX_BITS:
        NEEDS_AT_LEAST(msg_tx_bits_t);
        tx_bits(to, PAYLOAD(msg_tx_bits_t), BODY(msg_tx_bits_t));
        return;
    case REQ_TX_BLE:
        NEEDS_AT_LEAST(msg_tx_ble_t);
        tx_ble(to, PAYLOAD(msg_tx_ble_t), BODY(msg_tx_ble_t));
        return;
    case REQ_TX_NRF24:
        NEEDS_AT_LEAST(msg_tx_nrf24_t);
        tx_nrf24(to, PAYLOAD(msg_tx_nrf24_t), BODY(msg_tx_nrf24_t));
        return;
    case REQ_LINK_EXCHANGE:
        NEEDS_AT_LEAST(msg_link_exchange_t);
        link_exchange(to, PAYLOAD(msg_link_exchange_t), BODY(msg_link_exchange_t));
        return;
    case REQ_LINK_RESPOND:
        NEEDS_AT_LEAST(msg_link_respond_t);
        link_respond(to, PAYLOAD(msg_link_respond_t), BODY(msg_link_respond_t));
        return;
    case REQ_SET_FREQUENCY:
        NEEDS(msg_set_frequency_t);
        ack(to, error_of(esp_sdr_set_frequency(PAYLOAD(msg_set_frequency_t)->mhz)));
        return;
    case REQ_SET_CRYSTAL_PPB:
        NEEDS(msg_set_crystal_ppb_t);
        esp_sdr_set_crystal_ppb(PAYLOAD(msg_set_crystal_ppb_t)->ppb);
        ack(to, WIRE_ERR_NONE);
        return;
    case REQ_SET_PLL_OFFSET:
        NEEDS(msg_set_pll_offset_t);
        esp_sdr_set_pll_offset(PAYLOAD(msg_set_pll_offset_t)->khz);
        ack(to, WIRE_ERR_NONE);
        return;
    case REQ_SET_GAIN: {
        NEEDS(msg_set_gain_t);
        const msg_set_gain_t *q = PAYLOAD(msg_set_gain_t);
        ack(to, q->manual > 1 ? WIRE_ERR_BAD_ARGUMENT : error_of(esp_sdr_set_gain(q->manual, q->index)));
        return;
    }
    case REQ_GET_GAIN:
        get_gain(to);
        return;
    case REQ_SET_FILTER:
        NEEDS(msg_set_filter_t);
        ack(to, error_of(esp_sdr_set_filter(PAYLOAD(msg_set_filter_t)->code)));
        return;
    case REQ_SET_BANDWIDTH:
        NEEDS(msg_set_bandwidth_t);
        ack(to, error_of(esp_sdr_set_bandwidth(PAYLOAD(msg_set_bandwidth_t)->mhz)));
        return;
    case REQ_GET_FILTER:
        get_filter(to);
        return;
    case REQ_SET_DC_MODE: {
        NEEDS(msg_set_dc_mode_t);
        uint8_t mode = PAYLOAD(msg_set_dc_mode_t)->mode;
        if (mode > 1) {
            ack(to, WIRE_ERR_BAD_ARGUMENT);
            return;
        }
        esp_sdr_set_dc_mode(mode);
        ack(to, WIRE_ERR_NONE);
        return;
    }
    case REQ_SET_WORKERS: {
        NEEDS(msg_set_workers_t);
        uint8_t mode = PAYLOAD(msg_set_workers_t)->mode;
        if (mode > ESP_SDR_WORKERS_DUAL) {
            ack(to, WIRE_ERR_BAD_ARGUMENT);
            return;
        }
        esp_sdr_set_workers((esp_sdr_workers_t)mode);
        ack(to, WIRE_ERR_NONE);
        return;
    }
    case REQ_GET_WORKERS:
        get_workers(to);
        return;
    case REQ_GET_MEMORY:
        get_memory(to);
        return;
    case REQ_CAPTURE:
        NEEDS(msg_capture_t);
        capture(to, PAYLOAD(msg_capture_t));
        return;
    case REQ_RING_STATS:
        NEEDS(msg_ring_stats_t);
        ring_stats(to, PAYLOAD(msg_ring_stats_t));
        return;
    case REQ_RING_CAPTURE:
        NEEDS(msg_ring_capture_t);
        ring_capture(to, PAYLOAD(msg_ring_capture_t));
        return;
    case REQ_SPECTRUM:
        NEEDS(msg_spectrum_t);
        spectrum(to, PAYLOAD(msg_spectrum_t));
        return;
    case REQ_IQ_STREAM:
        NEEDS(msg_iq_stream_t);
        iq_stream(to, PAYLOAD(msg_iq_stream_t));
        return;
    case REQ_SCAN:
        NEEDS(msg_scan_t);
        scan(to, PAYLOAD(msg_scan_t));
        return;
    case REQ_STOP:
        ack(to, WIRE_ERR_NONE);  /* nothing running */
        return;
    case REQ_GET_GPIO:
        get_gpio(to);
        return;
    case REQ_SET_GPIO: {
        NEEDS(msg_set_gpio_t);
        const msg_set_gpio_t *q = PAYLOAD(msg_set_gpio_t);
        ack(to, error_of(gpio_control_set(q->pin, (gpio_control_state_t)q->state)));
        return;
    }
    default:
        ack(to, WIRE_ERR_UNKNOWN_COMMAND);
        return;
    }
}
