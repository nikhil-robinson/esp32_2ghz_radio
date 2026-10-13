/*
 * Two-way packet link: exchange (send, then catch the reply) and respond
 * (catch a packet, then answer after a fixed turnaround).
 *
 * One LO, LINK_IF_STEPS fine tone steps (3.984375 MHz) below the channel,
 * serves both directions: the transmitter reaches the channel through the
 * tone generator offset and the receiver hears it near fs/4.
 */
#include "esp_sdr_link.h"

#include <string.h>
#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"
#include "link_frame.h"
#include "link_rx.h"
#include "mode.h"
#include "packet.h"
#include "phy_private.h"
#include "rf.h"
#include "soc/soc.h"
#include "tx_internal.h"
#include "tx_plan.h"

#define LINK_IF_STEPS 102         /* x 39.0625 kHz = 3984.375 kHz */
#define LINK_IF_KHZ 3984u         /* LO below the channel (0.375 kHz rounding: TX lands 375 Hz high) */
#define CYCLES_PER_US 240u

typedef struct {
    link_rx_plan_t plan;
    link_rx_t rx;
    tx_job_t job;
    uint32_t on, idle;
} link_ctx_t;

static uint32_t deviation_hz(uint32_t bitrate) { return bitrate == 2000000u ? 320000u : 160000u; }

/* Validate, tune the shared LO, learn the tone word and build the TX table for `tx`. */
static esp_err_t setup(link_ctx_t *c, const esp_sdr_link_t *l, const uint8_t *tx, size_t tx_len, size_t rx_len)
{
    if (sdr_mode() != ESP_SDR_MODE_TXRX) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!l || !tx || !tx_len || tx_len > ESP_SDR_LINK_MAX_PAYLOAD || !tx_bitrate_supported(l->bitrate) ||
        !link_rx_plan(&c->plan, l->addr, l->addr_len, l->crc_len, rx_len)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t bits[PACKET_NRF24_MAX_BYTES];
    size_t nbits = packet_nrf24(bits, l->addr, l->addr_len, l->crc_len, tx, tx_len);
    esp_err_t err = tx_job_init(&c->job, l->bitrate, deviation_hz(l->bitrate), false, nbits);
    if (err == ESP_OK) {
        err = tx_check(l->channel_khz, tx_steps_khz(c->job.dev_steps));
    }
    if (err != ESP_OK) {
        tx_job_free(&c->job);
        return err;
    }
    rf_tune_rx_khz(l->channel_khz - LINK_IF_KHZ);  /* receive path prepared at the shared LO (cheap if unchanged) */
    /* The generator's register word (field 0, tone on) is learnt once: re-arming the
     * responder must not start and stop the generator every time. */
    static bool known;
    static uint32_t on_word;
    if (!known) {
        on_word = tx_tone_start(l->atten);
        REG_WRITE(PHY_TONE1_REG, on_word & ~PHY_TONE_ON);
        tx_tone_stop();
        rf_rx_resume();
        known = true;
    }
    c->on = on_word;
    c->idle = on_word & ~PHY_TONE_ON;
    tx_job_fill(&c->job, bits, nbits, c->on, LINK_IF_STEPS);
    link_rx_init(&c->rx, &c->plan, l->bitrate);
    return ESP_OK;
}

/* Cycles from our first bit to the end of our last bit (the table adds two tail bits). */
static uint32_t tx_cycles(const link_ctx_t *c) { return (uint32_t)(c->job.steps - 2u * c->job.sub) * c->job.cycles; }

/* Generator on (silent), packet, generator off, receiver back on. Returns the cycle count at our first bit. */
static IRAM_ATTR uint32_t transmit(link_ctx_t *c, uint8_t atten, uint32_t not_before)
{
    tx_tone_start(atten);
    REG_WRITE(PHY_TONE1_REG, c->idle);
    while ((int32_t)(esp_cpu_get_cycle_count() - not_before) < 0) {
    }
    uint32_t start = esp_cpu_get_cycle_count();
    send_once(c->job.table, c->job.steps, c->job.cycles, c->idle, c->job.fine);
    tx_tone_stop();
    rf_rx_resume();  /* ~27 us; a retune would take 0.7-0.9 ms (measured) */
    return start;
}

static uint32_t packet_cycles(const link_ctx_t *c, uint32_t bitrate)
{
    /* preamble + address + body, in CPU cycles */
    return (uint32_t)((8u + 8u * c->plan.addr_len + c->plan.body_bits) * (CYCLES_PER_US * 1000000ull / bitrate));
}

esp_err_t esp_sdr_link_exchange(const esp_sdr_link_t *link, const uint8_t *tx, size_t tx_len, uint8_t *rx,
                                size_t rx_len, uint32_t timeout_us, esp_sdr_link_info_t *info)
{
    link_ctx_t c;
    esp_err_t err = setup(&c, link, tx, tx_len, rx_len);
    if (err != ESP_OK) {
        return err;
    }
    esp_sdr_link_info_t local = {0};
    unsigned irq = portSET_INTERRUPT_MASK_FROM_ISR();
    uint32_t last_bit = transmit(&c, link->atten, esp_cpu_get_cycle_count()) + tx_cycles(&c);
    link_rx_start(&c.rx);
    uint32_t deadline = last_bit + timeout_us * CYCLES_PER_US;
    err = ESP_ERR_TIMEOUT;
    while (link_rx_run(&c.rx, deadline)) {
        if (link_rx_check(&c.plan, c.rx.body)) {
            uint32_t reply_start = c.rx.end_cycles - packet_cycles(&c, link->bitrate);
            local.delay_ns = (uint32_t)((int32_t)(reply_start - last_bit) * 1000 / (int32_t)CYCLES_PER_US);
            memcpy(rx, c.rx.body, rx_len);
            err = ESP_OK;
            break;
        }
        local.crc_failures++;
    }
    link_rx_stop(&c.rx);
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
    tx_job_free(&c.job);
    local.syncs = c.rx.syncs;
    local.edge_errors = c.rx.bad_edges;
    if (info) {
        *info = local;
    }
    return err;
}

esp_err_t esp_sdr_link_respond(const esp_sdr_link_t *link, uint8_t *rx, size_t rx_len, const uint8_t *reply,
                               size_t reply_len, uint32_t turnaround_us, uint32_t timeout_ms,
                               esp_sdr_link_info_t *info)
{
    link_ctx_t c;
    esp_err_t err = setup(&c, link, reply, reply_len, rx_len);
    if (err != ESP_OK) {
        return err;
    }
    esp_sdr_link_info_t local = {0};
    unsigned irq = portSET_INTERRUPT_MASK_FROM_ISR();
    link_rx_start(&c.rx);
    uint32_t deadline = esp_cpu_get_cycle_count() + timeout_ms * 1000u * CYCLES_PER_US;
    err = ESP_ERR_TIMEOUT;
    while (link_rx_run(&c.rx, deadline)) {
        if (!link_rx_check(&c.plan, c.rx.body)) {
            local.crc_failures++;
            continue;
        }
        link_rx_stop(&c.rx);
        uint32_t their_last = c.rx.end_cycles;
        uint32_t start_at = their_last + turnaround_us * CYCLES_PER_US;
        /* our first bit goes out when send_once starts */
        uint32_t first_bit = transmit(&c, link->atten, start_at);
        local.delay_ns = (uint32_t)((int32_t)(first_bit - their_last) * 1000 / (int32_t)CYCLES_PER_US);
        memcpy(rx, c.rx.body, rx_len);
        err = ESP_OK;
        break;
    }
    if (err != ESP_OK) {
        link_rx_stop(&c.rx);
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);

    tx_job_free(&c.job);
    local.syncs = c.rx.syncs;
    local.edge_errors = c.rx.bad_edges;
    if (info) {
        *info = local;
    }
    return err;
}

