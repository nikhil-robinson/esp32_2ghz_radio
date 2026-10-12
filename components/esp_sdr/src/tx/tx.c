/*
 * Transmitter: the PHY tone generator, frequency-modulated by the CPU.
 *
 * A packet becomes a table of tone-register words (one per bit, or per
 * Gaussian sub-step) on the heap. Core 0 then writes them with interrupts
 * masked at absolute cycle-count deadlines, so timing errors never add up.
 * The carrier is held two bits past the packet and is off between packets.
 */
#include "esp_sdr_tx.h"

#include <math.h>
#include <stdlib.h>
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mode.h"
#include "packet.h"
#include "phy_private.h"
#include "rf.h"
#include "soc/soc.h"
#include "tx_plan.h"
#include "xtensa/xtruntime.h"

#define TAG "esp_sdr_tx"
#define TAIL_BITS 2u
#define GAUSSIAN_BT 0.5f
#define GAUSSIAN_SPAN 2  /* bits each side that a bit's pulse reaches */
#define YIELD_GAP_US 2000u

static esp_sdr_region_t region = ESP_SDR_REGION_ISM;
static bool carrier_on;

esp_err_t esp_sdr_tx_set_region(esp_sdr_region_t r)
{
    if ((unsigned)r > ESP_SDR_REGION_UNRESTRICTED) {
        return ESP_ERR_INVALID_ARG;
    }
    if (r == ESP_SDR_REGION_HAM) {
        ESP_LOGW(TAG, "region HAM: 2300-2450 MHz, for licensed radio amateurs only");
    } else if (r == ESP_SDR_REGION_UNRESTRICTED) {
        ESP_LOGW(TAG, "region UNRESTRICTED: 2210-2790 MHz includes licensed services; shielded/lab use only");
    }
    region = r;
    return ESP_OK;
}

esp_sdr_region_t esp_sdr_tx_get_region(void) { return region; }

static esp_err_t check(uint32_t center_khz, uint32_t deviation_khz)
{
    if (sdr_mode() != ESP_SDR_MODE_TX) {
        return ESP_ERR_INVALID_STATE;
    }
    if (center_khz < deviation_khz ||
        !tx_region_allows(region, center_khz - deviation_khz, center_khz + deviation_khz)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

/* Tune, start the generator, and return the register word with field 0, tone on. */
static uint32_t carrier_start(uint32_t center_khz, uint8_t atten)
{
    rf_tune_tx(center_khz / 1000u, center_khz % 1000u);
    txcal_debuge_mode();
    start_tx_tone_step(1, 0, atten, 0, 0, 0);
    carrier_on = true;
    return (REG_READ(PHY_TONE1_REG) & ~PHY_TONE_FIELD_MASK) | PHY_TONE_ON;
}

static void carrier_stop(void)
{
    start_tx_tone_step(0, 0, 0, 0, 0, 0);
    txcal_work_mode();
    carrier_on = false;
}

esp_err_t esp_sdr_tx_tone(uint32_t khz, uint8_t atten)
{
    esp_err_t err = check(khz, 0);
    if (err == ESP_OK) {
        REG_WRITE(PHY_TONE1_REG, carrier_start(khz, atten));
    }
    return err;
}

esp_err_t esp_sdr_tx_off(void)
{
    if (sdr_mode() != ESP_SDR_MODE_TX) {
        return ESP_ERR_INVALID_STATE;
    }
    if (carrier_on) {
        carrier_stop();
    }
    return ESP_OK;
}

static inline int bit_at(const uint8_t *bits, size_t nbits, long i)
{
    if (i < 0) {
        i = 0;
    } else if ((size_t)i >= nbits) {
        i = (long)nbits - 1;  /* the tail repeats the last bit */
    }
    return (bits[i >> 3] >> (7 - (i & 7))) & 1;
}

/* Fraction of a bit's frequency pulse at time tau (bits from its centre):
 * a rectangle one bit wide filtered by a Gaussian of bandwidth BT. */
static float gaussian_pulse(float tau)
{
    const float a = (float)M_PI * GAUSSIAN_BT * sqrtf(2.0f / logf(2.0f));
    return 0.5f * (erff(a * (tau + 0.5f)) - erff(a * (tau - 0.5f)));
}

/* Tone-field value for every sub-step, already in register form. */
static void build_table(uint32_t *table, const uint8_t *bits, size_t nbits, unsigned sub, int dev_steps,
                        uint32_t on)
{
    size_t n = 0;
    for (long k = 0; k < (long)(nbits + TAIL_BITS); k++) {
        for (unsigned s = 0; s < sub; s++) {
            float f;
            if (sub == 1) {
                f = bit_at(bits, nbits, k) ? 1.0f : -1.0f;
            } else {
                float t = ((float)s + 0.5f) / (float)sub - 0.5f;  /* sub-step centre relative to bit k */
                f = 0.0f;
                for (int j = -GAUSSIAN_SPAN; j <= GAUSSIAN_SPAN; j++) {
                    f += (bit_at(bits, nbits, k + j) ? 1.0f : -1.0f) * gaussian_pulse(t - (float)j);
                }
            }
            int field = (int)lroundf(f * (float)dev_steps);  /* positive field = higher frequency */
            table[n++] = on | ((uint32_t)field & PHY_TONE_FIELD_MASK);
        }
    }
}

/* One packet with interrupts masked; returns the worst lateness in cycles. */
static IRAM_ATTR int32_t send_once(const uint32_t *table, size_t n, uint32_t cycles, uint32_t idle)
{
    unsigned irq = XTOS_SET_INTLEVEL(XCHAL_EXCM_LEVEL);
    int32_t worst = 0;
    /* The first tone-register write after a pause can stall for 250+ cycles
     * (measured); absorb it with the carrier still off, before the clock starts. */
    REG_WRITE(PHY_TONE1_REG, idle);
    __asm__ __volatile__("memw");
    (void)REG_READ(PHY_TONE1_REG);
    uint32_t t = esp_cpu_get_cycle_count();
    for (size_t i = 0; i < n; i++) {
        REG_WRITE(PHY_TONE1_REG, table[i]);
        t += cycles;
        int32_t d;
        while ((d = (int32_t)(esp_cpu_get_cycle_count() - t)) < 0) {
        }
        if (d > worst) {
            worst = d;
        }
    }
    REG_WRITE(PHY_TONE1_REG, idle);
    XTOS_RESTORE_INTLEVEL(irq);
    return worst;
}

static void gap(uint32_t us)
{
    if (us >= YIELD_GAP_US) {
        vTaskDelay(pdMS_TO_TICKS(us / 1000u));
        us %= 1000u;
    }
    esp_rom_delay_us(us);
}

esp_err_t esp_sdr_tx_bits(const esp_sdr_fsk_t *fsk, const uint8_t *bits, size_t nbits, uint16_t count,
                          uint32_t gap_us)
{
    if (!fsk || !bits || !nbits || nbits > ESP_SDR_TX_MAX_BITS || !count || !tx_bitrate_supported(fsk->bitrate)) {
        return sdr_mode() != ESP_SDR_MODE_TX ? ESP_ERR_INVALID_STATE : ESP_ERR_INVALID_ARG;
    }
    int dev_steps = tx_deviation_steps(fsk->deviation_hz);
    esp_err_t err = check(fsk->center_khz, ((uint32_t)dev_steps * ESP_SDR_TX_STEP_HZ + 999u) / 1000u);
    if (err != ESP_OK) {
        return err;
    }
    unsigned sub = tx_substeps(fsk->bitrate, fsk->gaussian);
    size_t n = (nbits + TAIL_BITS) * sub;
    uint32_t *table = heap_caps_malloc(n * sizeof(uint32_t), MALLOC_CAP_INTERNAL);  /* not static: sram_guard */
    if (!table) {
        return ESP_ERR_NO_MEM;
    }
    uint32_t on = carrier_start(fsk->center_khz, fsk->atten);
    uint32_t idle = on & ~PHY_TONE_ON;
    build_table(table, bits, nbits, sub, dev_steps, on);
    uint32_t cycles = esp_clk_cpu_freq() / (fsk->bitrate * sub);
    int32_t worst = 0;
    REG_WRITE(PHY_TONE1_REG, idle);
    for (unsigned c = 0; c < count; c++) {
        int32_t late = send_once(table, n, cycles, idle);
        if (late > worst) {
            worst = late;
        }
        if (c + 1 < count) {
            gap(gap_us);
        }
    }
    carrier_stop();
    free(table);
    /* Half a bit late can flip the receiver's decision for that bit. */
    return worst > (int32_t)(cycles * sub / 2u) ? ESP_ERR_TIMEOUT : ESP_OK;
}

static const uint32_t ble_channel_khz[3] = {2402000u, 2426000u, 2480000u};

esp_err_t esp_sdr_tx_ble_adv(uint8_t channel, const uint8_t adv_addr[6], const uint8_t *data, size_t len,
                             uint8_t atten, uint16_t count, uint32_t gap_us)
{
    uint8_t bits[PACKET_BLE_MAX_BYTES];
    size_t nbits = adv_addr ? packet_ble_adv(bits, channel, adv_addr, data, len) : 0;
    if (!nbits) {
        return sdr_mode() != ESP_SDR_MODE_TX ? ESP_ERR_INVALID_STATE : ESP_ERR_INVALID_ARG;
    }
    const esp_sdr_fsk_t fsk = {.center_khz = ble_channel_khz[channel - 37], .bitrate = 1000000u,
                               .deviation_hz = 250000u, .atten = atten};
    return esp_sdr_tx_bits(&fsk, bits, nbits, count, gap_us);
}

esp_err_t esp_sdr_tx_nrf24(const esp_sdr_nrf24_t *config, const uint8_t *payload, size_t len,
                           uint16_t count, uint32_t gap_us)
{
    uint8_t bits[PACKET_NRF24_MAX_BYTES];
    size_t nbits = config && config->channel <= 125
                       ? packet_nrf24(bits, config->addr, config->addr_len, config->crc_len, payload, len)
                       : 0;
    if (!nbits) {
        return sdr_mode() != ESP_SDR_MODE_TX ? ESP_ERR_INVALID_STATE : ESP_ERR_INVALID_ARG;
    }
    /* nRF24L01+ datasheet: +-160 kHz at 250 kbps / 1 Mbps, +-320 kHz at 2 Mbps. */
    const esp_sdr_fsk_t fsk = {.center_khz = 2400000u + 1000u * config->channel, .bitrate = config->bitrate,
                               .deviation_hz = config->bitrate == 2000000u ? 320000u : 160000u,
                               .atten = config->atten};
    return esp_sdr_tx_bits(&fsk, bits, nbits, count, gap_us);
}
