/*
 * Tuning, receive mode, gain and baseband filter.
 *
 * Receive mode is (re)entered lazily by rf_prepare_rx() before every capture:
 * tune (with a fresh DC calibration when the LO changed or the PHY replaced
 * its tables), stop the calibration tone, force the front end into RX, apply
 * gain, select the LO divider path, then let the analog chain settle.
 */
#include "rf.h"

#include "esp_rom_sys.h"
#include "lo_plan.h"
#include "phy_private.h"
#include "soc/soc.h"

#define RX_SETTLE_US 3000u      /* analog settling after entering receive */
#define DEFAULT_MHZ 2412u
#define DEFAULT_GAIN_INDEX 40u

static uint32_t frequency_mhz = DEFAULT_MHZ;
static int32_t pll_offset_khz;
static uint32_t calibrated_mhz;
static bool rx_ready;

static bool gain_manual;
static uint8_t gain_index = DEFAULT_GAIN_INDEX;

static int filter_code = -1;
static uint32_t filter_saved[2];

/* Required by librftest.a (force_rx_gain); no RF test shell is exposed. */
int cmd_parse(char *cmd, char *name, int *argc, char **argv)
{
    (void)cmd;
    (void)name;
    (void)argc;
    (void)argv;
    return -1;
}

static void select_lo_divider(bool alternate)
{
    uint32_t lo = rom_chip_i2c_readReg(I2C_BLOCK_LO, 1, I2C_REG_LO);
    uint32_t want = (lo & ~I2C_LO_ALTERNATE) | (alternate ? I2C_LO_ALTERNATE : 0u);
    if (want != lo) {
        rom_chip_i2c_writeReg(I2C_BLOCK_LO, 1, I2C_REG_LO, want);
    }
}

static void tune(uint32_t mhz)
{
    if (calibrated_mhz != mhz || rf_calibration_stale()) {
        rf_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    lo_plan_t plan = lo_plan(mhz);
    /* A PLL offset requires the direct path even on channel frequencies. */
    bool use_channel_table = !pll_offset_khz && lo_is_wifi_channel(mhz);
    select_lo_divider(false);
    set_chanfreq(use_channel_table ? mhz : DEFAULT_MHZ, 0);
    if (!use_channel_table) {
        set_rf_freq_offset(0, plan.pll_mhz, plan.offset_khz + pll_offset_khz);
    }
}

uint8_t rf_gain_max(void)
{
    uint32_t max = PHY_AGC_MAX_INDEX(REG_READ(PHY_AGC_CTRL7_REG));
    return max <= PHY_GAIN_INDEX_LIMIT ? (uint8_t)max : 0;  /* never expose uncalibrated slots */
}

static void apply_gain(void)
{
    if (gain_index > rf_gain_max()) {
        gain_index = rf_gain_max();
    }
    force_rx_gain(gain_manual, gain_index, 0);
}

void rf_prepare_rx(void)
{
    if (rx_ready && !rf_calibration_stale()) {
        return;
    }
    tune(frequency_mhz);
    stop_tx_tone(1);
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    apply_gain();
    select_lo_divider(lo_plan(frequency_mhz).alternate);
    esp_rom_delay_us(RX_SETTLE_US);
    rx_ready = true;
}

void rf_invalidate(void) { rx_ready = false; }

void rf_tune_tx(uint32_t mhz, uint32_t khz)
{
    select_lo_divider(false);
    set_chanfreq(DEFAULT_MHZ, 0);
    set_rf_freq_offset(0, mhz, (int32_t)khz);
    rx_ready = false;
}

void rf_set_frequency(uint32_t mhz)
{
    frequency_mhz = mhz;
    rx_ready = false;
    rf_prepare_rx();
}

uint32_t rf_frequency(void) { return frequency_mhz; }

void rf_set_pll_offset(int32_t khz)
{
    pll_offset_khz = khz;
    rx_ready = false;
    rf_prepare_rx();
}

void rf_set_gain(bool manual, uint8_t index)
{
    gain_manual = manual;
    if (manual) {
        gain_index = index;
    }
    /* A gain change needs the complete receive setup, not just the index. */
    rx_ready = false;
    rf_prepare_rx();
}

bool rf_gain_manual(void) { return gain_manual; }
uint8_t rf_gain_index(void) { return gain_index; }
bool rf_gain_forced(void) { return (REG_READ(PHY_AGC_CTRL7_REG) & PHY_AGC_FORCED_BIT) != 0; }

void rf_set_filter(int code) { filter_code = code; }
int rf_filter(void) { return filter_code; }

void rf_filter_active(uint8_t *code_i, uint8_t *code_q)
{
    *code_i = (uint8_t)(rom_chip_i2c_readReg(I2C_BLOCK_RX_FILTER, 0, I2C_REG_FILTER_I) & I2C_FILTER_MASK);
    *code_q = (uint8_t)(rom_chip_i2c_readReg(I2C_BLOCK_RX_FILTER, 0, I2C_REG_FILTER_Q) & I2C_FILTER_MASK);
}

void rf_filter_apply(void)
{
    static const uint32_t regs[2] = {I2C_REG_FILTER_I, I2C_REG_FILTER_Q};
    for (unsigned j = 0; j < 2; j++) {
        filter_saved[j] = rom_chip_i2c_readReg(I2C_BLOCK_RX_FILTER, 0, regs[j]);
        if (filter_code >= 0) {
            rom_chip_i2c_writeReg(I2C_BLOCK_RX_FILTER, 0, regs[j],
                                  (filter_saved[j] & ~I2C_FILTER_MASK) | (uint32_t)filter_code);
        }
    }
}

void rf_filter_restore(void)
{
    static const uint32_t regs[2] = {I2C_REG_FILTER_I, I2C_REG_FILTER_Q};
    if (filter_code >= 0) {
        for (unsigned j = 0; j < 2; j++) {
            rom_chip_i2c_writeReg(I2C_BLOCK_RX_FILTER, 0, regs[j], filter_saved[j]);
        }
    }
}
