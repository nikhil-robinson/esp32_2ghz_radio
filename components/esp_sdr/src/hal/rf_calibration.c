/*
 * Receive-only DC recalibration for any LO frequency.
 *
 * The PHY calibrates DC on fixed reference channels. To measure at the LO we
 * actually use, the channel setup (chip_v7_set_chan_ana) is wrapped: during
 * our synchronous measurement the wrapper retunes to `measuring_mhz`. Clearing
 * the "DC valid" flag in phy_param makes set_rx_gain_table measure again
 * while the IQ-valid flag keeps it from running the loopback IQ step, which
 * would transmit a tone.
 *
 * Staleness: the PHY's own tracker can replace DC tables at any time, so
 * every DC measurement (set_rx_gain_cal_dc) is counted through a second
 * wrapper. A measurement count different from ours means "stale".
 *
 * Both wrappers are installed with -Wl,--wrap (see CMakeLists.txt).
 */
#include <stdatomic.h>
#include <stdint.h>
#include <sys/lock.h>
#include "esp_private/phy.h"
#include "soc/soc.h"
#include "rf.h"
#include "phy_private.h"
#include "lo_plan.h"

static uint32_t measuring_mhz;
static atomic_uint dc_measurements;
static unsigned measurements_at_calibration;
static bool calibrated;

extern void __real_chip_v7_set_chan_ana(uint32_t channel);
extern void __real_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d, uintptr_t e,
                                      uintptr_t f, uintptr_t g, uintptr_t h, uintptr_t i, uintptr_t j);

static void tune_for_measurement(void)
{
    lo_plan_t plan = lo_plan(measuring_mhz);
    set_rf_freq_offset(0, plan.pll_mhz, plan.offset_khz);
    uint32_t lo = rom_chip_i2c_readReg(I2C_BLOCK_LO, 1, I2C_REG_LO);
    uint32_t want = (lo & ~I2C_LO_ALTERNATE) | (plan.alternate ? I2C_LO_ALTERNATE : 0u);
    if (want != lo) {
        rom_chip_i2c_writeReg(I2C_BLOCK_LO, 1, I2C_REG_LO, want);
    }
}

void __wrap_chip_v7_set_chan_ana(uint32_t channel)
{
    __real_chip_v7_set_chan_ana(channel);
    if (measuring_mhz) {
        tune_for_measurement();
    }
}

/* Forward the private 10-word calling convention unchanged. */
void __wrap_set_rx_gain_cal_dc(uintptr_t a, uintptr_t b, uintptr_t c, uintptr_t d, uintptr_t e,
                               uintptr_t f, uintptr_t g, uintptr_t h, uintptr_t i, uintptr_t j)
{
    __real_set_rx_gain_cal_dc(a, b, c, d, e, f, g, h, i, j);
    atomic_fetch_add(&dc_measurements, 1);
}

bool rf_calibration_stale(void)
{
    return !calibrated || atomic_load(&dc_measurements) != measurements_at_calibration;
}

void rf_recalibrate(uint32_t mhz)
{
    _lock_t lock = phy_get_lock();
    _lock_acquire(&lock);
    measuring_mhz = mhz;
    /* A manual gain from earlier must not override the measurement's sweep. */
    REG_CLR_BIT(PHY_AGC_CTRL7_REG, PHY_AGC_FORCED_BIT);
    tune_for_measurement();
    *(uint32_t *)(phy_param + PHY_PARAM_RX_DC_FLAGS_OFFSET) &= ~PHY_PARAM_RX_DC_VALID;
    set_rx_gain_table(lo_plan(mhz).pll_mhz, 0);
    measuring_mhz = 0;
    measurements_at_calibration = atomic_load(&dc_measurements);
    calibrated = true;
    _lock_release(&lock);
}
