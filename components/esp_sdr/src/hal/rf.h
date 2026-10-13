/* RF control: tuning, receive path, gain, baseband filter, DC calibration. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ---- calibration (rf_calibration.c) ------------------------------------ */
/* Measure fresh RX DC at mhz, keeping the startup IQ correction (no TX tone). */
void rf_recalibrate(uint32_t mhz);
/* True if no measurement was made yet or the PHY replaced the DC tables since. */
bool rf_calibration_stale(void);

/* ---- tuning and receive path (rf_frontend.c) --------------------------- */
void rf_set_frequency(uint32_t mhz);
uint32_t rf_frequency(void);
void rf_set_pll_offset(int32_t khz);
/* (Re)establish receive mode at the current frequency if anything changed. */
void rf_prepare_rx(void);
/* Force the next rf_prepare_rx() to run completely. */
void rf_invalidate(void);

/* Transmit: put the LO exactly on mhz + khz through the direct PLL path
 * (2210..2790 MHz; no receive calibration). */
void rf_tune_tx(uint32_t mhz, uint32_t khz);
/* Receive at exactly `khz` (sets frequency and PLL offset) and prepare the path. */
void rf_tune_rx_khz(uint32_t khz);
/* Receive path back on after transmitting, same LO, no settle. */
void rf_rx_resume(void);
/* Crystal error to cancel in every tune, parts per billion (+ = runs fast). */
void rf_set_crystal_ppb(int32_t ppb);
int32_t rf_crystal_ppb(void);

void rf_set_gain(bool manual, uint8_t index);
bool rf_gain_manual(void);
uint8_t rf_gain_index(void);
uint8_t rf_gain_max(void);
bool rf_gain_forced(void);

/* -1 = PHY automatic, 0..63 = fixed code applied only around captures. */
void rf_set_filter(int code);
int rf_filter(void);
void rf_filter_active(uint8_t *code_i, uint8_t *code_q);
/* Apply / undo the requested filter code around one capture or stream. */
void rf_filter_apply(void);
void rf_filter_restore(void);
