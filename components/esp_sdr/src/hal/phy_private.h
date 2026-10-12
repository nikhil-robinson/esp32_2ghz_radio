/*
 * Private ESP32-S3 PHY / ROM entry points used by the receiver.
 *
 * None of these are public ESP-IDF API. Each was checked to exist with the
 * same calling convention in the S3 libphy.a shipped with ESP-IDF 5.1 and 5.3
 * and in the S3 ROM linker script (esp32s3.rom.ld). Keep every such symbol in
 * this header so an IDF update has exactly one place to review.
 */
#pragma once

#include <stdint.h>

/* libphy.a: retune the RF PLL to a Wi-Fi channel frequency (mode 0). */
extern void set_chanfreq(uint32_t mhz, uint32_t mode);
/* libphy.a: retune to any frequency; xtal_sel 0 = 40 MHz crystal.
 * PLL divider N = 4 f / (3 f_xtal) - 32 (8-bit integer + 16-bit fraction). */
extern void set_rf_freq_offset(uint32_t xtal_sel, uint32_t mhz, int32_t khz);
/* librftest.a: manual != 0 pins the RX gain table index. */
extern void force_rx_gain(uint32_t manual, uint32_t index, uint32_t unused);
/* libphy.a: rebuild DC tables / gain memory for an LO frequency. */
extern void set_rx_gain_table(uint32_t mhz, uint32_t flags);
/* libphy.a: stop the calibration tone generator. */
extern void stop_tx_tone(uint32_t wait);
/* libphy.a: tone generator for transmit (proven in the TX spike, IDF 5.3):
 * debug mode routes the generator to the PA; step(1, 0, atten, ...) starts it. */
extern void txcal_debuge_mode(void);
extern void txcal_work_mode(void);
extern void start_tx_tone_step(uint32_t on, int32_t a, int32_t atten, uint32_t b, int32_t c, int32_t d);
/* Tone generator 1: bits 0..9 signed frequency offset, 40 MHz / 256 per step;
 * bit 18 enables the tone. After esp_sdr_init(TX) a positive value raises
 * the frequency (measured 2026-10-13, tests/hw/tx_rf.py; the rftest-mode spike
 * saw the opposite). */
#define PHY_TONE1_REG 0x60006040u
#define PHY_TONE_FIELD_MASK 0x3FFu
#define PHY_TONE_ON (1u << 18)
/* Tone generator fine frequency: bits 0..1 add 40 MHz / 1024 per step on top
 * of the coarse field (measured 2026-10-13; no enable bit needed). */
#define PHY_TONE1_FINE_REG 0x60006050u
#define PHY_TONE_FINE_MASK 0x3u
/* libphy.a: PHY parameter block. +288 bit 0x200 marks the RX DC tables valid. */
extern uint8_t phy_param[];
#define PHY_PARAM_RX_DC_FLAGS_OFFSET 288u
#define PHY_PARAM_RX_DC_VALID 0x200u

/* ROM: force the RF front end into receive. */
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(uint32_t on);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(uint32_t on);
/* ROM: analog register bus (block, host, register). */
extern uint32_t rom_chip_i2c_readReg(uint32_t block, uint32_t host, uint32_t reg);
extern void rom_chip_i2c_writeReg(uint32_t block, uint32_t host, uint32_t reg, uint32_t value);

/* Analog register locations. */
#define I2C_BLOCK_RX_FILTER 0x67u  /* regs 4 (I) and 5 (Q), bits 0..5: baseband filter code */
#define I2C_REG_FILTER_I 4u
#define I2C_REG_FILTER_Q 5u
#define I2C_FILTER_MASK 63u
#define I2C_BLOCK_LO 0x65u         /* reg 0 bit 4: alternate (6/5) receive LO divider */
#define I2C_REG_LO 0u
#define I2C_LO_ALTERNATE 0x10u

/* AGC control 7: bits 8..14 = largest calibrated gain index, bit 23 = forced gain. */
#define PHY_AGC_CTRL7_REG 0x6001c02cu
#define PHY_AGC_MAX_INDEX(v) (((v) >> 8) & 127u)
#define PHY_AGC_FORCED_BIT (1u << 23)
#define PHY_GAIN_INDEX_LIMIT 82u    /* larger values mean "not calibrated" */
