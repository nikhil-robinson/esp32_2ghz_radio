#include "burst.h"

#include "dump_regs.h"
#include "esp_timer.h"
#include "heap_memory_layout.h"
#include "soc/soc.h"

#define BURST_TIMEOUT_US 20000
/* The dump engine writes these banks: keep them out of the heap (both aliases). */
SOC_RESERVE_MEMORY_REGION(DUMP_RING_BASE, DUMP_RING_END, esp_sdr_dump_banks);

#define UNWRITTEN 0xa5a0055au  /* fill pattern: cannot be produced by the engine (bits 28..31) */

uint32_t burst_rate_bits(esp_sdr_rate_t rate)
{
    return rate == ESP_SDR_RATE_16M ? DUMP_CTRL_RATE_16M : rate == ESP_SDR_RATE_40M ? DUMP_CTRL_RATE_40M : 0;
}

uint32_t *burst_buffer(void)
{
    return (uint32_t *)DUMP_BURST_ADDR;
}

esp_err_t burst_capture(esp_sdr_rate_t rate, uint32_t samples, uint32_t *elapsed_us)
{
    uint32_t *words = burst_buffer();
    for (uint32_t j = 0; j < samples; j++) {
        words[j] = UNWRITTEN;
    }
    const uint32_t owner = REG_READ(DUMP_BANK_SELECT_REG);
    const uint32_t ctrl = DUMP_CTRL_RUN | burst_rate_bits(rate) | samples;
    const int64_t start = esp_timer_get_time();

    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_CONFIG_REG, DUMP_CONFIG_IQ);
    REG_WRITE(DUMP_BANK_SELECT_REG, (owner & ~DUMP_BANK_MASK) | DUMP_BURST_BANK_MASK);
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    REG_WRITE(DUMP_CTRL_REG, ctrl | DUMP_CTRL_START);
    REG_WRITE(DUMP_CTRL_REG, ctrl);
    while (!(REG_READ(DUMP_CTRL_REG) & DUMP_CTRL_DONE) && esp_timer_get_time() - start < BURST_TIMEOUT_US) {
    }
    const bool done = (REG_READ(DUMP_CTRL_REG) & DUMP_CTRL_DONE) != 0;
    *elapsed_us = (uint32_t)(esp_timer_get_time() - start);
    REG_WRITE(DUMP_CTRL_REG, 0);
    REG_WRITE(DUMP_BANK_SELECT_REG, owner);

    if (!done) {
        return ESP_ERR_TIMEOUT;
    }
    for (uint32_t j = 0; j < samples; j++) {
        if (words[j] == UNWRITTEN) {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}
