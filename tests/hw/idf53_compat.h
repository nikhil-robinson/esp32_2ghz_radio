/* Force-included (-include) to build the S3 firmware on ESP-IDF 5.3, which lacks
 * two helpers added in later releases. Used only by tests/hw/run_s3.sh.
 * Macros only, no includes: this header lands in every IDF translation unit,
 * and expands only at the two call sites, which already include soc.h/FreeRTOS. */
#pragma once

/* Same sequence IDF 5.3 cpu_start.c uses to bring up the APP CPU:
 * SYSTEM_CORE_1_CONTROL_0_REG = DR_REG_SYSTEM_BASE + 0; bit0 RUNSTALL, bit1 CLKGATE_EN, bit2 RESETING. */
#define cpu_utility_ll_enable_clock_and_reset_app_cpu() do { \
    REG_SET_BIT(DR_REG_SYSTEM_BASE, BIT(1));                 \
    REG_CLR_BIT(DR_REG_SYSTEM_BASE, BIT(0));                 \
    REG_SET_BIT(DR_REG_SYSTEM_BASE, BIT(2));                 \
    REG_CLR_BIT(DR_REG_SYSTEM_BASE, BIT(2));                 \
} while (0)

/* ponytail: fixed delay instead of a real FIFO drain; only used before driver uninstall. */
#define usb_serial_jtag_wait_tx_done(ticks) (vTaskDelay(ticks), 0)
