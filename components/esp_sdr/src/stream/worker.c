#include "worker.h"

#include "esp32s3/rom/ets_sys.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "soc/soc.h"
#include "soc/system_reg.h"

#define START_TIMEOUT_US 50000

worker_mailbox_t g_worker;
static stream_unit_t queue[WORKER_QUEUE_DEPTH];
static worker_hooks_t hooks;
static bool alive;

extern void app_cpu_entry(void);  /* app_cpu_entry.S */

/* Same release sequence ESP-IDF's cpu_start.c uses for the APP CPU. */
static void app_cpu_power_up(void)
{
    esp_cpu_unstall(1);
    REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_CLKGATE_EN);
    REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RUNSTALL);
    REG_SET_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETING);
    REG_CLR_BIT(SYSTEM_CORE_1_CONTROL_0_REG, SYSTEM_CONTROL_CORE_1_RESETING);
}

bool worker_start(void)
{
    if (alive) {
        return true;
    }
    app_cpu_power_up();
    ets_set_appcpu_boot_addr((uint32_t)app_cpu_entry);
    int64_t start = esp_timer_get_time();
    while (!g_worker.alive && esp_timer_get_time() - start < START_TIMEOUT_US) {
    }
    alive = g_worker.alive != 0;
    return alive;
}

bool worker_alive(void)
{
    return alive;
}

/* Core 1 entry (from app_cpu_entry.S): interrupts masked, never returns. */
IRAM_ATTR void worker_main(void)
{
    uint32_t run_seen = g_worker.run;
    MEMORY_BARRIER();
    g_worker.alive = 1;
    MEMORY_BARRIER();
    for (;;) {
        while (g_worker.run == run_seen) {
        }
        run_seen = g_worker.run;
        MEMORY_BARRIER();
        uint32_t taken = 0;
        for (;;) {
            MEMORY_BARRIER();
            if (taken == g_worker.posted) {
                hooks.idle();
                if (!g_worker.end) {
                    continue;
                }
                MEMORY_BARRIER();
                if (taken == g_worker.posted) {  /* `end` is written after the last post */
                    break;
                }
                continue;
            }
            stream_unit_t unit = queue[taken % WORKER_QUEUE_DEPTH];
            hooks.unit(&unit);
            taken++;
            g_worker.taken = taken;
            hooks.idle();
        }
        hooks.finish();
        MEMORY_BARRIER();
        g_worker.done = 1;
    }
}

void worker_begin_run(const worker_hooks_t *run_hooks)
{
    hooks = *run_hooks;
    g_worker.posted = g_worker.taken = g_worker.end = g_worker.done = g_worker.busy = g_worker.block_max = 0;
    for (unsigned i = 0; i < ESP_SDR_RING_BANKS; i++) {
        g_worker.bank_seq[i] = 0;
    }
    MEMORY_BARRIER();
    g_worker.run = g_worker.run + 1u;
}

STREAM_HOT bool worker_post(const stream_unit_t *unit)
{
    uint32_t i = g_worker.posted;
    if (i - g_worker.taken >= WORKER_QUEUE_DEPTH) {
        return false;
    }
    queue[i % WORKER_QUEUE_DEPTH] = *unit;
    g_worker.bank_seq[unit->bank] = unit->seq;
    MEMORY_BARRIER();
    g_worker.posted = i + 1u;
    return true;
}

STREAM_HOT void worker_revoke_bank(unsigned bank)
{
    g_worker.bank_seq[bank] = 0;
    MEMORY_BARRIER();
    while (g_worker.busy == bank + 1u) {
    }
}

STREAM_HOT bool worker_bank_idle(unsigned bank)
{
    return g_worker.bank_seq[bank] == 0;
}

STREAM_HOT void worker_end_run(void)
{
    MEMORY_BARRIER();
    g_worker.end = 1;
    MEMORY_BARRIER();
    while (!g_worker.done) {
        tx_queue_pump();
    }
    MEMORY_BARRIER();
}
