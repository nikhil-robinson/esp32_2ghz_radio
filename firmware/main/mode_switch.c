#include "mode_switch.h"

#include "esp_attr.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MODE_MAGIC 0x5D2A0000u

/* Survives esp_restart(); garbage after power-on, hence the magic. */
static RTC_NOINIT_ATTR uint32_t requested;

esp_sdr_mode_t mode_switch_boot_mode(void)
{
    bool valid = (requested & 0xFFFF0000u) == MODE_MAGIC && (requested & 0xFFFFu) <= ESP_SDR_MODE_TX;
    return valid ? (esp_sdr_mode_t)(requested & 0xFFFFu) : ESP_SDR_MODE_RX;
}

void mode_switch_restart(esp_sdr_mode_t mode)
{
    requested = MODE_MAGIC | (uint32_t)mode;
    vTaskDelay(pdMS_TO_TICKS(50));  /* let the ACK leave */
    esp_restart();
}
