/* ESP-SDR serial firmware: serve host requests over USB-Serial-JTAG / UART0. */
#include "esp_log.h"
#include "esp_sdr.h"
#include "link.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_control.h"
#include "mode_switch.h"
#include "requests.h"

void app_main(void)
{
    /* The console shares the link: any log line would corrupt a frame. */
    esp_log_level_set("*", ESP_LOG_NONE);
    ESP_ERROR_CHECK(esp_sdr_init(mode_switch_boot_mode()));
    link_init();
    gpio_control_init();
    for (;;) {
        link_request_t request;
        if (link_poll(&request)) {
            requests_handle(&request);
        } else {
            vTaskDelay(1);
        }
    }
}
