#include "gpio_control.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#define PIN(n) (UINT64_C(1) << (n))
#define PINS(first, last) ((PIN((last) + 1) - 1) & ~(PIN(first) - 1))
/* S3 SPI flash / PSRAM pads (including optional octal PSRAM). */
#define MEMORY_PINS PINS(26, 37)
/* S3 USB D-/D+ are fixed in silicon (USB-Serial-JTAG). */
#define USB_DM_PIN 19
#define USB_DP_PIN 20

static uint64_t free_pins;
static gpio_control_state_t states[SOC_GPIO_PIN_COUNT];

static uint64_t reserved_pins(void)
{
    uint64_t mask = MEMORY_PINS | PIN(USB_DM_PIN) | PIN(USB_DP_PIN);
#if CONFIG_ESP_SDR_UART_ENABLED
    mask |= PIN(CONFIG_ESP_SDR_UART_TX_PIN) | PIN(CONFIG_ESP_SDR_UART_RX_PIN);
#endif
    return mask;
}

void gpio_control_init(void)
{
    free_pins = SOC_GPIO_VALID_OUTPUT_GPIO_MASK & ~reserved_pins();
    for (unsigned pin = 0; pin < SOC_GPIO_PIN_COUNT; pin++) {
        if (!(free_pins & PIN(pin))) {
            continue;
        }
        /* DISABLE disconnects input and output; no pulls. Boot only. */
        const gpio_config_t config = {
            .pin_bit_mask = PIN(pin),
            .mode = GPIO_MODE_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&config) != ESP_OK) {
            free_pins &= ~PIN(pin);
            continue;
        }
        states[pin] = GPIO_CONTROL_FLOAT;
    }
}

size_t gpio_control_list(uint8_t *pairs, size_t capacity_pairs)
{
    size_t n = 0;
    for (unsigned pin = 0; pin < SOC_GPIO_PIN_COUNT && n < capacity_pairs; pin++) {
        if (free_pins & PIN(pin)) {
            pairs[2 * n] = (uint8_t)pin;
            pairs[2 * n + 1] = (uint8_t)states[pin];
            n++;
        }
    }
    return n;
}

esp_err_t gpio_control_set(uint8_t pin, gpio_control_state_t state)
{
    if (pin >= SOC_GPIO_PIN_COUNT || !(free_pins & PIN(pin)) || state > GPIO_CONTROL_HIGH) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Preload the output latch before enabling the driver: no glitch from Z to 1. */
    esp_err_t err = ESP_OK;
    if (state != GPIO_CONTROL_FLOAT) {
        err = gpio_set_level(pin, state == GPIO_CONTROL_HIGH);
    }
    if (err == ESP_OK) {
        err = gpio_set_direction(pin, state == GPIO_CONTROL_FLOAT ? GPIO_MODE_DISABLE : GPIO_MODE_OUTPUT);
    }
    if (err == ESP_OK) {
        states[pin] = state;
    }
    return err;
}
