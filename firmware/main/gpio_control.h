/* Host control of the board's free GPIOs: float, drive low, drive high. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum { GPIO_CONTROL_FLOAT = 0, GPIO_CONTROL_LOW = 1, GPIO_CONTROL_HIGH = 2 } gpio_control_state_t;

/* Float every free pin. Call once after the radio and link own their pins. */
void gpio_control_init(void);
/* Fill (pin, state) pairs for every free pin; returns the pair count. */
size_t gpio_control_list(uint8_t *pairs, size_t capacity_pairs);
esp_err_t gpio_control_set(uint8_t pin, gpio_control_state_t state);
