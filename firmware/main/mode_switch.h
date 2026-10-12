/* Radio mode across restarts: the library takes its mode once per boot. */
#pragma once

#include "esp_sdr.h"

/* Mode requested before the last software restart (RX after power-on). */
esp_sdr_mode_t mode_switch_boot_mode(void);
/* Restart into `mode`. Does not return. */
void mode_switch_restart(esp_sdr_mode_t mode);
