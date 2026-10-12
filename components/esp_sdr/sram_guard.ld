/* The radio's dump engine owns 0x3fcb0000..0x3fce0000 (three ring banks,
 * bank 2 doubling as the burst window) in both the data and instruction
 * aliases. Static data and IRAM code must end below it. */
ASSERT(_iram_end <= 0x403a0000, "esp_sdr: IRAM overlaps the radio dump banks")
ASSERT(_data_end <= 0x3fcb0000, "esp_sdr: .data overlaps the radio dump banks")
ASSERT(_bss_end <= 0x3fcb0000, "esp_sdr: .bss overlaps the radio dump banks")
