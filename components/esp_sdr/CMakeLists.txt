if(NOT IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR "esp_sdr supports ESP32-S3 only (target is ${IDF_TARGET})")
endif()

idf_component_register(
    SRCS
        "src/sdr.c"
        "src/hal/rf_frontend.c"
        "src/hal/rf_calibration.c"
        "src/hal/burst.c"
        "src/hal/asm/app_cpu_entry.S"
        "src/hal/asm/fft_sc16_rounded.S"
        "src/hal/asm/iq10_unpack_windowed.S"
        "src/hal/asm/fir_kernels.S"
        "src/stream/ring.c"
        "src/stream/spectrum.c"
        "src/stream/iq_stream.c"
        "src/stream/worker.c"
        "src/stream/tx_queue.c"
        "src/dsp/bandwidth.c"
        "src/dsp/db_code.c"
        "src/dsp/fir_design.c"
        "src/dsp/iq_pack.c"
        "src/record/record.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src" "src/hal" "src/dsp" "src/record" "src/stream"
    LDFRAGMENTS "linker.lf"
    REQUIRES esp_wifi esp_phy esp_timer esp_event esp_hw_support nvs_flash heap
    PRIV_REQUIRES esp-dsp)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)

target_link_options(${COMPONENT_LIB} INTERFACE
    # Linker guard: static memory must stay below the radio's dump banks.
    "-T${CMAKE_CURRENT_LIST_DIR}/sram_guard.ld"
    # Receive-only DC recalibration at any LO (src/hal/rf_calibration.c).
    "-Wl,--wrap=chip_v7_set_chan_ana"
    "-Wl,--wrap=set_rx_gain_cal_dc")
