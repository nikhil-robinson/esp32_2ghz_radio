idf_component_register(SRCS "main.c" "requests.c" "link.c" "frame.c" "gpio_control.c"
                       PRIV_REQUIRES esp_sdr driver esp_hw_support)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)

# Version record regenerated on every build (not only at configure time).
idf_build_get_property(python PYTHON)
set(version_header "${CMAKE_CURRENT_BINARY_DIR}/generated/firmware_version.h")
add_custom_target(esp_sdr_fw_version
    COMMAND "${python}" "${PROJECT_DIR}/../tools/version.py" --root "${PROJECT_DIR}/.." --output "${version_header}"
    BYPRODUCTS "${version_header}"
    VERBATIM)
add_dependencies(${COMPONENT_LIB} esp_sdr_fw_version)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
