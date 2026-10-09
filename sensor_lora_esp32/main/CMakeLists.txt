
set(bench_defs "")
if(NO_SLEEP_DEEP)
    list(APPEND bench_defs "NO_SLEEP_DEEP=1")
    if(NOT CMAKE_BUILD_EARLY_EXPANSION)
        message(WARNING "app: NO_SLEEP_DEEP -- board stays awake during cycles, DO NOT SHIP")
    endif()
endif()
if(NO_BATTERY_PROBE)
    list(APPEND bench_defs "NO_BATTERY_PROBE=1")
    if(NOT CMAKE_BUILD_EARLY_EXPANSION)
        message(WARNING "app: NO_BATTERY_PROBE -- battery hardware will not be probed for")
    endif()
endif()

idf_component_register(
    SRCS "app.c"
    PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_uart esp_driver_i2c esp_driver_usb_serial_jtag esp_adc esp_partition esp_hw_support esp_app_format esp_bootloader_format nvs_flash vfs
    INCLUDE_DIRS
        "."
        "${IOTDATA_SRC_EXAMPLE}/include"            # iotdata_config.h
        "${IOTDATA_SRC_LIBRARY}"                    # iotdata.c/h
        "${IOTDATA_SRC_COMMON}/include"             # iotdata_variant.h (+ the variant file it pulls in)
        "${IOTDATA_SRC_COMMON}/include/device"      # d_common.h, d_hardware_*.h, d_interface_e22900t22.h
        "${IOTDATA_SRC_COMMON}/apps"
        "${IOTDATA_SRC_DEPEND_BLACKBOX}"            # blackbox.h (only used when BLACKBOX_ENABLE > 0)
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE ${bench_defs})

include("${IOTDATA_SRC_COMMON}/make/target-esp32.cmake")
