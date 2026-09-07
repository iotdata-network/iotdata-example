
idf_component_register(
    SRCS "app.c"
    PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_partition
    INCLUDE_DIRS
        "."
        "${IOTDATA_SRC_EXAMPLE}/include"            # iotdata_config.h
        "${IOTDATA_SRC_LIBRARY}"                    # iotdata.c/h
        "${IOTDATA_SRC_COMMON}/include"             # iotdata_variant.h (+ the variant file it pulls in)
        "${IOTDATA_SRC_DEPEND_E22XXXTXX}"           # e22xxxtxx.h
        "${IOTDATA_SRC_EXAMPLE_SIMULATOR}"          # simulator
        "${IOTDATA_SRC_DEPEND_BLACKBOX}"            # blackbox.h (only used when BLACKBOX_ENABLE > 0)
)

# Blackbox diagnostics: 0 = off, 1 = RAM (RTC ring), 2 = RAM + flash (`diag` partition).
if(NOT DEFINED IOTDATA_BLACKBOX_ENABLE)
    set(IOTDATA_BLACKBOX_ENABLE 0)
endif()
if(IOTDATA_BLACKBOX_ENABLE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE IOTDATA_BLACKBOX_ENABLE=${IOTDATA_BLACKBOX_ENABLE})
    if(IOTDATA_BLACKBOX_ENABLE GREATER_EQUAL 2)
        message(STATUS "simulator_sensor: blackbox diagnostics = RAM + flash (`diag` partition)")
    else()
        message(STATUS "simulator_sensor: blackbox diagnostics = RAM only (RTC ring)")
    endif()
endif()

include("${IOTDATA_SRC_COMMON}/make/target-esp32.cmake")
