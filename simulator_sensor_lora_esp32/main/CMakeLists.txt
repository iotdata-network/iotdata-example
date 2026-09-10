
idf_component_register(
    SRCS "app.c"
    PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_partition esp_hw_support
    INCLUDE_DIRS
        "."
        "${IOTDATA_SRC_EXAMPLE}/include"            # iotdata_config.h
        "${IOTDATA_SRC_LIBRARY}"                    # iotdata.c/h
        "${IOTDATA_SRC_COMMON}/include"             # iotdata_variant.h (+ the variant file it pulls in)
        "${IOTDATA_SRC_COMMON}/include/device"      # d_common.h, d_hardware_*.h, d_interface_e22900t22.h
        "${IOTDATA_SRC_EXAMPLE_SIMULATOR}"          # simulator
        "${IOTDATA_SRC_DEPEND_BLACKBOX}"            # blackbox.h (only used when BLACKBOX_ENABLE > 0)
)

include("${IOTDATA_SRC_COMMON}/make/target-esp32.cmake")
