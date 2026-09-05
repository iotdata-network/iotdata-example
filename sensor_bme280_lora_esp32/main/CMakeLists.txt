
idf_component_register(
    SRCS "app.c"
    PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_uart esp_driver_i2c esp_driver_usb_serial_jtag esp_adc
    INCLUDE_DIRS
        "."
        "${IOTDATA_SRC_EXAMPLE}/include"            # iotdata_config.h
        "${IOTDATA_SRC_LIBRARY}"                    # iotdata.c/h
        "${IOTDATA_SRC_COMMON}/include"             # iotdata_variant.h (+ the variant file it pulls in)
        "${IOTDATA_SRC_DEPEND_E22XXXTXX}"           # e22xxxtxx.h
)

include("${IOTDATA_SRC_COMMON}/make/target-esp32.cmake")

