idf_component_register(
    SRCS "app.c"
    PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag 
    INCLUDE_DIRS
        "."
        "/opt/iotdata-library"               # iotdata.c/h
        "../../iotdata"                 # iotdata_variant_suite.h
        "/opt/iotdata-depend/e22900t22/include"        # e22xxxtxx.h
        "../../simulator"               # simulator
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wdouble-promotion
    -Werror=double-promotion
    -Wfloat-conversion
    -Werror=float-conversion
    -Wall -Wextra
    -Werror
    -Wstrict-prototypes
    -Wold-style-definition
    -Wcast-align -Wcast-qual -Wconversion
    -Wfloat-equal -Wformat=2 -Wformat-security
    -Winit-self -Wjump-misses-init
    -Wlogical-op -Wmissing-include-dirs
    -Wnested-externs -Wpointer-arith
    -Wredundant-decls -Wshadow
    -Wstrict-overflow=2 -Wswitch-default
    -Wunreachable-code -Wunused
    -Wwrite-strings
    -Os
)
