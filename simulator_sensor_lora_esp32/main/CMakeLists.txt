# Source paths are normally injected by the Makefile (idf.py -DIOTDATA_SRC_...=...),
# from iotdata-common/make/config.mk — the single source of truth shared with the
# native examples. The block below only kicks in for a bare `idf.py build` (no
# Makefile): it self-locates IOTDATA_SRC from this file's location (main/ sits at
# <IOTDATA_SRC>/iotdata-example/simulator_sensor_lora_esp32/main).
if(NOT DEFINED IOTDATA_SRC_LIBRARY)
    get_filename_component(IOTDATA_SRC "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
    set(IOTDATA_SRC_LIBRARY           "${IOTDATA_SRC}/iotdata-library")
    set(IOTDATA_SRC_DEPEND            "${IOTDATA_SRC}/iotdata-depend")
    set(IOTDATA_SRC_EXAMPLE           "${IOTDATA_SRC}/iotdata-example")
    set(IOTDATA_SRC_EXAMPLE_COMMON    "${IOTDATA_SRC_EXAMPLE}/iotdata")
    set(IOTDATA_SRC_EXAMPLE_SIMULATOR "${IOTDATA_SRC_EXAMPLE}/simulator")
endif()

# Pin map variant: AE_SDC_CARRIER (default 1, i.e. on) selects the AE SDC carrier PCB
# pin map in app.c; set it empty (or 0) for the original bench wiring. Resolved from
# `idf.py -D AE_SDC_CARRIER=...` or the AE_SDC_CARRIER environment variable exported by
# the Makefile; defaults to 1.
if(NOT DEFINED AE_SDC_CARRIER)
    if(DEFINED ENV{AE_SDC_CARRIER})
        set(AE_SDC_CARRIER "$ENV{AE_SDC_CARRIER}")
    else()
        set(AE_SDC_CARRIER "1")
    endif()
endif()
if(AE_SDC_CARRIER)
    set(carrier_defs "AE_SDC_CARRIER")
    message(STATUS "app: pin map = AE SDC carrier (-D AE_SDC_CARRIER)")
else()
    set(carrier_defs "")
    message(STATUS "app: pin map = original bench wiring")
endif()

idf_component_register(
    SRCS "app.c"
    PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag
    INCLUDE_DIRS
        "."
        "${IOTDATA_SRC_LIBRARY}"                    # iotdata.c/h
        "${IOTDATA_SRC_EXAMPLE_COMMON}"             # iotdata_variant_suite.h
        "${IOTDATA_SRC_DEPEND}/e22900t22/include"   # e22xxxtxx.h
        "${IOTDATA_SRC_EXAMPLE_SIMULATOR}"          # simulator
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${carrier_defs})
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
