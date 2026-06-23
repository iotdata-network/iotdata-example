include ../../iotdata-common/make/config.mk

PLATFORM = esp32c3
NAME     = app
SOURCES  = main/$(NAME).c
VERSION  = 1.00
DEVICE   = /dev/ttyACM0

# Push the shared source paths into CMake as cache entries, so main/CMakeLists.txt
# resolves its INCLUDE_DIRS from the same single source of truth (config.mk).
BUILDER_DEFS = \
    -DIOTDATA_SRC_LIBRARY=$(IOTDATA_SRC_LIBRARY) \
    -DIOTDATA_SRC_DEPEND=$(IOTDATA_SRC_DEPEND) \
    -DIOTDATA_SRC_EXAMPLE_COMMON=$(IOTDATA_SRC_EXAMPLE_COMMON) \
    -DIOTDATA_SRC_EXAMPLE_SIMULATOR=$(IOTDATA_SRC_EXAMPLE_SIMULATOR)

include ../../iotdata-common/make/target-esp32.mk
