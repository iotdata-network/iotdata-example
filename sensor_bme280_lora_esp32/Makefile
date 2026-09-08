
##

IOTDATA_SRC=../..
include $(IOTDATA_SRC)/iotdata-common/make/config.mk
include $(IOTDATA_SRC)/iotdata-depend/make/config.mk

# IOTDATA_VARIANT=$(IOTDATA_SRC_COMMON)/include/iotdata_variant_weather_station.h
IOTDATA_CONFIG := $(firstword $(wildcard $(IOTDATA_SRC_EXAMPLE)/include/iotdata_config.$(IOTDATA_HOST).h iotdata_config.$(IOTDATA_HOST).h))

##

PLATFORM = esp32c3
NAME     = app
VERSION  = 1.00

SOURCES  = main/$(NAME).c main/*.h

# Blackbox diagnostics:
#   0 = off (compiled out entirely)
#   1 = RAM only    — store in RTC_NOINIT ring; survives deep sleep and watchdog/panic resets, but not a power cycle.
#   2 = RAM + flash — the ring is backed into the `diag` partition (partitions.csv), so records outlive a power loss.
BLACKBOX_ENABLE ?= 2

##

BUILDER_DEFS = \
    -DIOTDATA_SRC_LIBRARY=$(IOTDATA_SRC_LIBRARY) \
    -DIOTDATA_SRC_DEPEND_E22XXXTXX=$(IOTDATA_SRC_DEPEND_E22XXXTXX) \
    -DIOTDATA_SRC_DEPEND_BLACKBOX=$(IOTDATA_SRC_DEPEND_BLACKBOX) \
    -DIOTDATA_SRC_EXAMPLE=$(IOTDATA_SRC_EXAMPLE) \
    -DIOTDATA_SRC_COMMON=$(IOTDATA_SRC_COMMON) \
    -DIOTDATA_VARIANT=$(IOTDATA_VARIANT) \
    -DIOTDATA_BLACKBOX_ENABLE=$(BLACKBOX_ENABLE)
ifneq ($(strip $(IOTDATA_CONFIG)),)
BUILDER_DEFS += -DIOTDATA_CONFIG=$(abspath $(IOTDATA_CONFIG))
endif

##

include $(IOTDATA_SRC_COMMON)/make/target-esp32.mk

##

