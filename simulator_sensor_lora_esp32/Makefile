
##

IOTDATA_SRC=../..
include $(IOTDATA_SRC)/iotdata-common/make/config.mk
include $(IOTDATA_SRC)/iotdata-depend/make/config.mk
IOTDATA_SRC_EXAMPLE_SIMULATOR=$(IOTDATA_SRC_EXAMPLE)/simulator

IOTDATA_VARIANT=$(IOTDATA_SRC_COMMON)/include/iotdata_variant_weather_station.h
IOTDATA_CONFIG := $(firstword $(wildcard $(IOTDATA_SRC_EXAMPLE)/include/iotdata_config.$(IOTDATA_HOST).h iotdata_config.$(IOTDATA_HOST).h))

##

PLATFORM = esp32c3
NAME     = simulator-sensor
VERSION  = 0.9.9

##

# SIMULATE_RECEIVE_ALWAYS=  1 (default) the simulated sensors listen all the time, so a node
#                           command is acted on when sent. 0 makes them imitate a real sleeping
#                           sensor (windows + advertisements), to exercise that path on the bench.
SIMULATE_RECEIVE_ALWAYS ?= 1

##

SOURCES_TARGET= main/app.c
SOURCES_DEPEND=
SOURCES=$(SOURCES_TARGET) $(SOURCES_DEPEND)

##

BUILDER_DEFS = \
    -DIOTDATA_SRC_LIBRARY=$(IOTDATA_SRC_LIBRARY) \
    -DIOTDATA_SRC_DEPEND_BLACKBOX=$(IOTDATA_SRC_DEPEND_BLACKBOX) \
    -DIOTDATA_SRC_EXAMPLE=$(IOTDATA_SRC_EXAMPLE) \
    -DIOTDATA_SRC_EXAMPLE_SIMULATOR=$(IOTDATA_SRC_EXAMPLE_SIMULATOR) \
    -DIOTDATA_SRC_COMMON=$(IOTDATA_SRC_COMMON) \
    -DIOTDATA_VARIANT=$(IOTDATA_VARIANT) \
    -DSIMULATE_RECEIVE_ALWAYS=$(SIMULATE_RECEIVE_ALWAYS)
ifneq ($(strip $(IOTDATA_CONFIG)),)
BUILDER_DEFS += -DIOTDATA_CONFIG=$(abspath $(IOTDATA_CONFIG))
endif

##

include $(IOTDATA_SRC_COMMON)/make/target-esp32.mk

##

