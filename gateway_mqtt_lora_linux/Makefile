
##

IOTDATA_SRC=../..
include $(IOTDATA_SRC)/iotdata-common/make/config.mk
include $(IOTDATA_SRC)/iotdata-depend/make/config.mk

# IOTDATA_VARIANT=$(IOTDATA_SRC_COMMON)/include/iotdata_variant_weather_station.h
IOTDATA_CONFIG := $(firstword $(wildcard $(IOTDATA_SRC_EXAMPLE)/include/iotdata_config.$(IOTDATA_HOST).h iotdata_config.$(IOTDATA_HOST).h))

##

CC=gcc
CFLAGS_DEFINES=-DIOTDATA_VARIANT='"$(abspath $(IOTDATA_VARIANT))"'
ifneq ($(strip $(IOTDATA_CONFIG)),)
CFLAGS_DEFINES += -DIOTDATA_CONFIG='"$(abspath $(IOTDATA_CONFIG))"'
endif
CFLAGS_COMMON=$(IOTDATA_CFLAGS_COMMON)
CFLAGS_OPT=$(IOTDATA_CFLAGS_OPT)
CFLAGS_INCLUDES=-Iinclude -I$(IOTDATA_SRC_EXAMPLE)/include -I$(IOTDATA_SRC_LIBRARY) -I$(IOTDATA_SRC_COMMON)/include/device -I$(IOTDATA_SRC_COMMON)/include -I$(IOTDATA_SRC_DEPEND_BLACKBOX)
CFLAGS=$(CFLAGS_COMMON) $(CFLAGS_OPT) $(CFLAGS_DEFINES) $(CFLAGS_INCLUDES)
LDFLAGS=$(IOTDATA_LDFLAGS_COMMON)
LIBS=$(IOTDATA_LIBS_COMMON) -lmosquitto -lpthread

##

TARGET=iotdata_gateway
MAIN=iotdata_gateway.c
SOURCES_TARGET=\
    iotdata_gateway_ddup.h iotdata_gateway_exec.h iotdata_gateway_mesh.h iotdata_gateway_ctrl.h iotdata_gateway_netw.h iotdata_gateway_node.h iotdata_gateway_stat.h \
    include/config_linux.h include/mqtt_linux.h
SOURCES_DEPEND=\
    $(IOTDATA_SRC_COMMON)/include/device/d_platform_linux.h \
    $(IOTDATA_SRC_COMMON)/include/device/d_common.h \
    $(IOTDATA_SRC_COMMON)/include/device/d_interface_e22900t22.h \
    $(IOTDATA_SRC_COMMON)/include/iotdata_variant.h $(IOTDATA_VARIANT) \
    $(IOTDATA_SRC_COMMON)/include/iotdata_blackbox.h $(IOTDATA_SRC_DEPEND_BLACKBOX)/blackbox.h \
    $(IOTDATA_SRC_COMMON)/include/iotdata_station_filter.h \
    $(IOTDATA_SRC_LIBRARY_SOURCES)
SOURCES=$(SOURCES_TARGET) $(SOURCES_DEPEND)

TEST_MAIN=iotdata_gateway_test.c

PACKAGES_DEV=libmosquitto-dev libcjson-dev
INSTALL=iotdata-gateway

##

include $(IOTDATA_SRC_COMMON)/make/target-linux.mk

##

