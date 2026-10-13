
##

IOTDATA_SRC=../..
include $(IOTDATA_SRC)/iotdata-depend/make/config.mk
include $(IOTDATA_SRC)/iotdata-platform/make/config.mk

IOTDATA_VARIANT=$(IOTDATA_SRC_PLATFORM)/include/iotdata_variant_weather_station.h
IOTDATA_CONFIG := $(firstword $(wildcard $(IOTDATA_SRC_EXAMPLE)/include/iotdata_config.$(IOTDATA_HOST).h iotdata_config.$(IOTDATA_HOST).h))

##

PLATFORM = linux
NAME     = gateway
VERSION  = 0.9.9

##

CC=gcc
CFLAGS_DEFINES=-DIOTDATA_VARIANT='"$(abspath $(IOTDATA_VARIANT))"'
ifneq ($(strip $(IOTDATA_CONFIG)),)
CFLAGS_DEFINES += -DIOTDATA_CONFIG='"$(abspath $(IOTDATA_CONFIG))"'
endif
CFLAGS_COMMON=$(IOTDATA_CFLAGS_COMMON)
CFLAGS_OPT=$(IOTDATA_CFLAGS_OPT)
CFLAGS_INCLUDES=-Iinclude -I$(IOTDATA_SRC_EXAMPLE)/include -I$(IOTDATA_SRC_LIBRARY) -I$(IOTDATA_SRC_PLATFORM)/include/device -I$(IOTDATA_SRC_PLATFORM)/include -I$(IOTDATA_SRC_DEPEND_BLACKBOX) -I$(IOTDATA_SRC_DEPEND_HPATCH)
CFLAGS=$(CFLAGS_COMMON) $(CFLAGS_OPT) $(CFLAGS_DEFINES) $(CFLAGS_INCLUDES)
LDFLAGS=$(IOTDATA_LDFLAGS_COMMON)
# -lz FOR THE OTA CODEC SEAM. d_module_inflate_linux.h is zlib's raw-deflate inflater, and the
# gateway includes it because every node built on the framework has an update path -- see the OTA
# seam block in iotdata_gateway.c. Nothing calls it YET (the gateway does not run the facade's
# begin), so the inline functions are unreferenced and the link would currently succeed without
# this; it is here unconditionally because the compile already needs zlib's header either way, and
# because the day the linux apply path is wired the omission would show up as a pile of undefined
# inflate* symbols rather than as the missing dependency it actually is.
LIBS=$(IOTDATA_LIBS_COMMON) -lmosquitto -lpthread -lz

##

TARGET=iotdata_gateway
MAIN=iotdata_gateway.c
SOURCES_TARGET=iotdata_gateway_conf.h iotdata_gateway_ctrl.h iotdata_gateway_ddup.h iotdata_gateway_exec.h iotdata_gateway_mesh.h iotdata_gateway_netw.h iotdata_gateway_node.h \
    iotdata_gateway_peer.h iotdata_gateway_state.h iotdata_gateway_stat.h iotdata_gateway_survey.h iotdata_gateway_trace.h iotdata_gateway_util.h \
    include/mqtt_linux.h
SOURCES_DEPEND=\
    $(IOTDATA_SRC_PLATFORM)/include/device/d_platform_linux.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_common.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_interface_e22900t22.h \
    $(IOTDATA_SRC_PLATFORM)/include/iotdata_variant.h $(IOTDATA_VARIANT) \
    $(IOTDATA_SRC_PLATFORM)/include/iotdata_node_diagnostics.h $(IOTDATA_SRC_DEPEND_BLACKBOX)/blackbox.h \
    $(IOTDATA_SRC_PLATFORM)/include/iotdata_node_stations.h \
    $(IOTDATA_SRC_PLATFORM)/include/iotdata_node_version.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_module_buffers.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_format.h \
    $(IOTDATA_SRC_LIBRARY_SOURCES)
SOURCES=$(SOURCES_TARGET) $(SOURCES_DEPEND)

# THE ONE SEAM THAT IS NOT HEADER-ONLY. d_module_hpatch.h is a wrapper over HPatchLite's two
# entry points, and they live in a .c file -- so including the header is not enough, the engine
# has to be compiled in. The esp32 projects say the same thing in their CMakeLists SRCS.
#
# Why this was not noticed until a -O0 build: at -O3 gcc never emits an unreferenced static
# function, and nothing calls the patcher yet (the gateway does not run the facade's begin), so
# _patch_apply was optimised away and its calls to hpatch_lite_open/_patch went with it. Drop the
# optimiser -- `make CFLAGS_OPT=`, which is what a debug build does -- and the function appears,
# along with two undefined references. The dependency was always real; -O3 was hiding it.
SOURCES_EXTRA=$(IOTDATA_SRC_DEPEND_HPATCH)/hpatch_lite.c

TEST_MAIN=iotdata_gateway_test.c

# zlib1g-dev IS A REAL BUILD DEPENDENCY, not an optional extra: d_module_inflate_linux.h includes
# <zlib.h> unconditionally, so a box without it fails at the #include rather than at the link. It
# went unnoticed here because zlib's headers were already installed on the build host and are not
# on a fresh one -- the gateway stopped building on a clean box while `make install-dev` reported
# everything already present, which is the failure this line exists to prevent.
PACKAGES_DEV=libmosquitto-dev libcjson-dev zlib1g-dev
INSTALL=iotdata-gateway

##

include $(IOTDATA_SRC_PLATFORM)/make/target-linux.mk

##

