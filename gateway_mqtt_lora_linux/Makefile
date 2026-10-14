
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
CFLAGS_INCLUDES=-I$(IOTDATA_SRC_EXAMPLE)/include -I$(IOTDATA_SRC_LIBRARY) -I$(IOTDATA_SRC_PLATFORM)/include/device -I$(IOTDATA_SRC_PLATFORM)/include -I$(IOTDATA_SRC_DEPEND_BLACKBOX) -I$(IOTDATA_SRC_DEPEND_HPATCH)
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
SOURCES_TARGET=iotdata_gateway_conf.h iotdata_gateway_ctrl.h iotdata_gateway_ddup.h iotdata_gateway_exec.h iotdata_gateway_mesh.h iotdata_gateway_mesh_survey.h \
    iotdata_gateway_mesh_trace.h iotdata_gateway_netw.h iotdata_gateway_node.h iotdata_gateway_node_content.h iotdata_gateway_state.h iotdata_gateway_stat.h \
    iotdata_gateway_util.h
SOURCES_DEPEND=\
    $(IOTDATA_SRC_PLATFORM)/include/device/d_platform_linux.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_common.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_interface_e22900t22.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_module_mqtt_linux.h \
    $(IOTDATA_SRC_PLATFORM)/include/iotdata_variant.h $(IOTDATA_VARIANT) \
    $(IOTDATA_SRC_PLATFORM)/include/iotdata_node_diagnostics.h $(IOTDATA_SRC_DEPEND_BLACKBOX)/blackbox.h \
    $(IOTDATA_SRC_PLATFORM)/include/iotdata_node_stations.h \
    $(IOTDATA_SRC_PLATFORM)/include/iotdata_node_version.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_module_buffers.h \
    $(IOTDATA_SRC_PLATFORM)/include/device/d_format.h \
    $(IOTDATA_SRC_LIBRARY_SOURCES)
SOURCES=$(SOURCES_TARGET) $(SOURCES_DEPEND)

# NOTHING EXTRA TO COMPILE. HPatchLite's engine used to be named here (and in the esp32 projects'
# SRCS) because d_module_hpatch.h is a wrapper over two entry points that live in a .c file;
# iotdata_node_otaupdate.h now compiles that .c in itself, the way iotdata_node_diagnostics.h does
# the blackbox, so every project gets the engine by including the header that assembles the update
# path. Only the include path for hpatch_lite.h is still needed, above.

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

# THE HEADER TIERS ARE CHECKED, not just documented. The include order in iotdata_gateway.c IS the
# dependency order -- see the comment above that block -- and the script asserts it still is: every
# iotdata_gateway_*.h accounted for in a tier, the order exactly as the comment claims, and no
# lower-tier header naming an identifier a higher tier defines. `test` depends on it, so the
# reorganisation that reintroduces an upward pointer fails on the spot instead of at the next one.
# PYTHON is overridable because this is the only thing here that needs an interpreter.
PYTHON ?= python3
TESTS   = tests/check_header_tiers.py

.PHONY: check
check: $(TESTS)
	@$(PYTHON) tests/check_header_tiers.py

test: check

##

