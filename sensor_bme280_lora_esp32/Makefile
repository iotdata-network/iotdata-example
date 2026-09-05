
##

IOTDATA_SRC=../..
include $(IOTDATA_SRC)/iotdata-common/make/config.mk
include $(IOTDATA_SRC)/iotdata-depend/make/config.mk

IOTDATA_CONFIG := $(firstword $(wildcard $(IOTDATA_SRC_EXAMPLE)/include/iotdata_config.$(IOTDATA_HOST).h iotdata_config.$(IOTDATA_HOST).h))

##

PLATFORM = esp32c3
NAME     = app
SOURCES  = main/$(NAME).c
VERSION  = 1.00

##

BUILDER_DEFS = \
    -DIOTDATA_SRC_LIBRARY=$(IOTDATA_SRC_LIBRARY) \
    -DIOTDATA_SRC_DEPEND_E22XXXTXX=$(IOTDATA_SRC_DEPEND_E22XXXTXX) \
    -DIOTDATA_SRC_EXAMPLE=$(IOTDATA_SRC_EXAMPLE) \
    -DIOTDATA_SRC_COMMON=$(IOTDATA_SRC_COMMON) \
    -DIOTDATA_VARIANT=$(IOTDATA_VARIANT)
ifneq ($(strip $(IOTDATA_CONFIG)),)
BUILDER_DEFS += -DIOTDATA_CONFIG=$(abspath $(IOTDATA_CONFIG))
endif

##

include ../../iotdata-common/make/target-esp32.mk

##

