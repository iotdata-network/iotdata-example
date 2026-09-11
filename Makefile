# iotdata-example — top-level build aggregator for the worked examples.
#
# Each example is a self-contained subdir with its own Makefile:
#   simulator                    native (gcc)
#   gateway_mqtt_lora_linux      native (gcc)
#   simulator_sensor_lora_esp32  esp32 — needs the ESP-IDF toolchain (idf.py)
#
# Where the iotdata sources live is defined once in iotdata-common/make/config.mk
# and shared with every sub-Makefile across all repos (and, for the esp32 build,
# passed through to CMake). IOTDATA_SRC defaults to $(IOTDATA_APEX)/src, with the
# apex self-located from config.mk; override IOTDATA_SRC or IOTDATA_APEX (env or
# command line) to relocate. `make config` prints the resolved paths.
#
# The esp32 build needs ESP-IDF, so it is kept out of the default `all` and
# built on request (`make embedded` / `make everything`).

include ../iotdata-common/make/config.mk

NATIVE   = simulator gateway_mqtt_lora_linux
EMBEDDED = simulator_sensor_lora_esp32 sensor_bme280_lora_esp32

.DEFAULT_GOAL := all
.PHONY: all native embedded everything clean config help $(NATIVE) $(EMBEDDED)

all: native

native: $(NATIVE)

embedded: $(EMBEDDED)

everything: native embedded

$(NATIVE) $(EMBEDDED):
	$(MAKE) -C $@

clean:
	@for d in $(NATIVE); do $(MAKE) -C $$d clean done
	@for d in $(EMBEDDED); do $(MAKE) -C $$d clean; done

format:
	@for d in $(NATIVE) $(EMBEDDED); do $(MAKE) -C $$d format; done
	@clang-format-19 -i include/*.h

config:
	@echo "IOTDATA_APEX                  = $(IOTDATA_APEX)"
	@echo "IOTDATA_SRC                   = $(IOTDATA_SRC)"
	@echo "IOTDATA_SRC_LIBRARY           = $(IOTDATA_SRC_LIBRARY)"
	@echo "IOTDATA_SRC_DEPEND            = $(IOTDATA_SRC_DEPEND)"
	@echo "IOTDATA_SRC_COMMON            = $(IOTDATA_SRC_COMMON)"
	@echo "IOTDATA_SRC_EXAMPLE           = $(IOTDATA_SRC_EXAMPLE)"
	@echo "IOTDATA_SRC_EXAMPLE_SIMULATOR = $(IOTDATA_SRC_EXAMPLE_SIMULATOR)"
	@echo "IOTDATA_VARIANT               = $(IOTDATA_VARIANT)"

help:
	@echo "iotdata-example — worked examples (plain 'make' builds the native ones)"
	@echo
	@awk 'BEGIN{FS=":.*## "} /^[a-zA-Z_-]+:.*## /{printf "  make %-11s %s\n",$$1,$$2}' $(MAKEFILE_LIST)
