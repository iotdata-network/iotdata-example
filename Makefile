# iotdata-example — top-level build aggregator for the worked examples.
#
# Each example is a self-contained subdir with its own Makefile:
#   simulator                    native (gcc)
#   gateway_mqtt_lora_linux      native (gcc)
#   simulator_sensor_lora_esp32  esp32 — needs the ESP-IDF toolchain (idf.py)
#
# Dependencies are pulled in by the sub-Makefiles via absolute path:
#   /opt/iotdata-library            the iotdata library (headers + source)
#   /opt/iotdata-depend/e22900t22   the E22 LoRa driver (gateway example)
#
# The esp32 build needs ESP-IDF, so it is kept out of the default `all` and
# built on request (`make embedded` / `make everything`).

NATIVE   = simulator gateway_mqtt_lora_linux
EMBEDDED = simulator_sensor_lora_esp32

.DEFAULT_GOAL := all
.PHONY: all native embedded everything clean help $(NATIVE) $(EMBEDDED)

all: native ## Build the native (gcc) examples — the default

native: $(NATIVE) ## Build simulator + gateway_mqtt_lora_linux (gcc)

embedded: $(EMBEDDED) ## Build the esp32 example (needs ESP-IDF / idf.py)

everything: native embedded ## Build every example (native + esp32)

# build one example by recursing into its own Makefile
$(NATIVE) $(EMBEDDED):
	$(MAKE) -C $@

clean: ## Clean native artifacts + the esp32 build/ dir
	@for d in $(NATIVE); do $(MAKE) -C $$d clean; done
	rm -rf $(EMBEDDED)/build

help: ## Show this help
	@echo "iotdata-example — worked examples (plain 'make' builds the native ones)"
	@echo
	@awk 'BEGIN{FS=":.*## "} /^[a-zA-Z_-]+:.*## /{printf "  make %-11s %s\n",$$1,$$2}' $(MAKEFILE_LIST)
