
##
## iotdata gateway monitor — install only; the target is a script, so there is nothing to build.
##

TARGET=iotdata_gateway_monitor
SCRIPT=$(TARGET).js

all: node_modules
	@echo "nothing to build ($(SCRIPT) is a script); use 'make install'"

# The single dependency, pinned to the version the apex tier (lib/package.json) already uses.
node_modules: package.json
	npm install --omit=dev
	@touch node_modules
deps: node_modules
.PHONY: deps

check: $(SCRIPT)
	node --check $(SCRIPT)
	@echo "syntax OK"

format:
	@command -v prettier >/dev/null && prettier --write $(SCRIPT) || echo "prettier not installed, skipping"

.PHONY: all check format

##

# Runtime dependencies rather than build ones: node runs the script and npm fetches the one
# module it needs. mosquitto-clients is no longer among them — the monitor speaks MQTT itself
# now — though mosquitto_pub remains handy for poking at the broker by hand.
DEV_PACKAGES=nodejs npm
install-dev:
	apt install -y $(DEV_PACKAGES)
remove-dev:
	apt purge -y $(DEV_PACKAGES)
.PHONY: install-dev remove-dev

##

INSTALL=iotdata-gateway-monitor
DIR_INSTALL=/usr/local/bin
DIR_LIB=/usr/local/lib/iotdata-gateway-monitor
DIR_DEFAULT=/etc/default
DIR_SYSTEMD=/etc/systemd/system
# Prefer a per-host config ($(TARGET).<hostname>.cfg) over the shared default,
# matching the convention used by the gateway, hostmon and connmon.
HOSTNAME:=$(shell hostname)
CFG_SRC:=$(if $(wildcard $(TARGET).$(HOSTNAME).cfg),$(TARGET).$(HOSTNAME).cfg,$(TARGET).cfg)
define install_service_systemd
	-systemctl stop $(2) 2>/dev/null || true
	-systemctl disable $(2) 2>/dev/null || true
	install -m 644 $(1).service $(DIR_SYSTEMD)/$(2).service
	systemctl daemon-reload
	systemctl enable $(2)
	systemctl start $(2) || echo "Warning: Failed to start $(2)"
endef
# The script needs its node_modules beside it, so it is installed into a directory of its own
# and symlinked onto the path rather than dropped straight into $(DIR_INSTALL). Node resolves
# the entry script's real path before it looks for modules, so running the symlink still finds
# $(DIR_LIB)/node_modules.
install_target: $(SCRIPT) node_modules
	install -d $(DIR_LIB)
	install -m 755 $(SCRIPT) $(DIR_LIB)/$(INSTALL)
	rm -rf $(DIR_LIB)/node_modules
	cp -a node_modules $(DIR_LIB)/node_modules
	ln -sfn $(DIR_LIB)/$(INSTALL) $(DIR_INSTALL)/$(INSTALL)
# 600, not 644: the config carries the LoRaWAN network server API key the tracker
# subscription authenticates with, and this file is otherwise world-readable.
install_default: $(CFG_SRC)
	@echo "installing config from $(CFG_SRC)"
	install -m 600 $(CFG_SRC) $(DIR_DEFAULT)/$(INSTALL)
install_service: $(TARGET).service
	$(call install_service_systemd,$(TARGET),$(INSTALL))
install: install_target install_default install_service
restart:
	systemctl restart $(INSTALL)
status:
	systemctl status $(INSTALL) --no-pager
uninstall:
	-systemctl stop $(INSTALL)
	-systemctl disable $(INSTALL)
	rm -f $(DIR_SYSTEMD)/$(INSTALL).service $(DIR_INSTALL)/$(INSTALL)
	rm -rf $(DIR_LIB)
	systemctl daemon-reload
	@echo "left in place: $(DIR_DEFAULT)/$(INSTALL)"
.PHONY: install install_target install_default install_service
.PHONY: restart status uninstall

##
