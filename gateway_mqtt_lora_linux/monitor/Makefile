
##
## iotdata gateway monitor — install only; the target is a script, so there is nothing to build.
##

TARGET=iotdata_gateway_monitor
SCRIPT=$(TARGET).js

all:
	@echo "nothing to build ($(SCRIPT) is a script); use 'make install'"

check: $(SCRIPT)
	node --check $(SCRIPT)
	@echo "syntax OK"

format:
	@command -v prettier >/dev/null && prettier --write $(SCRIPT) || echo "prettier not installed, skipping"

.PHONY: all check format

##

# Runtime dependencies rather than build ones: node runs the script, mosquitto-clients
# provides the mosquitto_sub this subscribes with. Named to match the other components.
DEV_PACKAGES=nodejs mosquitto-clients
install-dev:
	apt install -y $(DEV_PACKAGES)
remove-dev:
	apt purge -y $(DEV_PACKAGES)
.PHONY: install-dev remove-dev

##

INSTALL=iotdata-gateway-monitor
DIR_INSTALL=/usr/local/bin
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
install_target: $(SCRIPT)
	install -m 755 $(SCRIPT) $(DIR_INSTALL)/$(INSTALL)
install_default: $(CFG_SRC)
	@echo "installing config from $(CFG_SRC)"
	install -m 644 $(CFG_SRC) $(DIR_DEFAULT)/$(INSTALL)
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
	systemctl daemon-reload
	@echo "left in place: $(DIR_DEFAULT)/$(INSTALL)"
.PHONY: install install_target install_default install_service
.PHONY: restart status uninstall

##
