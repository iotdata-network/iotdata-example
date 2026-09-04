##
## iotdata gateway command — install only; the target is a script, so there is nothing to build.
##

TARGET=iotdata_gateway_command
SCRIPT=$(TARGET).js

all: node_modules
	@echo "nothing to build ($(SCRIPT) is a script); use 'make install'"

# The single dependency, the same one the monitor beside it uses.
node_modules: package.json
	npm install --omit=dev
	@touch node_modules
deps: node_modules
.PHONY: deps

# mqtt is require()d lazily inside main(), so --help, --dry-run and argument errors all work
# before node_modules exists; this checks the script itself, not the install.
check: $(SCRIPT)
	node --check $(SCRIPT)
	@echo "syntax OK"

format:
	@command -v prettier >/dev/null && prettier --write $(SCRIPT) || echo "prettier not installed, skipping"

.PHONY: all check format

##

# Runtime dependencies rather than build ones: node runs the script and npm fetches the one
# module it needs.
DEV_PACKAGES=nodejs npm
install-dev:
	apt install -y $(DEV_PACKAGES)
remove-dev:
	apt purge -y $(DEV_PACKAGES)
.PHONY: install-dev remove-dev

##

INSTALL=iotdata-gateway-command
DIR_INSTALL=/usr/local/bin
DIR_LIB=/usr/local/lib/iotdata-gateway-command
# No config and no service, unlike the gateway and the monitor: this is a one-shot CLI rather
# than a daemon. It takes everything from its arguments (or MQTT_BROKER / IOTDATA_PREFIX in the
# environment), so there is no $(DIR_DEFAULT) file to install and nothing for systemd to run.
#
# The script needs its node_modules beside it, so it is installed into a directory of its own
# and symlinked onto the path rather than dropped straight into $(DIR_INSTALL). Node resolves
# the entry script's real path before it looks for modules, so running the symlink still finds
# $(DIR_LIB)/node_modules. That is a second copy of mqtt alongside the monitor's; at a few MB
# it is cheaper than coupling the two installs together.
install_target: $(SCRIPT) node_modules
	install -d $(DIR_LIB)
	install -m 755 $(SCRIPT) $(DIR_LIB)/$(INSTALL)
	rm -rf $(DIR_LIB)/node_modules
	cp -a node_modules $(DIR_LIB)/node_modules
	ln -sfn $(DIR_LIB)/$(INSTALL) $(DIR_INSTALL)/$(INSTALL)
install: install_target
uninstall:
	rm -f $(DIR_INSTALL)/$(INSTALL)
	rm -rf $(DIR_LIB)
.PHONY: install install_target uninstall

##
