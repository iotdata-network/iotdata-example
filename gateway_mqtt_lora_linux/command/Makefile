
##
## iotdata gateway command — install only
##

TARGET=iotdata_gateway_command
SCRIPT=$(TARGET).js

all: node_modules
	@echo "nothing to build ($(SCRIPT) is a script); use 'make install'"

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
