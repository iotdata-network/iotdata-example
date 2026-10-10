
##
## iotdata gateway control — install only
##

TARGET=iotdata_gateway_control
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
	@$(MAKE) --no-print-directory check-vocabulary

# Every command this script offers must exist in iotdata_node_control.h, because THAT is what the
# gateway resolves a `cmd` against -- a name only this side knows travels over MQTT and comes back
# "unknown command". Four names are local by design: `raw` never leaves this script, and conf-set,
# survey and survey-cancel are handled in the gateway's own ctrl_from_message rather than through
# the control table. The header is absent in an installed copy, so a missing one skips the check
# rather than failing it.
VOCABULARY=../../../iotdata-common/include/iotdata_node_control.h
VOCABULARY_LOCAL=conf-set survey survey-cancel raw
check-vocabulary: $(SCRIPT)
	@if [ ! -f $(VOCABULARY) ]; then echo "no $(VOCABULARY), skipping vocabulary check"; exit 0; fi; \
	grep -oE "^[[:space:]]+\{ \"[a-z-]+\"" $(VOCABULARY) | tr -d '{" \t' | sort -u > .vocab.c; \
	grep -oE "^[[:space:]]+'[a-z-]+':" $(SCRIPT) | tr -d "': \t" | sort -u > .vocab.js; \
	for n in $(VOCABULARY_LOCAL); do echo $$n; done | sort -u >> .vocab.c; \
	sort -u -o .vocab.c .vocab.c; \
	dead=`comm -23 .vocab.js .vocab.c`; \
	rm -f .vocab.c .vocab.js; \
	if [ -n "$$dead" ]; then echo "vocabulary: offered here but unknown to the gateway:"; echo "$$dead" | sed 's/^/  /'; exit 1; fi; \
	echo "vocabulary OK"
.PHONY: check-vocabulary

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

INSTALL=iotdata-gateway-control
DIR_INSTALL=/usr/local/bin
DIR_LIB=/usr/local/lib/iotdata-gateway-control
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
