
PLATFORM=esp32c3
NAME=app
SOURCES=main/$(NAME).c
VERSION=1.00
DEVICE=/dev/ttyACM0

##

BUILDER=idf.py
BUILDER_ARGS=build
BUILDER_CFGS=build/build.ninja

TARGET=build/$(NAME).bin

##

all: $(TARGET)

$(TARGET): $(SOURCES) $(BUILDER_CFGS)
	$(BUILDER) $(BUILDER_ARGS)

$(BUILDER_CFGS):
	$(BUILDER) set-target $(PLATFORM)

upload:
	$(BUILDER) flash -p $(DEVICE)
debug:
	$(BUILDER) flash -p $(DEVICE) monitor
monitor:
	$(BUILDER) -p $(DEVICE) monitor

.PHONY: all

##

clean:
	$(BUILDER) clean
fullclean:
	rm -rf build sdkconfig sdkconfig.old
format:
	clang-format -i $(SOURCES)

.PHONY: all clean fullclean format

##

