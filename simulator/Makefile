
include ../../iotdata-common/make/config.mk

DIR_IOTDATA=$(IOTDATA_SRC_LIBRARY)
DIR_IOTDATA_VARIANT=$(IOTDATA_SRC_EXAMPLE_COMMON)
DIR_SIMULATOR=.

##

CC=gcc
CFLAGS_DEFINES=
CFLAGS_COMMON=-Wall -Wextra -Wpedantic
CFLAGS_STRICT=-Werror \
    -Wstrict-prototypes \
    -Wold-style-definition \
    -Wcast-align -Wcast-qual -Wconversion \
    -Wfloat-equal -Wformat=2 -Wformat-security \
    -Winit-self -Wjump-misses-init \
    -Wlogical-op -Wmissing-include-dirs \
    -Wnested-externs -Wpointer-arith \
    -Wredundant-decls -Wshadow \
    -Wstrict-overflow=2 -Wswitch-default \
    -Wundef \
    -Wunreachable-code -Wunused \
    -Wwrite-strings
CFLAGS_OPT=-O3
CFLAGS_INCLUDES=-I$(DIR_IOTDATA) -I$(DIR_SIMULATOR) -I$(DIR_IOTDATA_VARIANT)
CFLAGS= $(CFLAGS_COMMON) $(CFLAGS_STRICT) $(CFLAGS_DEFINES) $(CFLAGS_OPT) $(CFLAGS_INCLUDES)
LDFLAGS=
LIBS=-lm

##

TARGET=simulator
MAIN=$(DIR_SIMULATOR)/iotdata_variant_simulator.c
SOURCES=$(DIR_SIMULATOR)/iotdata_variant_simulator.h \
	$(DIR_IOTDATA_VARIANT)/iotdata_variant_suite.h \
	$(DIR_IOTDATA)/iotdata.h $(DIR_IOTDATA)/iotdata.c

##

all: $(TARGET)

$(TARGET): $(MAIN) $(SOURCES)
	$(CC) $(CFLAGS) -DTEST_MAIN -o $(TARGET) $(MAIN) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(TARGET)
format:
	clang-format -i $(MAIN) $(SOURCES)

.PHONY: all clean

##

