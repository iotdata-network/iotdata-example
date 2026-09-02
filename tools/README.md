# iotdata-example/tools

Host-side helpers for the ESP32 examples. No compiler/IDF toolchain required.

## `esp32-deploy`

Flash a **pre-built** ESP-IDF image and/or monitor an ESP32 over USB serial, on a
host that has **no IDF toolchain** — built for the split workflow where you build on a
fast toolchain host and flash/watch on the host the boards are plugged into.

```
esp32-deploy flash    -p PORT -b BUILD_DIR [--pull HOST:PATH]   # write image to chip
esp32-deploy monitor  -p PORT [command]                          # reset + print console
esp32-deploy deploy   -p PORT -b BUILD_DIR [--pull HOST:PATH]    # flash, then monitor
```

Everything the flash needs (chip, flash settings, the three binaries and their
offsets) is read from the build dir's `flasher_args.json`, so it works for any
project without hardcoding anything. `monitor` is the same colored-log / reconnect /
command-inject loop as `iotdata-device/sds/tools/esp32-boot` — this tool is a superset.

### Typical two-host flow

Build on the toolchain host (fast):

```sh
# on workshop
cd iotdata-example/simulator_sensor_lora_esp32 && make
```

Flash + watch on the target host where the boards live (e.g. `iotdata-tst-3`):

```sh
# on iotdata-tst-3, board on ttyACM0 — artifacts already local (shared FS / copied):
esp32-deploy deploy -p /dev/ttyACM0 -b ../simulator_sensor_lora_esp32/build

# or pull the binaries straight off the build host, then flash + monitor:
esp32-deploy deploy -p /dev/ttyACM1 \
    --pull workshop:/opt/iotdata/src/iotdata-example/simulator_sensor_lora_esp32/build
```

### Dependencies on the target host

None of these is the compiler toolchain:

| For | Needs | Install |
|---|---|---|
| `monitor` | python3, pyserial | already present (used by `esp32-boot`) |
| `flash` | esptool | `pip install esptool` |
| `--pull` | rsync + ssh | usually already present |

### Notes

- **Two boards / stable ports.** `ttyACM` numbering can move across a reset or
  re-enumeration. To pin a physical board, address it by a stable path:
  `-p /dev/serial/by-path/<...>` (see `ls -l /dev/serial/by-path`).
- **Port default** is `$ESP32_PORT` then `/dev/ttyACM0`.
- Exit the monitor with **Ctrl-C**.
