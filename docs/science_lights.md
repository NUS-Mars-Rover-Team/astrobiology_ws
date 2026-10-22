# Science lights setup — 2026-10-22

ROS 2 node (`lights_node`) for controlling four science lights via a dedicated ESP32 over serial.
The code is on branch `feat/led_integration`.

For the node source, see
[`src/astrobiology_actuators/astrobiology_actuators/lights_node.py`](../src/astrobiology_actuators/astrobiology_actuators/lights_node.py).

## Status

| Item | Status |
|---|---|
| ESP32 firmware (`science_lights_simple.ino`) | Done |
| ROS 2 node (`lights_node`) | Done, not deployed to Pi yet |
| Config / launch / tests | Done (6 tests passing) |
| udev rule for `/dev/esp_lights` | **Open:** needs vendor/product IDs or serial number |
| On-Pi deployment and hardware test | **Open** |

## Hardware

The lights ESP32 is a separate board from the stepper ESP32. It drives four GPIO pins
HIGH/LOW to switch lights on and off.

| Light | Firmware name | GPIO | Use case |
|---|---|---|---|
| UV LED 1 | UV1 | 25 | UV fluorescence excitation |
| UV LED 2 | UV2 | 26 | UV fluorescence excitation |
| Laser | LAS | 27 | Spectroscope line source / alignment |
| Halogen | HAL | 33 | Broadband illumination for absorption spectra |

All outputs are LOW (off) at boot. The ESP32 runs at 115200 baud.

## Serial protocol

Send a line `<NAME> <0|1>\n`. The ESP32 echoes back `<NAME> <val>\n` on success.
Unrecognized names are silently dropped.

```
UV1 1      → turn UV LED 1 on     → echo: UV1 1
LAS 0      → turn laser off       → echo: LAS 0
```

Commands are instantaneous (`digitalWrite`), so there is no busy state or completion feedback.

## ROS 2 architecture

### Node: `lights_node`

**Package:** `astrobiology_actuators`

**Parameters** (from `config/lights.yaml`):

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/esp_lights` | Serial device |
| `baud` | `115200` | Baud rate |
| `dry_run` | `false` | Log commands instead of sending serial |

### Services

| Service | Type | Description |
|---|---|---|
| `/astrobiology/uv1/set` | `std_srvs/SetBool` | UV LED 1 on/off |
| `/astrobiology/uv2/set` | `std_srvs/SetBool` | UV LED 2 on/off |
| `/astrobiology/laser/set` | `std_srvs/SetBool` | Laser on/off |
| `/astrobiology/halogen/set` | `std_srvs/SetBool` | Halogen lamp on/off |

`data: true` = ON, `data: false` = OFF. Response always returns `success: true` with the
command string as the message (e.g. `"LAS 1"`).

On shutdown the node sends OFF for all four lights, then closes the serial port.

## Files

| File | Purpose |
|---|---|
| `astrobiology_actuators/lights_commands.py` | Pure-Python protocol helpers (`LIGHTS` dict, `light_command()`, `all_off_commands()`) |
| `astrobiology_actuators/lights_node.py` | ROS 2 node — serial bridge with 4 SetBool services |
| `config/lights.yaml` | Node parameters |
| `launch/lights.launch.py` | Launch file with `config` and `dry_run` arguments |
| `test/test_lights_node.py` | Unit tests for the protocol helpers (no ROS needed) |

## Usage

### Launch

```bash
# Live (requires /dev/esp_lights)
ros2 launch astrobiology_actuators lights.launch.py

# Dry run (no hardware)
ros2 launch astrobiology_actuators lights.launch.py dry_run:=true
```

### Control lights

```bash
ros2 service call /astrobiology/laser/set std_srvs/srv/SetBool "{data: true}"
ros2 service call /astrobiology/uv1/set std_srvs/srv/SetBool "{data: false}"
```

### Direct serial debugging

```bash
screen /dev/esp_lights 115200
```

Type `LAS 1` to turn the laser on, `HAL 0` to turn the halogen off. Exit with `Ctrl+A`, `K`, `Y`.

## udev rule

Both ESP32s may use the same USB-serial chip (e.g. CP2102), so a udev rule must distinguish
them. Options:

1. **By USB port path** (if they always plug into the same physical port):
   ```
   SUBSYSTEM=="tty", ENV{ID_PATH}=="*-1.2:1.0", SYMLINK+="esp_lights", MODE="0666"
   ```
   Find the path with `udevadm info -q property /dev/ttyUSB1 | grep ID_PATH`.

2. **By chip serial number** (if the chips have unique serials):
   ```
   SUBSYSTEM=="tty", ATTRS{serial}=="XXXXXXXX", SYMLINK+="esp_lights", MODE="0666"
   ```
   Find the serial with `udevadm info -a /dev/ttyUSB1 | grep serial`.

Add the chosen rule to `/etc/udev/rules.d/99-astrobiology.rules` and reload:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Open items

- [ ] Deploy to the Pi: push the branch, build on the Pi, create the udev rule.
- [ ] Fill in the actual udev rule (port path or chip serial) once the ESP32 is wired up.
- [ ] Hardware test: verify all four lights toggle correctly via ROS services.
- [ ] Integrate with the spectrometer workflow (e.g. turn halogen on before taking a blank).
