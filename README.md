# Astrobiology Payload ROS 2 Workspace

ROS 2 (Humble) workspace for the NUS Mars Rover astrobiology payload. Controls stepper motors (pumps, gantry, centrifuge) and a USB spectroscope camera via a Raspberry Pi.

## Architecture

![Architecture](docs/architecture.dot)

Render: `dot -Tpng docs/architecture.dot -o docs/architecture.png` or paste into [GraphvizOnline](https://dreampuf.github.io/GraphvizOnline).

## Hardware

| Device | Interface | udev alias |
|--------|-----------|------------|
| ESP32 6-channel stepper controller | Serial 115200 (g1 protocol) | `/dev/esp_steppers` |
| Sonix spectroscope camera | V4L2 (1920x1080 MJPG) | `/dev/spectroscope` |

### ESP32 Motor Channels

| Channel | Firmware name | Short | STEP | DIR | EN |
|---------|--------------|-------|------|-----|----|
| Pump | pump | p | 2 | 4 | 15 |
| Flush | flush | f | 18 | 19 | 15 |
| Centrifuge | centrifuge | c | 16 | 17 | 15 |
| X gantry | x | x | 32 | 33 | 5 |
| Y gantry | y | y | 26 | 27 | 25 |
| Z gantry | z | z | 12 | 13 | 14 |

## Packages

| Package | Description |
|---------|-------------|
| `astrobiology_actuators` | Stepper motor node (pump, flush, centrifuge, gantry) |
| `spectro_cam` | USB spectrometer node (capture, process, publish spectra) |
| `astrobiology_interfaces` | Service definitions (`StepperMove.srv`, etc.) |
| `astrobiology_msgs` | Legacy message/service definitions (`Spectrum.msg`, `MotorStatus.msg`) |
| `astrobiology_bringup` | Legacy motor bridge node |

## Setup (Raspberry Pi)

### Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- Python: `numpy`, `scipy`, `opencv-python`, `pyserial`

### Build

```bash
cd ~/astrobiology_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### udev rules

Create `/etc/udev/rules.d/99-astrobiology.rules`:

```
# ESP32 stepper controller (CP2102)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="esp_steppers", MODE="0666"

# Sonix spectroscope camera
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="spectroscope", MODE="0666"
```

Then reload:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Startup

### Launch all stepper motors

```bash
ros2 launch astrobiology_actuators pumps.launch.py
```

This starts `pump_node` with 12 services (move + stop for each of 6 motors).

### Launch spectrometer

```bash
ros2 launch spectro_cam spectro_cam.launch.py
```

### Live spectrum plot (diagnostic)

Runs on any machine on the same network as the Pi (same `ROS_DOMAIN_ID=67` and
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`; build `astrobiology_msgs` and `spectro_cam` there too):

```bash
ros2 run spectro_cam spectrum_plot            # x-axis in nm
ros2 run spectro_cam spectrum_plot --pixels   # x-axis in ROI columns, for calibration
```

## Usage

### Stepper motor control

All motors use the `StepperMove` service (`steps`, `speed_hz`, `accel_hz2`):

```bash
# Move pump 3200 steps forward at 800 steps/s, 400 steps/s² accel
ros2 service call /astrobiology/pump/move \
  astrobiology_interfaces/srv/StepperMove "{steps: 3200, speed_hz: 800, accel_hz2: 400}"

# Move gantry X axis 1000 steps in reverse
ros2 service call /astrobiology/x_gantry/move \
  astrobiology_interfaces/srv/StepperMove "{steps: -1000, speed_hz: 1000, accel_hz2: 500}"

# Stop a motor
ros2 service call /astrobiology/pump/stop std_srvs/srv/Trigger
```

### Available services

| Service | Type | Description |
|---------|------|-------------|
| `/astrobiology/pump/move` | StepperMove | Peristaltic pump |
| `/astrobiology/pump/stop` | Trigger | Stop pump |
| `/astrobiology/flush/move` | StepperMove | Flush pump |
| `/astrobiology/flush/stop` | Trigger | Stop flush |
| `/astrobiology/centrifuge/move` | StepperMove | Centrifuge motor |
| `/astrobiology/centrifuge/stop` | Trigger | Stop centrifuge |
| `/astrobiology/x_gantry/move` | StepperMove | Gantry X axis |
| `/astrobiology/x_gantry/stop` | Trigger | Stop X axis |
| `/astrobiology/y_gantry/move` | StepperMove | Gantry Y axis |
| `/astrobiology/y_gantry/stop` | Trigger | Stop Y axis |
| `/astrobiology/z_gantry/move` | StepperMove | Gantry Z axis |
| `/astrobiology/z_gantry/stop` | Trigger | Stop Z axis |
| `/spectro/spectrum_node/set_zero_reference` | Trigger | Snapshot the current spectrum as the blank (I0) |
| `/spectro/spectrum_node/clear_zero_reference` | Trigger | Clear the blank; back to raw spectra |

### Spectrometer measurements

The wavelength calibration and the camera settings for the 5 V halogen lamp are in
[`src/spectro_cam/config/spectro.yaml`](src/spectro_cam/config/spectro.yaml) (±1.5 nm over
404–611 nm). The full calibration record, accuracy figures and open items are in
[`docs/spectrometer_calibration.md`](docs/spectrometer_calibration.md).

To measure an absorption spectrum (e.g. a dye in a cuvette):

```bash
# 1. Halogen lamp on, water blank in the cuvette holder; raw combined level should be ~0.2
ros2 service call /spectro/spectrum_node/clear_zero_reference std_srvs/srv/Trigger
sleep 2   # let the averaging buffer refill with the blank
ros2 service call /spectro/spectrum_node/set_zero_reference std_srvs/srv/Trigger
# 2. Swap in the sample without moving the lamp or the holder
```

With `reference_mode: absorbance` (the default in `spectro.yaml`), `/spectro/spectrum` then
carries absorbance, `-log10(I/I0)`. Absorption bands show up as positive peaks at their true
wavelength (in `peak_wavelengths`), and their height scales with concentration. Set
`reference_mode subtract` for the spectro-cam-rs behaviour (`I - I0`).

Things to keep in mind:
- **Retake the blank** after changing the lamp, the exposure or the cuvette position.
- **Line sources (CFL, lasers):** clear the blank and drop to exposure ~50 at 30 fps, or they
  saturate.
- **Usable range with the halogen lamp:** ~500–900 nm. There's very little light below 470 nm.
- **Use only `combined`:** the R/G/B channels don't separate colour on this camera.
- **Live parameter changes aren't saved:** `ros2 param set` changes the running node only and is
  lost on restart. Copy values into `spectro.yaml` on the Pi to keep them.

### StepperMove.srv

```
int32 steps       # signed step count (+ forward, - reverse)
int32 speed_hz    # 1..50000 pulses/s
int32 accel_hz2   # 1..1000000 pulses/s²
---
bool  success
string message
```

### Step calculations (1.8 deg/step motor, 16 microstep)

- 3200 steps = 1 full revolution
- 8.89 steps = 1 degree
- 44 steps = 5 degrees
- 533 steps = 60 degrees

## Serial protocol (ESP32 firmware)

The ESP32 accepts line-based commands at 115200 baud:

```
g1 <motor> <signed_steps> <speed_hz> <accel_hz2>    Move motor
stop <motor|all>                                     Smooth stop
estop [motor|all]                                    Emergency stop
enable <motor|all> on|off                            Enable/disable driver
status [motor|all]                                   Show position and state
zero <motor|all>                                     Reset step counter
help                                                 Print help
```

Motor IDs: `pump` (`p`), `flush` (`f`), `centrifuge` (`c`), `x`, `y`, `z`.

Firmware feedback:
```
ok <motor> accepted          Move started
done <motor> steps=<N>       Move completed
stopped <motor> steps=<N>    Smooth stop completed
error busy: <motor>          Motor already moving
```

## Testing

```bash
# Stepper motor tests (no ROS needed)
python -m pytest src/astrobiology_actuators/test/ -v

# Spectrometer processing tests
colcon test --packages-select spectro_cam && colcon test-result --verbose
```

## Direct serial debugging

If you need to bypass ROS and talk to the ESP32 directly:

```bash
screen /dev/esp_steppers 115200
```

Type commands like `g1 p 3200 800 400`. Exit with `Ctrl+A`, `K`, `Y`.
