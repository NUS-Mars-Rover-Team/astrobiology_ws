# Astrobiology Payload ROS 2 Workspace

ROS 2 (Humble) workspace for the NUS Mars Rover astrobiology payload. Controls stepper motors (pumps, gantry, centrifuge), science lights (UV, laser, halogen), and a USB spectroscope camera via a Raspberry Pi.

## Architecture

![Architecture](docs/architecture.dot)

Render: `dot -Tpng docs/architecture.dot -o docs/architecture.png` or paste into [GraphvizOnline](https://dreampuf.github.io/GraphvizOnline).

## Hardware

| Device | Interface | udev alias |
|--------|-----------|------------|
| ESP32 6-channel stepper controller (CH340) | Serial 115200 (custom G1, see below) | `/dev/esp_steppers` (see [udev rules](#udev-rules)) |
| Arduino science lights + drill controller (`firmware/LEDdrill.ino`) | Serial 9600 (`<light> on\|off`) | `/dev/esp_lights` |
| Arduino drill controller (`firmware/LAdrill.ino`) | Serial 9600 (single chars `b`/`s`) | `/dev/uno_drill` |
| Sonix spectroscope camera | V4L2 (1920x1080 MJPG) | `/dev/spectroscope` |
| Microscope camera (Pi CSI, MMAL `bcm2835-v4l2`) | V4L2 | `/dev/uv_microscope_cam` |

### ESP32 Motor Channels

Pins as in the current stepper firmware (TMC2208 drivers, hardware-timer step generation):

| Channel | Firmware name | Short | STEP | DIR | EN | Endstop | Units |
|---------|--------------|-------|------|-----|----|---------|-------|
| Z gantry | z | z | 2 | 4 | 15 | GPIO21 | mm (3200 steps/mm) |
| Y gantry | y | y | 16 | 17 | 15 | GPIO34 | mm (3200 steps/mm) |
| X gantry | x | x | 18 | 19 | 15 | GPIO35 | mm (3200 steps/mm) |
| Centrifuge | centrifuge | c | 32 | 33 | 5 | — | rev, RPM (1600 steps/rev) |
| Pump | pump | p | 26 | 27 | 25 | — | raw pulses |
| Flush | flush | f | 12 | 13 | 14 | — | raw pulses |

Endstops trigger HIGH and the axes home in the negative direction, so gantry coordinates are
≥ 0 from the endstops. GPIO34/35 have no internal pull resistors, so X and Y need external
pull-downs.

### Science Lights Channels (Arduino, `firmware/LEDdrill.ino`)

Each light is switched by a MOSFET.

| Light | Firmware name | Pin |
|-------|--------------|-----|
| UVM LED | uvm | 12 |
| UVC LED | uvc | 11 |
| Laser | laser | 10 |
| Halogen | halogen | 9 |

## Packages

| Package | Description |
|---------|-------------|
| `astrobiology_actuators` | Stepper motor node, science lights node, and the `microscope_scan` gantry/microscope tool |
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

Create `/etc/udev/rules.d/99-astrobiology.rules` (on the current Pi the serial rules are in
`99-mrt-serial.rules`):

```
# ESP32 stepper controller. It is a CH340 (1a86:7523), the same chip as the Raman spectrometer's
# RS232 adapter, so match it by USB port, not by vendor/product ID. Check the path with:
#   udevadm info -q property -n /dev/ttyUSBn | grep '^ID_PATH='
SUBSYSTEM=="tty", ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.4.3:1.0", SYMLINK+="esp_steppers", MODE="0660", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"

# Arduino science lights controller — update idVendor/idProduct or use ATTRS{serial} to distinguish from esp_steppers
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="esp_lights", MODE="0666"

# Arduino drill controller — same caveat: match by ATTRS{serial} or ENV{ID_PATH} if the IDs clash
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="uno_drill", MODE="0666"

# Pi CSI microscope camera (MMAL / bcm2835-v4l2), capture node only. /dev/videoN renumbers
# when USB cameras are plugged in, so microscope_scan uses this link.
SUBSYSTEM=="video4linux", ATTR{name}=="camera0", ATTR{index}=="0", SYMLINK+="uv_microscope_cam", MODE="0660", GROUP="video"

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
ros2 launch astrobiology_actuators steppers.launch.py
```

This starts `steppers_node` with 12 services (move + stop for each of 6 motors).

### Launch science lights

```bash
ros2 launch astrobiology_actuators lights.launch.py
```

This starts `lights_node` with 4 services (SetBool for UVM, UVC, laser, halogen).

### Launch drill

```bash
ros2 launch astrobiology_actuators drill.launch.py
```

This starts `drill_node` with 2 Trigger services (`drill/start`, `drill/stop`).

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
ros2 run spectro_cam spectrum_plot --autoscale  # autoscale y, for absorbance (zero reference set)
```

The y-axis is fixed at 0–0.35 by default (saturation is 0.333), so the level and any clipping
can be read directly; change it with `--ylim MIN MAX`.

## Usage

### Science light control

Each light has a `SetBool` service (`data: true` = ON, `data: false` = OFF):

```bash
# Turn laser on
ros2 service call /astrobiology/laser/set std_srvs/srv/SetBool "{data: true}"

# Turn UVM off
ros2 service call /astrobiology/uvm/set std_srvs/srv/SetBool "{data: false}"
```

All lights are turned off automatically when the node shuts down.

### Drill control

```bash
ros2 service call /astrobiology/drill/start std_srvs/srv/Trigger   # sends b (drill runs backward)
ros2 service call /astrobiology/drill/stop std_srvs/srv/Trigger    # sends s (stop, coast)
```

The node's log shows the Arduino's reply (`arduino: Motor: BACKWARD`). The drill is stopped
automatically when the node shuts down.

### Microscope grid scan

`microscope_scan` homes the gantry, steps the cache tray through a 2 × 2 grid at five Z levels,
logs each position to a CSV and saves a picture at each point (`~/Pictures/test1.jpg` …
`test20.jpg`). It also has `pos` / `home` / `goto` commands for jogging the gantry. It talks
to the stepper ESP32 directly, so stop `steppers_node` first.

```bash
ros2 run astrobiology_actuators microscope_scan --port /dev/ttyUSB1 scan --dry-run
ros2 run astrobiology_actuators microscope_scan --port /dev/ttyUSB1 scan
```

Full guide (commands, config, coordinate model, viewing the pictures, troubleshooting):
[`docs/microscope_scan.md`](docs/microscope_scan.md).

### Stepper motor control

All motors use the `StepperMove` service (`steps`, `speed_hz`, `accel_hz2`):

> **Known issue:** `steppers_node` forwards `steps`/`speed_hz`/`accel_hz2` to the firmware unchanged,
> but the current firmware reads the gantry axes in mm, mm/s and mm/s², and the centrifuge in
> rev, RPM and RPM/s. The x/y/z gantry and centrifuge services therefore move by the wrong
> amount, or are rejected as out of range. Until `steppers_node` is updated, move the gantry with
> `microscope_scan goto`. Pump and flush are still raw pulses and unaffected.

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
| `/astrobiology/uvm/set` | SetBool | UVM LED on/off |
| `/astrobiology/uvc/set` | SetBool | UVC LED on/off |
| `/astrobiology/laser/set` | SetBool | Laser on/off |
| `/astrobiology/halogen/set` | SetBool | Halogen lamp on/off |
| `/astrobiology/drill/start` | Trigger | Start the drill (`b`) |
| `/astrobiology/drill/stop` | Trigger | Stop the drill (`s`) |
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

The stepper ESP32 accepts line-based commands at 115200 baud (newline or CR terminated,
case-insensitive, decimals allowed but no exponents). It is a custom G1 syntax, not standard
CNC G-code:

```
g1 <motor> <distance> <speed> <accel>   Relative move, in the motor's units (table above)
                                        e.g. g1 x 10.5 5 2    X +10.5 mm at 5 mm/s, 2 mm/s^2
                                             g1 c 2 120 60    centrifuge 2 rev at 120 RPM
                                             g1 p 3200 800 400  pump 3200 pulses
pos | m114                              Print all coordinates: pos Z:.. Y:.. X:.. C:.. P:.. F:..
g28 [x] [y] [z]                         Home onto endstops (always Z, Y, X order); sets pos to 0
stop <motor|all>                        Decelerate, then halt
estop [motor|all]                       Emergency stop (drivers disabled; re-home after)
!                                       Single byte, no Enter: instant estop of everything
enable <motor|all> on|off               Enable/disable driver
status [motor|all]                      State, endstop, steps and position
zero <motor|all>                        Set the position to 0 (motor must be idle)
diag                                    Timer ISR rate
help                                    Print help
```

Motor IDs: `pump` (`p`), `flush` (`f`), `centrifuge` (`c`), `x`, `y`, `z`. Wait for `done`
before sending another `g1` to the same motor. Different motors can move at the same time.
Speed is limited to 1–100000 steps/s (31.25 mm/s on the gantry), and acceleration to
1–1000000 steps/s².

### Science lights Arduino protocol

`firmware/LEDdrill.ino` accepts line-based commands at 9600 baud (case-insensitive):

```
<light> on | off   Light on or off; lights: uvm, uvc, laser, halogen
drill f | r | s    Drill forward / reverse / stop (unused: the drill is on LAdrill.ino)
```

It replies `<NAME> ON` / `<NAME> OFF` (e.g. `UVM ON`, `Laser OFF`), or `Invalid command: <text>`.
Opening the port resets the Arduino, which turns every light off.

The sketch waits only 50 ms between characters, so send each command in one write (as
`lights_node` does). Typing letter by letter in picocom or screen gets `Invalid command: u`.

### Drill Arduino protocol

`firmware/LAdrill.ino` reads single characters at 9600 baud (newlines ignored, case-insensitive):

```
f   Forward at PWM 200    -> Motor: FORWARD
b   Backward at PWM 200   -> Motor: BACKWARD        (drill/start)
s   Stop, driver disabled -> Motor: STOP (Coasting) (drill/stop)
```

Anything else replies `Invalid command. Use f, b, or s.` Single characters mean typing in
picocom works.

### Stepper firmware feedback

```
ok <motor> accepted                    Move started
done <motor> steps=<N> pos=<units>     Move completed
stopped <motor> steps=<N> pos=<units>  Smooth stop completed
error busy: <motor>                    Motor already moving or homing
error bad number for <motor>: ...      Distance/speed/accel out of range (limits in units)
ok g28 accepted: z y x                 Homing queued
homing <axis> / homed <axis> steps=0   Per-axis homing progress
done g28                               All requested axes homed
error homing failed <axis>: <reason>   Endstop not found / not released
aborted <motor>; physical position may be lost   After estop
error line too long or invalid; discarded        Line contained control bytes (e.g. Backspace)
```

## Testing

```bash
# Stepper motor, lights and microscope_scan tests (no ROS or hardware needed)
cd src/astrobiology_actuators && python3 -m pytest test/ -v

# Spectrometer processing tests
colcon test --packages-select spectro_cam && colcon test-result --verbose
```

## Direct serial debugging

If you need to bypass ROS and talk to the ESP32 directly (stop `steppers_node` first):

```bash
picocom -b 115200 --echo --imap lfcrlf /dev/esp_steppers   # quit: Ctrl+A then Ctrl+X
```

Type commands like `pos`, `status x` or `g1 x 2 2 5`. Retype a mistyped line in full: the
firmware discards any line containing Backspace or arrow-key bytes. Typing `!` stops every
motor instantly.
