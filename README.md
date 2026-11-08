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
| B&W Tek BTC100-2S Raman spectrometer (RS232 via CH340) | Serial, 9600 at power-on; `raman_node` switches it to 115200 (`K0`) | `/dev/raman_spectroscope` |
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
| `astrobiology_actuators` | Stepper motor node, science lights node, and the `gantry_scan` gantry/microscope tool |
| `spectro_cam` | USB spectrometer node (capture, process, publish spectra) |
| `raman_spectrometer` | BTC100 Raman spectrometer node (`raman_node`: scan while started, publish Raman spectra) |
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
# when USB cameras are plugged in, so gantry_scan uses this link.
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

### Launch Raman spectrometer

```bash
ros2 launch raman_spectrometer raman.launch.py          # port:=/dev/ttyUSBn to override
```

This starts `raman_node` in namespace `raman`, stopped. See [Raman spectra](#raman-spectra).

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

### Gantry grid scan

`gantry_scan` homes the gantry, steps the cache tray through a 2 × 2 grid at five Z levels,
logs each position to a CSV and saves a picture at each point (`~/Pictures/test1.jpg` …
`test20.jpg`). It also has `pos` / `home` / `goto` commands for jogging the gantry. It talks
to the stepper ESP32 directly, so stop `steppers_node` first.

`steppers_node` also runs the scan as a service, sharing its port:

```bash
ros2 service call /astrobiology/gantry/home astrobiology_interfaces/srv/Command
ros2 service call /astrobiology/gantry/scan astrobiology_interfaces/srv/Command
ros2 service call /astrobiology/gantry/handover astrobiology_interfaces/srv/Command
ros2 service call /astrobiology/gantry/lid astrobiology_interfaces/srv/Command     # dock lidding
ros2 service call /astrobiology/gantry/unlid astrobiology_interfaces/srv/Command   # dock unlidding
ros2 service call /astrobiology/gantry/stop std_srvs/srv/Trigger
```

```bash
ros2 run astrobiology_actuators gantry_scan --port /dev/ttyUSB1 scan --dry-run
ros2 run astrobiology_actuators gantry_scan --port /dev/ttyUSB1 scan
```

Full guide (commands, config, coordinate model, viewing the pictures, troubleshooting):
[`docs/gantry_scan.md`](docs/gantry_scan.md).

### Stepper motor control

Pump, flush and centrifuge use the `StepperMove` service (`distance`, `speed`, `accel`, all
`float32`), sent to the firmware unchanged as `g1 <motor> <distance> <speed> <accel>`, so the
units are the firmware's for that motor: **pump/flush in pulses, pulses/s, pulses/s²;
centrifuge in revolutions, RPM, RPM/s**. The gantry axes use `MoveAxis` in mm (below).

```bash
# Move pump 3200 pulses forward at 800 pulses/s, 400 pulses/s² accel
ros2 service call /astrobiology/pump/move \
  astrobiology_interfaces/srv/StepperMove "{distance: 3200, speed: 800, accel: 400}"

# Turn the centrifuge motor a quarter turn at 10 RPM
ros2 service call /astrobiology/centrifuge/move \
  astrobiology_interfaces/srv/StepperMove "{distance: 0.25, speed: 10, accel: 20}"

# Gantry: jog X by +0.5 mm, or go to X = 28 mm (MoveAxis, mm)
ros2 service call /astrobiology/gantry/x/move \
  astrobiology_interfaces/srv/MoveAxis "{position_mm: 0.5, relative: true}"
ros2 service call /astrobiology/gantry/x/move \
  astrobiology_interfaces/srv/MoveAxis "{position_mm: 28.0, relative: false}"

# Stop a motor
ros2 service call /astrobiology/pump/stop std_srvs/srv/Trigger
```

Gantry moves (`gantry/{x,y,z}/move`) reply at once (`message` like `X 0.6 -> 28 mm`,
`expected_s`) and run at `jog_speed_mm_s` (5 mm/s, `config/steppers.yaml`). They are refused
until that axis has been homed since `steppers_node` started (`/astrobiology/gantry/home`), if
the target is outside 0..`travel_max` (`gantry_scan.yaml`), while that axis is still moving, or
while gantry homing, the scan or the handover runs. Different axes can move at the same time.

### Available services

| Service | Type | Description |
|---------|------|-------------|
| `/astrobiology/pump/move` | StepperMove | Peristaltic pump |
| `/astrobiology/pump/stop` | Trigger | Stop pump |
| `/astrobiology/flush/move` | StepperMove | Flush pump |
| `/astrobiology/flush/stop` | Trigger | Stop flush |
| `/astrobiology/pump/dispense` | Dispense | Extraction pump: pump `volume_ml`, then stop (`pump/stop` ends it early) |
| `/astrobiology/flush/dispense` | Dispense | Dosing pump: pump `volume_ml`, then stop (`flush/stop` ends it early) |
| `/astrobiology/centrifuge/move` | StepperMove | Centrifuge motor (revolutions, RPM, RPM/s) |
| `/astrobiology/centrifuge/stop` | Trigger | Stop centrifuge (ends a spin or carousel step) |
| `/astrobiology/centrifuge/spin` | Spin | Spin at `rpm` for `duration_s`, then stop |
| `/astrobiology/carousel/rotate` | Rotate | Step the carousel `step_deg` (1, 5, 60) CW or CCW |
| `/astrobiology/carousel/home` | Trigger | Make the current carousel angle 0 (the cuvette at the camera); starts `/astrobiology/carousel/angle` |
| `/astrobiology/gantry/x/move` | MoveAxis | Gantry X axis, mm, relative or absolute |
| `/astrobiology/gantry/x/stop` | Trigger | Stop X axis |
| `/astrobiology/gantry/y/move` | MoveAxis | Gantry Y axis, mm, relative or absolute |
| `/astrobiology/gantry/y/stop` | Trigger | Stop Y axis |
| `/astrobiology/gantry/z/move` | MoveAxis | Gantry Z axis, mm, relative or absolute |
| `/astrobiology/gantry/z/stop` | Trigger | Stop Z axis |
| `/astrobiology/uvm/set` | SetBool | UVM LED on/off |
| `/astrobiology/uvc/set` | SetBool | UVC LED on/off |
| `/astrobiology/laser/set` | SetBool | Laser on/off |
| `/astrobiology/halogen/set` | SetBool | Halogen lamp on/off |
| `/astrobiology/drill/start` | Trigger | Start the drill (`b`) |
| `/astrobiology/drill/stop` | Trigger | Stop the drill (`s`) |
| `/astrobiology/gantry/home` | Command | Home the gantry, Z then Y then X (`steppers_node`); replies at once |
| `/astrobiology/gantry/scan` | Command | Start the gantry grid scan (`steppers_node`); replies at once |
| `/astrobiology/gantry/handover` | Command | Cache handover: home all, X to 53, Y to 105, home Y, home X (`steppers_node`); replies at once |
| `/astrobiology/gantry/lid` | Command | Dock lidding: home all, X to 160, Y to 6, home X, home Y; replies at once |
| `/astrobiology/gantry/unlid` | Command | Dock unlidding: home all, X to 160, Y to 0, home X; replies at once |
| `/astrobiology/gantry/stop` | Trigger | Stop gantry homing, the scan, the handover or lid docking (decelerates X/Y/Z) |
| `/spectro/spectrum_node/set_zero_reference` | Trigger | Snapshot the current spectrum as the blank (I0) |
| `/spectro/spectrum_node/clear_zero_reference` | Trigger | Clear the blank; back to raw spectra |
| `/raman/start` | Trigger | Start scanning the Raman spectrometer back to back |
| `/raman/stop` | Trigger | Stop scanning (the scan in progress is abandoned) |
| `/raman/set_dark` | Trigger | Use the latest raw scan as the dark (take it with the laser off) |
| `/raman/clear_dark` | Trigger | Stop subtracting the dark |

### Dispensing pumps (volume)

`steppers_node` runs the two peristaltic pumps by volume: `pump` (firmware `p`) is the
**extraction pump**, `flush` (firmware `f`) is the **dosing pump**.

```bash
ros2 service call /astrobiology/flush/dispense astrobiology_interfaces/srv/Dispense "{volume_ml: 5.0}"   # dosing
ros2 service call /astrobiology/flush/stop std_srvs/srv/Trigger
ros2 service call /astrobiology/pump/dispense astrobiology_interfaces/srv/Dispense "{volume_ml: 5.0}"    # extraction
```

- The reply comes at once (`dispensing 5.00 ml (5000 steps)`, `expected_s`); the pump stops by
  itself after the volume. A second dispense on the same pump is refused (`busy`) until the
  ESP32 reports `done`, as is a dispense while a raw `pump/move` / `flush/move` runs.
- Volumes must be more than 0 and at most `max_ml` (50 mL).
- Settings per pump in `config/steppers.yaml` (`pump.*`, `flush.*`):
  `pulses_per_ml`, `speed_hz` (800), `accel_hz2` (400), `direction` (`cw`/`ccw`), `max_ml`.
- **`pulses_per_ml` is a placeholder (1000) until calibrated.** In picocom run e.g.
  `g1 f 3200 800 400` into a measuring cylinder; `pulses_per_ml` = 3200 / mL collected.
  Repeat for `p`.
- If no `done` arrives, the pump is freed after its expected time plus 2 s, with a warning.

### Centrifuge and carousel

The centrifuge and the carousel are the same motor (firmware `c`, 1600 steps per motor
revolution).

```bash
# Centrifuge at 1300 RPM for 5 min (-> g1 c 6500 1300 100), then it stops by itself
ros2 service call /astrobiology/centrifuge/spin astrobiology_interfaces/srv/Spin "{rpm: 1300, duration_s: 300}"
ros2 service call /astrobiology/centrifuge/stop std_srvs/srv/Trigger     # stop early (decelerates)

# Carousel 60 degrees clockwise (direction 0 = CW, 1 = CCW)
ros2 service call /astrobiology/carousel/rotate astrobiology_interfaces/srv/Rotate "{step_deg: 60, direction: 0}"
```

- Spin: revolutions = `rpm` × `duration_s` / 60, ramped at `centrifuge.accel_rpm_s` (100);
  `expected_s` adds the ramps. Refused above `centrifuge.max_rpm` (3000; the firmware's own
  limit is 3750) or while the motor is busy.
- Rotate: `step_deg` must be one of `carousel.allowed_steps_deg` (1, 5, 60). Each step is
  rounded to whole motor steps with the remainder carried into the next, so six 60° steps are
  exactly one turn (267 + 266 + 267 + 267 + 266 + 267 = 1600 steps). Runs at
  `carousel.speed_rpm` (10). Set `carousel.gear_ratio` (motor revolutions per carousel
  revolution) if the carousel is geared, and `carousel.cw_positive: false` if CW turns the
  wrong way.
- The carousel angle isn't measured by a sensor, only counted from the motor steps.
  To give it a reference, turn cuvette 1 to the camera (carousel camera, ±° steps) and call
  `/astrobiology/carousel/home`. It sends the firmware's `zero c` (refused while the motor
  moves). From then on `steppers_node` publishes **`/astrobiology/carousel/angle`**
  (`std_msgs/Float32`, degrees clockwise from the camera, 0–360) from the `C:` field of the
  ESP32's `pos` replies (2 Hz) and each `done c` line.
- The angle stops being published, with a warning, after an estop (`!`, `estop c`: the driver
  is switched off and the wheel can coast) or an ESP32 restart (the count goes back to 0).
  Call `carousel/home` again. Skipped steps (overload) aren't detected, so check the camera
  after a hard spin.

```bash
ros2 service call /astrobiology/carousel/home std_srvs/srv/Trigger
ros2 topic echo /astrobiology/carousel/angle
```

### Gantry position topic

`steppers_node` publishes `/astrobiology/gantry/position` (`geometry_msgs/msg/Point`, **mm**, not
metres) at `position_rate_hz` (default 2 Hz; 0 turns polling off). It polls the ESP32 with `pos`
while idle; during homing or a scan it publishes the positions those report instead. Nothing is
published until the ESP32 has reported all three axes. The values count from power-on until the
gantry is homed.

```bash
ros2 topic echo /astrobiology/gantry/position
```

### Raman spectra

`raman_node` (package `raman_spectrometer`) drives the B&W Tek BTC100. While started it scans
back to back and publishes `/raman/spectrum` (`astrobiology_msgs/msg/RamanSpectrum`, reliable
QoS) after each scan: `counts` per detector pixel with `raman_shift` (cm⁻¹) and `wavelengths`
(nm) axes. Plot `raman_shift` against `counts`; the 550 nm longpass filter limits the useful
range to about 600–2850 cm⁻¹. Each scan takes the integration time × averages plus ~1 s of
transfer.

```bash
ros2 service call /astrobiology/laser/set std_srvs/srv/SetBool "{data: false}"   # laser off
ros2 service call /raman/start std_srvs/srv/Trigger
ros2 service call /raman/set_dark std_srvs/srv/Trigger        # after a scan with the laser off
ros2 service call /astrobiology/laser/set std_srvs/srv/SetBool "{data: true}"    # laser on (Class 3B: goggles)
ros2 param set /raman/raman_node integration_ms 1000          # live; clears the dark
ros2 service call /raman/stop std_srvs/srv/Trigger
```

- The node never switches the 532 nm laser; that is `lights_node`'s `/astrobiology/laser/set`.
- `raw_max` ≥ 65000 or `saturated_pixels` > 0 means clipped: lower `integration_ms` or the laser power.
- The dark is only subtracted while `integration_ms`/`averages` match the ones it was taken with;
  changing either clears it.
- Parameters are in `src/raman_spectrometer/config/raman.yaml`: `port`, `integration_ms`,
  `averages`, `laser_nm` (532), the wavelength calibration `coeffs` (CFL lines, 2026-10-22,
  ~355–627 nm) and `autostart`.
- The standalone bench script (CSV capture, absorbance, live plot) is still
  `~/Documents/NUSMRT/btc100_test/btc100_capture.py`. Stop `raman_node` before using it:
  only one program can open the port.

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
float32 distance  # signed, in the motor's firmware units (pump/flush: pulses; centrifuge: rev)
float32 speed     # > 0 (pulses/s or RPM)
float32 accel     # > 0 (pulses/s² or RPM/s)
---
bool    success
string  message
```

### Step calculations (firmware MOTOR TABLE, 1.8°/step motors)

- Gantry X/Y/Z: 200 steps × 1/16 microstep / 1 mm pitch = 3200 steps per mm
- Centrifuge/carousel `c`: 200 steps × 1/8 microstep = 1600 steps per revolution,
  4.44 steps per degree, so 1° ≈ 4.4, 5° ≈ 22.2 and 60° ≈ 266.7 steps (`carousel/rotate` rounds
  each step and carries the remainder)
- Pump/flush: raw pulses (1:1)

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
# Stepper motor, lights and gantry_scan tests (no ROS or hardware needed)
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
