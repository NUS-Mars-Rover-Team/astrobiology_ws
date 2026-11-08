# ROS 2 Message Types Reference

Complete reference of all ROS 2 message and service types used in the astrobiology workspace.

---

## Custom Messages

### `astrobiology_msgs/msg/Spectrum`

Published by `spectrum_node` on `/spectro/spectrum`, subscribed by `spectrum_plot`.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame ID |
| `wavelengths` | `float32[]` | Wavelength in nm per ROI column (linear two-point calibration) |
| `r` | `float32[]` | Red channel intensity, normalised by `roi_height * 255 * 3` |
| `g` | `float32[]` | Green channel intensity (same normalisation) |
| `b` | `float32[]` | Blue channel intensity (same normalisation) |
| `combined` | `float32[]` | `(r+g+b)/3` after gain, low-pass filter, and zero reference. With zero ref: subtract mode = `I - I0`, absorbance mode = `-log10(I/I0)` |
| `peak_wavelengths` | `float32[]` | Detected peak positions (nm) |
| `peak_values` | `float32[]` | Detected peak amplitudes |
| `dip_wavelengths` | `float32[]` | Detected dip positions (nm) |
| `dip_values` | `float32[]` | Detected dip amplitudes |

```bash
ros2 topic echo /spectro/spectrum astrobiology_msgs/msg/Spectrum
```

### `astrobiology_msgs/msg/RamanSpectrum`

Published by `raman_node` on `/raman/spectrum` after every scan while started (`/raman/start`).

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame ID (`raman`) |
| `raman_shift` | `float32[]` | Raman shift in cm⁻¹ per pixel: `1e7/laser_nm - 1e7/wavelength`; negative below the laser line |
| `wavelengths` | `float32[]` | Wavelength in nm per pixel (cubic calibration) |
| `counts` | `float32[]` | Detector counts, minus the dark when `dark_subtracted` |
| `dark_subtracted` | `bool` | Whether a dark (`/raman/set_dark`) was subtracted |
| `laser_nm` | `float32` | Excitation wavelength used for `raman_shift` (532) |
| `integration_ms` | `uint32` | Integration time of this scan |
| `averages` | `uint32` | Scans averaged by the unit |
| `raw_max` | `uint32` | Largest count before dark subtraction; ≥ 65000 is saturated |
| `saturated_pixels` | `uint32` | Pixels at or above 65000 |

```bash
ros2 topic echo /raman/spectrum --field raw_max
```

### `astrobiology_msgs/msg/MotorStatus`

Published by `motor_bridge` on `/astrobiology/motor_status`.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame ID |
| `motor_ids` | `uint8[]` | IDs of known motors |
| `moving` | `bool[]` | Whether each motor is currently moving |
| `port_connected` | `bool` | Whether the serial port is connected |

```bash
ros2 topic echo /astrobiology/motor_status astrobiology_msgs/msg/MotorStatus
```

---

## Custom Services (`astrobiology_interfaces`)

### `StepperMove.srv`

Move a stepper motor by a relative amount. Sent unchanged as `g1 <motor> <distance> <speed> <accel>`, so the units are the firmware's for that motor: pump/flush in pulses, pulses/s, pulses/s²; centrifuge in revolutions, RPM, RPM/s.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `distance` | `float32` | Signed (negative = reverse); fractions allowed, e.g. 0.25 rev |
| **Req** | `speed` | `float32` | > 0 |
| **Req** | `accel` | `float32` | > 0 |
| **Res** | `success` | `bool` | Sent to the ESP32 |
| **Res** | `message` | `string` | The `g1` command sent, or why it was rejected |

Servers: `/astrobiology/pump/move`, `/astrobiology/flush/move`, `/astrobiology/centrifuge/move` (the gantry axes use `MoveAxis`)

```bash
# Pump forward 3200 steps
ros2 service call /astrobiology/pump/move \
  astrobiology_interfaces/srv/StepperMove \
  "{distance: 3200, speed: 800, accel: 400}"

# Centrifuge motor a quarter turn at 10 RPM
ros2 service call /astrobiology/centrifuge/move \
  astrobiology_interfaces/srv/StepperMove \
  "{distance: 0.25, speed: 10, accel: 20}"

```

### `Dispense.srv`

Run a peristaltic pump for a set volume. No flow sensor: run time = volume / calibrated flow rate.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `volume_ml` | `float32` | Volume to dispense |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Servers: `/astrobiology/pump/dispense` (extraction, firmware `p`), `/astrobiology/flush/dispense` (dosing, firmware `f`) (`steppers_node`; volume → pulses via the `pulses_per_ml` parameter; stop early with `pump/stop` / `flush/stop`)

```bash
# Dispense 5 mL with the dosing pump (flush)
ros2 service call /astrobiology/flush/dispense \
  astrobiology_interfaces/srv/Dispense \
  "{volume_ml: 5.0}"
```

### `MoveAxis.srv`

Move one gantry axis to an absolute position or by a relative offset.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `position_mm` | `float32` | Absolute target, or signed offset if relative |
| **Req** | `relative` | `bool` | If true, `position_mm` is an offset |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Servers: `/astrobiology/gantry/x/move`, `/astrobiology/gantry/y/move`, `/astrobiology/gantry/z/move` (`steppers_node`; the axis must be homed, target within 0..`travel_max`)

```bash
# Move Z gantry to 2.5 mm absolute (within travel_max.z)
ros2 service call /astrobiology/gantry/z/move \
  astrobiology_interfaces/srv/MoveAxis \
  "{position_mm: 2.5, relative: false}"

# Nudge Y gantry +3 mm relative
ros2 service call /astrobiology/gantry/y/move \
  astrobiology_interfaces/srv/MoveAxis \
  "{position_mm: 3.0, relative: true}"
```

### `Command.srv`

A command with no arguments that takes time to finish.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | *(empty)* | | No request fields |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Servers: `/astrobiology/gantry/home`, `/astrobiology/gantry/scan`, `/astrobiology/gantry/handover`, `/astrobiology/gantry/lid`, `/astrobiology/gantry/unlid` (`steppers_node`)

```bash
# Home the gantry (Z, then Y, then X)
ros2 service call /astrobiology/gantry/home \
  astrobiology_interfaces/srv/Command
```

### `Spin.srv`

Run the carousel/centrifuge hybrid as a centrifuge at `rpm` for `duration_s`, then stop on its own. Stop early with `/astrobiology/centrifuge/stop`.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `duration_s` | `float32` | Time at full speed (ConOps: 300 s); ramps add `rpm / accel` |
| **Req** | `rpm` | `float32` | > 0, at most `centrifuge.max_rpm` (3000) |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Server: `/astrobiology/centrifuge/spin` (`steppers_node`; sends `g1 c <rpm × duration_s / 60> <rpm> <accel>`)

```bash
# Centrifuge at 1300 RPM for 300 seconds
ros2 service call /astrobiology/centrifuge/spin \
  astrobiology_interfaces/srv/Spin \
  "{rpm: 1300.0, duration_s: 300.0}"
```

### `Rotate.srv`

Step the carousel to bring a cuvette into view. Steps are rounded to whole motor steps with the remainder carried over, so repeated steps don't drift.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `DIRECTION_CW` | `uint8` = 0 | Constant: clockwise |
| **Req** | `DIRECTION_CCW` | `uint8` = 1 | Constant: counter-clockwise |
| **Req** | `step_deg` | `float32` | Step angle, one of `carousel.allowed_steps_deg` (1, 5, 60) |
| **Req** | `direction` | `uint8` | CW or CCW |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Server: `/astrobiology/carousel/rotate` (`steppers_node`, firmware motor `c`)

```bash
# Rotate carousel 60 degrees clockwise
ros2 service call /astrobiology/carousel/rotate \
  astrobiology_interfaces/srv/Rotate \
  "{step_deg: 60.0, direction: 0}"

# Rotate carousel 5 degrees counter-clockwise
ros2 service call /astrobiology/carousel/rotate \
  astrobiology_interfaces/srv/Rotate \
  "{step_deg: 5.0, direction: 1}"
```

### `SetCarouselMode.srv`

Switch the hybrid between carousel and centrifuge mode. Rejected while busy.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `MODE_CAROUSEL` | `uint8` = 0 | Constant: carousel mode |
| **Req** | `MODE_CENTRIFUGE` | `uint8` = 1 | Constant: centrifuge mode |
| **Req** | `mode` | `uint8` | Which mode to switch to |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Server: `/astrobiology/carousel/set_mode`

```bash
# Switch to centrifuge mode
ros2 service call /astrobiology/carousel/set_mode \
  astrobiology_interfaces/srv/SetCarouselMode \
  "{mode: 1}"

# Switch to carousel mode
ros2 service call /astrobiology/carousel/set_mode \
  astrobiology_interfaces/srv/SetCarouselMode \
  "{mode: 0}"
```

---

## Custom Services (`astrobiology_msgs`)

### `MoveMotor.srv`

Legacy motor bridge service.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `motor` | `uint8` | Motor ID: 1-3, or 0 for all |
| **Req** | `forward` | `bool` | Direction |
| **Req** | `steps` | `uint32` | Step count |
| **Req** | `speed_us` | `uint32` | Step period in microseconds (0 = default) |
| **Res** | `accepted` | `bool` | Whether the command was accepted |
| **Res** | `message` | `string` | Error reason if failed |

Server: `/astrobiology/move_motor`

```bash
# Move motor 2 forward 500 steps at default speed
ros2 service call /astrobiology/move_motor \
  astrobiology_msgs/srv/MoveMotor \
  "{motor: 2, forward: true, steps: 500, speed_us: 0}"
```

---

## Standard ROS 2 Types

### `std_srvs/srv/Trigger`

No request fields. Used for stop commands and spectro zero-reference.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | *(empty)* | | No request fields |
| **Res** | `success` | `bool` | Whether it succeeded |
| **Res** | `message` | `string` | Informational |

Servers: `/astrobiology/pump/stop`, `/astrobiology/flush/stop`, `/astrobiology/centrifuge/stop`, `/astrobiology/carousel/home`, `/astrobiology/gantry/x/stop`, `/astrobiology/gantry/y/stop`, `/astrobiology/gantry/z/stop`, `/astrobiology/gantry/stop`, `/spectro/.../set_zero_reference`, `/spectro/.../clear_zero_reference`

```bash
# Stop the pump
ros2 service call /astrobiology/pump/stop std_srvs/srv/Trigger

# Set spectro zero reference
ros2 service call /spectro/spectrum_node/set_zero_reference std_srvs/srv/Trigger

# Clear spectro zero reference
ros2 service call /spectro/spectrum_node/clear_zero_reference std_srvs/srv/Trigger
```

### `std_srvs/srv/SetBool`

Toggle a light on or off.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `data` | `bool` | True = on, False = off |
| **Res** | `success` | `bool` | Whether it succeeded |
| **Res** | `message` | `string` | Informational |

Servers: `/astrobiology/uvm/set`, `/astrobiology/uvc/set`, `/astrobiology/laser/set`, `/astrobiology/halogen/set`

```bash
# Turn laser on
ros2 service call /astrobiology/laser/set std_srvs/srv/SetBool "{data: true}"

# Turn UVM off
ros2 service call /astrobiology/uvm/set std_srvs/srv/SetBool "{data: false}"
```

### `sensor_msgs/msg/CompressedImage`

Published by `spectrum_node` on `/spectro/preview/compressed`.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp + frame ID |
| `format` | `string` | Encoding (e.g. `"jpeg"`) |
| `data` | `uint8[]` | Compressed image bytes |

```bash
ros2 topic echo /spectro/preview/compressed sensor_msgs/msg/CompressedImage
```

### `std_msgs/msg/Header`

Embedded in `Spectrum`, `MotorStatus`, and `CompressedImage`.

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | `builtin_interfaces/Time` | Timestamp (`sec` + `nanosec`) |
| `frame_id` | `string` | Reference frame |
