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

Move a stepper motor. Maps directly to `g1 <motor> <steps> <speed_hz> <accel_hz2>`. Used by all stepper actuators (pumps, centrifuge, flush, gantry axes).

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `steps` | `int32` | Signed step count (+forward, -reverse) |
| **Req** | `speed_hz` | `int32` | Pulses/sec (1..50000) |
| **Req** | `accel_hz2` | `int32` | Pulses/sec² (1..1000000) |
| **Res** | `success` | `bool` | Whether accepted |
| **Res** | `message` | `string` | Error reason if failed |

Servers: `/astrobiology/pump/move`, `/astrobiology/flush/move`, `/astrobiology/centrifuge/move`, `/astrobiology/x_gantry/move`, `/astrobiology/y_gantry/move`, `/astrobiology/z_gantry/move`

```bash
# Pump forward 3200 steps
ros2 service call /astrobiology/pump/move \
  astrobiology_interfaces/srv/StepperMove \
  "{steps: 3200, speed_hz: 800, accel_hz2: 400}"

# Gantry X reverse 1000 steps
ros2 service call /astrobiology/x_gantry/move \
  astrobiology_interfaces/srv/StepperMove \
  "{steps: -1000, speed_hz: 1000, accel_hz2: 500}"
```

### `Dispense.srv`

Run a peristaltic pump for a set volume. No flow sensor: run time = volume / calibrated flow rate.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `volume_ml` | `float32` | Volume to dispense |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Servers: `/astrobiology/pump1/dispense`, `/astrobiology/pump2/dispense`, `/astrobiology/pump3/dispense`

```bash
# Dispense 5 mL from pump3
ros2 service call /astrobiology/pump3/dispense \
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

Servers: `/astrobiology/x_gantry/move`, `/astrobiology/y_gantry/move`, `/astrobiology/z_gantry/move`

```bash
# Move Z gantry to 25 mm absolute
ros2 service call /astrobiology/z_gantry/move \
  astrobiology_interfaces/srv/MoveAxis \
  "{position_mm: 25.0, relative: false}"

# Nudge Y gantry +3 mm relative
ros2 service call /astrobiology/y_gantry/move \
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

Servers: `/astrobiology/x_gantry/home`, `/astrobiology/y_gantry/home`, `/astrobiology/z_gantry/home`

```bash
# Home X gantry
ros2 service call /astrobiology/x_gantry/home \
  astrobiology_interfaces/srv/Command
```

### `Spin.srv`

Run the carousel/centrifuge hybrid as a centrifuge. Rejected unless in centrifuge mode.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `duration_s` | `float32` | How long to spin (ConOps: 300 s) |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Server: `/astrobiology/centrifuge/spin`

```bash
# Centrifuge for 300 seconds
ros2 service call /astrobiology/centrifuge/spin \
  astrobiology_interfaces/srv/Spin \
  "{duration_s: 300.0}"
```

### `Rotate.srv`

Step the carousel to bring a cuvette into view. Rejected unless in carousel mode.

| | Field | Type | Description |
|---|-------|------|-------------|
| **Req** | `DIRECTION_CW` | `uint8` = 0 | Constant: clockwise |
| **Req** | `DIRECTION_CCW` | `uint8` = 1 | Constant: counter-clockwise |
| **Req** | `step_deg` | `float32` | Step angle (must be 5 or 60 from config) |
| **Req** | `direction` | `uint8` | CW or CCW |
| **Res** | `success` | `bool` | Accepted (not finished) |
| **Res** | `message` | `string` | Reason if rejected |
| **Res** | `expected_s` | `float32` | Estimated time to finish; 0 if rejected |

Server: `/astrobiology/carousel/rotate`

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

Servers: `/astrobiology/pump/stop`, `/astrobiology/flush/stop`, `/astrobiology/centrifuge/stop`, `/astrobiology/x_gantry/stop`, `/astrobiology/y_gantry/stop`, `/astrobiology/z_gantry/stop`, `/spectro/.../set_zero_reference`, `/spectro/.../clear_zero_reference`

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

Servers: `/astrobiology/uv1/set`, `/astrobiology/uv2/set`, `/astrobiology/laser/set`, `/astrobiology/halogen/set`

```bash
# Turn laser on
ros2 service call /astrobiology/laser/set std_srvs/srv/SetBool "{data: true}"

# Turn UV1 off
ros2 service call /astrobiology/uv1/set std_srvs/srv/SetBool "{data: false}"
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
