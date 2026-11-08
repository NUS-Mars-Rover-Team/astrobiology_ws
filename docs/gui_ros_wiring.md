# Science tab: GUI ↔ ROS wiring

For each control on the four Science sub-tabs of Mars-Rover-GUI: the ROS service to call or
topic to subscribe to, the request, and what the rover does. Types come from
`astrobiology_interfaces` and `astrobiology_msgs` unless noted. Field-by-field definitions are in
[`ros_message_types.md`](ros_message_types.md).

## Before wiring anything

**GUI machine**
- Build and source `astrobiology_msgs` and `astrobiology_interfaces` from `astrobiology_ws`.
- `ROS_DOMAIN_ID=67` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on both the GUI machine and the Pi.

**Running on the Pi**
- `steppers_node`: gantry, pumps, centrifuge, carousel
- `drill_node`: drill (Arduino, `/dev/uno_drill`)
- `lights_node`: UV LEDs, laser, halogen
- `ros2 launch raman_spectrometer raman.launch.py`
- `ros2 launch spectro_cam spectro_cam.launch.py`

**Calling services**
- Always `call_async`; never block the Qt thread. Callbacks run on the ROS thread, so hand
  results to Qt with a signal.
- Every service replies at once; long jobs carry on on the rover.
- On `success: false`, show `message` (e.g. `busy: …`, `X is not homed …`). Nothing was sent.

> **The red E-STOP strip does not stop the science payload.** The GUI publishes `/e_stop`
> (`std_msgs/Bool`), but no astrobiology node subscribes to it yet. Until then, stop motion with
> `/astrobiology/gantry/stop`, `/astrobiology/centrifuge/stop`, `/astrobiology/pump/stop`,
> `/astrobiology/flush/stop` and `/astrobiology/drill/stop`.

## Tab 1: Drill

| GUI control | ROS | Request / data | What happens |
|---|---|---|---|
| START DRILL (toggle on) | call `/astrobiology/drill/start` (`std_srvs/Trigger`) | empty | Arduino gets `b`; drill runs at fixed speed. No running/stopped feedback |
| START DRILL (toggle off) | call `/astrobiology/drill/stop` (`std_srvs/Trigger`) | empty | Arduino gets `s`; drill coasts to a stop |
| CACHE HANDOVER / TRANSFER | call `/astrobiology/gantry/handover` (`Command`) | empty | Home all, X → 53 mm, Y → 105 mm, home Y, home X. `expected_s` ≈ 150 s (upper bound) |
| EXTEND / RETRACT / EDIT length | — | | Not wired: no linear actuator firmware or service; length stays manual |
| Sensor readouts (temp, humidity, pH, EC, NPK) | — | | Not published by `astrobiology_ws` |
| Ground Cam, Ground Microscope | — | | No camera topics from `astrobiology_ws` |

## Tab 2: Imaging

| GUI control | ROS | Request / data | What happens |
|---|---|---|---|
| Positioning X / Y / Z | subscribe `/astrobiology/gantry/position` (`geometry_msgs/Point`) | `x`, `y`, `z` in **mm**, 2 Hz | Pass `{"X": x, "Y": y, "Z": z}` to `ImagingTab.set_coordinates()`. Meaningful after homing. Show `--` if nothing arrives for a few seconds |
| Home | call `/astrobiology/gantry/home` (`Command`) | empty | Homes Z, then Y, then X |
| Incremental ± → Confirm | call `/astrobiology/gantry/{x,y,z}/move` (`MoveAxis`) | `{position_mm: <delta>, relative: true}`, one call per axis | Moves at 5 mm/s. Refused until homed, outside 0..travel_max, while that axis moves, or during a gantry job |
| START SCAN | call `/astrobiology/gantry/scan` (`Command`) | empty | Home, 20 points (2 × 2 grid × 5 Z levels), a photo at each, CSV on the Pi. ≈ 40 s plus homing |
| Scan point counts, Total, Estimated size | — | | Unused: the scan size comes from `gantry_scan.yaml` on the Pi. Hide or disable |
| Raman Spectrometer plot | subscribe `/raman/spectrum` (`RamanSpectrum`) | x: `raman_shift` (cm⁻¹), y: `counts` | One spectrum per scan (~1 s) while started. Useful range ≈ 600–2850 cm⁻¹; `saturated_pixels` > 0 means clipped |
| Raman start / stop (no button yet) | call `/raman/start`, `/raman/stop` (`std_srvs/Trigger`) | empty | The node starts stopped; nothing is published until start |
| Raman dark (no button yet) | call `/raman/set_dark`, `/raman/clear_dark` (`std_srvs/Trigger`) | empty | Subtracts a scan taken with the laser off |
| Laser for Raman (no button yet) | call `/astrobiology/laser/set` (`std_srvs/SetBool`) | `{data: true}` / `{data: false}` | 532 nm, Class 3B: safety goggles |
| Auto Focus | — | | Not implemented |
| VIEW IMAGES | — | | Scan photos are files on the Pi (`~/Pictures/test1.jpg` … `test20.jpg`); no ROS topic |
| UV Microscope, Enclosure Camera | — | | No camera topics from `astrobiology_ws` |

`set_coordinates()` currently cancels pending jog offsets on every call; with position arriving at
2 Hz that wipes the jog clicks before Confirm. Only cancel when the position actually changes.

## Tab 3: Seal and Slurry

| GUI control | ROS | Request / data | What happens |
|---|---|---|---|
| Position X / Y / Z | subscribe `/astrobiology/gantry/position` | as on Imaging | |
| DOCK / UNDOCK LID (on) | call `/astrobiology/gantry/lid` (`Command`) | empty | Home all, X → 160 mm, Y → 6 mm, home X, home Y |
| DOCK / UNDOCK LID (off) | call `/astrobiology/gantry/unlid` (`Command`) | empty | Home all, X → 160 mm, Y → 0 mm, home X |
| DOSING PUMP ON (+ Set DP) | call `/astrobiology/flush/dispense` (`Dispense`) | `{volume_ml: <Set DP>}` | Pumps that volume (firmware `f`), then stops. 0 < volume ≤ 50 mL. Switch the toggle off after `expected_s` |
| DOSING PUMP OFF | call `/astrobiology/flush/stop` (`std_srvs/Trigger`) | empty | Stops the dosing pump |
| EXTRACTION PUMP ON (+ Set EP) | call `/astrobiology/pump/dispense` (`Dispense`) | `{volume_ml: <Set EP>}` | Pumps that volume (firmware `p`), then stops |
| EXTRACTION PUMP OFF | call `/astrobiology/pump/stop` (`std_srvs/Trigger`) | empty | Stops the extraction pump |
| Carousel ±1° / ±5° / ±60° | call `/astrobiology/carousel/rotate` (`Rotate`) | `{step_deg: <abs>, direction: 0}` for + (CW), `1` for − (CCW) | Steps the carousel at 10 RPM; repeated steps don't drift |
| DOCK / UNDOCK PIPES | — | | Not wired: sequence not defined yet |
| SHAKE & MIX CACHE | — | | Not wired: sequence not defined yet |
| Enclosure, Carousel cameras | — | | No camera topics from `astrobiology_ws` |

Pump volumes use a placeholder calibration (1000 pulses per mL) until it is measured.

## Tab 4: Carousel

| GUI control | ROS | Request / data | What happens |
|---|---|---|---|
| Centrifuge START (Time, RPM) | call `/astrobiology/centrifuge/spin` (`Spin`) | `{rpm: 1300, duration_s: 300}` (mm:ss → seconds) | Spins, then stops on its own. `expected_s` ≈ duration + ramps (313 s for 1300 RPM × 5 min). Max 3000 RPM |
| Centrifuge STOP | call `/astrobiology/centrifuge/stop` (`std_srvs/Trigger`) | empty | Decelerates to a stop |
| EMERGENCY STOP | — | | Not wired: no separate service; use Centrifuge STOP |
| UV LED ON / OFF | call `/astrobiology/uvc/set` (`std_srvs/SetBool`) | `{data: true}` / `{data: false}` | Carousel UV LED (`uvm` is the microscope's) |
| BULB ON / OFF | call `/astrobiology/halogen/set` (`std_srvs/SetBool`) | `{data: true}` / `{data: false}` | Halogen lamp for the spectroscope |
| Carousel ±1° / ±5° / ±60° | call `/astrobiology/carousel/rotate` (`Rotate`) | as on Seal and Slurry | Same wheel as the centrifuge; refused while it spins |
| Carousel Position wheel | subscribe `/astrobiology/carousel/angle` (`std_msgs/Float32`) | degrees clockwise from the camera, 0–360 | Rotate the drawing by this angle. Cuvette at CAM = `round(angle / 60) % 6 + 1`. Published only after Carousel home; stops after an estop or ESP32 restart |
| Carousel home (no button yet) | call `/astrobiology/carousel/home` (`std_srvs/Trigger`) | empty | With cuvette 1 lined up at the camera, sets that angle to 0 |
| Spectroscope panel | subscribe `/spectro/spectrum` (`Spectrum`), QoS `qos_profile_sensor_data` | x: `wavelengths` (nm), y: `combined` | Camera spectroscope, up to 15 Hz; usable ≈ 470–1000 nm. Default (reliable) QoS receives nothing |
| VIEW RESULTS, Carousel Cam | — | | Not provided by `astrobiology_ws` |

## Every tab

- **Shared hardware refuses overlapping work.** Gantry home, scan, handover, lid and unlid block
  each other and the jogs. The centrifuge spin and carousel steps share one motor. A refused call
  returns `success: false` with `busy: …` and sends nothing.
- **`/astrobiology/gantry/stop`** (`std_srvs/Trigger`) stops any running gantry job; no button yet.
- **Toggles show requests, not state.** Only the gantry position, carousel angle and the two
  spectra come back as live data.
- **Long jobs don't report completion.** Use `expected_s` from the reply to reset a button or
  show progress.

## All interfaces

**steppers_node services**

| Service | Type |
|---|---|
| `/astrobiology/gantry/home`, `scan`, `handover`, `lid`, `unlid` | `Command` |
| `/astrobiology/gantry/stop` | `std_srvs/Trigger` |
| `/astrobiology/gantry/{x,y,z}/move` | `MoveAxis` |
| `/astrobiology/gantry/{x,y,z}/stop` | `std_srvs/Trigger` |
| `/astrobiology/pump/dispense`, `/astrobiology/flush/dispense` | `Dispense` |
| `/astrobiology/{pump,flush,centrifuge}/move` | `StepperMove` |
| `/astrobiology/{pump,flush,centrifuge}/stop` | `std_srvs/Trigger` |
| `/astrobiology/centrifuge/spin` | `Spin` |
| `/astrobiology/carousel/rotate` | `Rotate` |
| `/astrobiology/carousel/home` | `std_srvs/Trigger` |

**Other services**

| Service | Type | Node |
|---|---|---|
| `/astrobiology/drill/start`, `/astrobiology/drill/stop` | `std_srvs/Trigger` | `drill_node` |
| `/astrobiology/{uvm,uvc,laser,halogen}/set` | `std_srvs/SetBool` | `lights_node` |
| `/raman/start`, `/raman/stop`, `/raman/set_dark`, `/raman/clear_dark` | `std_srvs/Trigger` | `raman_node` |
| `/spectro/spectrum_node/set_zero_reference`, `clear_zero_reference` | `std_srvs/Trigger` | `spectrum_node` |

**Topics**

| Topic | Type | Notes |
|---|---|---|
| `/astrobiology/gantry/position` | `geometry_msgs/Point` | mm, 2 Hz |
| `/astrobiology/carousel/angle` | `std_msgs/Float32` | degrees, 2 Hz, after carousel home |
| `/raman/spectrum` | `astrobiology_msgs/RamanSpectrum` | reliable, ~1 Hz while started |
| `/spectro/spectrum` | `astrobiology_msgs/Spectrum` | best-effort, up to 15 Hz |
| `/spectro/preview/compressed` | `sensor_msgs/CompressedImage` | only when `preview_rate_hz` > 0 |
