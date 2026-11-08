# Gantry grid scan — 2026-11-02

`gantry_scan` homes the X/Y/Z gantry and moves the cache tray through a 2 × 2 grid at
several Z levels. At each point it stops, logs the position and takes a picture with the Pi
camera (`test1.jpg` … `test20.jpg`). It also has commands for jogging the gantry by hand.

The script talks to the stepper ESP32 directly over serial. It doesn't use ROS, so **`steppers_node`
must be stopped while you use it**, because both open the same port.

Source: [`src/astrobiology_actuators/astrobiology_actuators/gantry_scan.py`](../src/astrobiology_actuators/astrobiology_actuators/gantry_scan.py)
Config: [`src/astrobiology_actuators/config/gantry_scan.yaml`](../src/astrobiology_actuators/config/gantry_scan.yaml)

## Status

| Item | Status |
|---|---|
| `pos`, `home`, `goto` jog commands | Done, tested on the Pi |
| 20-point scan with CSV log | Done, tested on the Pi (trial Z levels 0–4 mm) |
| Picture at every point | Done, tested on the Pi |
| Camera udev link `/dev/uv_microscope_cam` | Done on the Pi; see [Camera](#camera) |
| CSI camera delivering frames | **Open:** after renumbering, `ffmpeg` and `vcgencmd` hang; see [Camera](#camera) |
| Unit tests (no hardware) | Done, 105 passing in `astrobiology_actuators/test/` |
| Z travel limit | **Open:** capped at 5 mm in the config until measured |
| Real Z levels 0–40 mm (0–4 cm) | **Open:** needs the Z travel limit first |
| `z_offset_mm` (Z of the first level) | **Open:** 0 for now; tune with the microscope view |
| Microscope / grid offsets | Pattern centred at gantry X/Y 28, 15 (measured under the microscope) |
| `/dev/esp_steppers` udev rule | **Open:** points at the Raman adapter; see [Serial port](#serial-port) |

## Quick start (on the Pi)

```bash
cd ~/astrobiology_ws
git pull && colcon build --packages-select astrobiology_actuators && source install/setup.bash
alias ms='ros2 run astrobiology_actuators gantry_scan --port /dev/ttyUSB1'

ms scan --dry-run      # check the points, moves and picture names; nothing moves
ms scan                # home, scan all 20 points, return home
```

Press **Ctrl-C** at any time to stop. If a motor is moving, the script sends the firmware's
instant emergency stop (`!`), which also disables the drivers, so run `ms home` before moving
again.

Then copy the pictures to a laptop and view them:

```bash
mkdir -p ~/scan && cd ~/scan
scp 'marsroverscience@192.168.1.108:~/Pictures/test*.jpg' .
eog $(ls -v test*.jpg)                 # flip through with the arrow keys, in order
```

To see all 20 in one grid image (rows = X/Y positions, columns = Z levels 0 → 4):

```bash
ffmpeg -loglevel error -y -i test%d.jpg -vf "scale=480:-1,tile=5x4" -frames:v 1 grid.jpg && eog grid.jpg
```

`ls -v` sorts the files test1, test2, … test20. A plain `test*.jpg` puts test10 right after
test1.

## Commands

All commands accept `--port` (default `/dev/esp_steppers`) and `-v`, which prints every line sent
to and received from the ESP32.

| Command | What it does |
|---|---|
| `ms pos` | Print the gantry X/Y/Z in mm |
| `ms home` | Home all axes onto their endstops (the firmware always goes Z, then Y, then X); X/Y/Z = 0 afterwards |
| `ms home z` | Home only the listed axes |
| `ms goto --x 20 --y 20` | Move to absolute positions in mm; axes you leave out stay put |
| `ms goto --z 3 --speed 2` | Slower move (default 5 mm/s, max 31.25; accel default 10 mm/s², `--accel`) |
| `ms scan --dry-run` | Print the points, the `g1` moves and the picture names without opening the port |
| `ms scan` | Home, visit every point, log a CSV, take pictures, return to 0/0/0 |

`goto` never moves Z together with X/Y. If Z is heading toward 0, Z moves first (the tray lifts
clear, then travels). If Z is heading away from 0, X/Y move first and Z moves last. Targets
below 0 are rejected, because 0 is the endstop.

Options for `scan`:

| Option | Effect |
|---|---|
| `--config FILE` | Use another config. Pass the source file to try changes without rebuilding: `--config ~/astrobiology_ws/src/astrobiology_actuators/config/gantry_scan.yaml` |
| `--z-levels 0 1 2` | Override the Z levels (mm) for this run |
| `--skip-home` | Don't home first; trust the current position |
| `--no-return` | Stay at the last point instead of going back to 0/0/0 |
| `--no-images` | Don't take pictures |
| `--camera /dev/videoN` | Use another camera device |
| `--image-dir DIR` | Save pictures somewhere other than `~/Pictures` |
| `--out FILE.csv` | CSV path (default `scans/gantry_scan_<time>.csv` under the current folder) |

## As ROS services (steppers_node)

`steppers_node` runs homing and the scan as services, through its own ESP32 connection, so the
pump, flush and centrifuge services keep working meanwhile. Don't run `ms` while `steppers_node`
is up: only one program can open the port.

```bash
ros2 run astrobiology_actuators steppers_node --ros-args -p port:=/dev/ttyUSB0
ros2 service call /astrobiology/gantry/home astrobiology_interfaces/srv/Command
ros2 service call /astrobiology/gantry/scan astrobiology_interfaces/srv/Command
ros2 service call /astrobiology/gantry/stop std_srvs/srv/Trigger
```

| Service | Type | What it does |
|---|---|---|
| `/astrobiology/gantry/home` | `Command` | Homes Z, then Y, then X (`g28`); X/Y/Z = 0 afterwards. Replies at once; `expected_s` is an upper bound from `travel_max`. The log says `gantry homed` when it finishes |
| `/astrobiology/gantry/scan` | `Command` | Starts the full scan (home, 20 points, pictures, back to 0/0/0) in the background. Replies at once: `success`, the CSV path in `message`, and `expected_s` (rough time, not counting homing) |
| `/astrobiology/gantry/stop` | `Trigger` | Decelerates X/Y/Z and ends the scan or homing. A stopped scan keeps its CSV and needs no re-home (no estop); stopped homing leaves the position unset, so home again |

- Progress (`#3/20 (0,0) lvl 3 ...`) and the final `gantry scan done` line are in the node's log.
- Homing and the scan are refused while either is in progress, while a gantry axis is busy, or
  if the node has no serial connection (`dry_run`). The scan is also refused if `calibrated` is
  false, or pictures are on and ffmpeg or the camera is missing.
- While homing or a scan runs, `x_gantry/move`, `y_gantry/move` and `z_gantry/move` reply
  `busy: gantry homing in progress` / `busy: gantry scan in progress`.
- If a move fails (a timeout or a firmware error), the node sends `!` like the CLI, so re-home before the next run.

Node parameters:

| Parameter | Default | |
|---|---|---|
| `scan_config` | `''` = the installed `gantry_scan.yaml` | Point it at the source file to try config changes without rebuilding |
| `scan_dir` | `~/scans` | Where the CSVs go |

The CLI options (`--z-levels`, `--no-images`, ...) have no service equivalent; set them in the config.

## What a scan does

The scan is a 2 × 2 grid at 1 mm spacing (4 X/Y positions) times 5 Z levels, so 20 points.
X/Y is the outer loop and Z the inner loop:

```
home (g28)
position (0,0): Z→0, move X/Y, then Z level 0, 1, 2, 3, 4   → test1  … test5
position (1,0): Z→0, X +1 mm,  then Z level 0 … 4           → test6  … test10
position (1,1): Z→0, Y +1 mm,  then Z level 0 … 4           → test11 … test15
position (0,1): Z→0, X −1 mm,  then Z level 0 … 4           → test16 … test20
Z→0, then X/Y→0
```

Z goes back to 0, the clear position, before every X/Y move. At each point the scan:
1. waits `settle_s` (0.5 s);
2. reads `pos` from the ESP32;
3. takes one picture while the gantry is stationary;
4. writes a CSV row.

**CSV columns:** `index, iz, ix, iy, local_x/y/z, target_x/y/z, pos_x/y/z, error_mm, image,
timestamp`.
- `error_mm` is how far the reported position is from the target. The scan prints
  `WARNING off target` if it exceeds `pos_tolerance_mm`.
- `pos` comes from the firmware's step counter, so it confirms that the commands were carried
  out, not where the tray physically is. The steppers are open loop.

**Pictures:** each picture is saved with
`ffmpeg -f v4l2 -i /dev/uv_microscope_cam -frames:v 1 testN.jpg`, after first dropping `skip_frames`
frames so auto-exposure has settled. They go to `~/Pictures/test1.jpg` … `test20.jpg` and are
**overwritten every run**, so copy them off before the next scan.
- The old file is deleted before each capture, so a failed capture can't leave last run's picture
  under this run's name.
- If a capture fails, the scan prints `WARNING image failed: <ffmpeg error>`, leaves that CSV
  `image` cell empty, and carries on.
- Before homing, the scan checks that ffmpeg is installed and that the camera device exists, so a
  missing camera is caught before anything moves.

## Configuration

`config/gantry_scan.yaml` (plain YAML, not ROS parameters):

| Key | Current value | Meaning |
|---|---|---|
| `calibrated` | `true` | The live scan refuses to run while this is `false` (`--dry-run` always works) |
| `grid_size` | 40 × 40 | Cache size in mm |
| `grid_offset` | 0, 0 | Grid origin corner on the tray |
| `microscope_offset` | 48, 35 | Lens position. With the values above this centres the pattern at gantry X/Y 28, 15 (centre + 20, half the grid) |
| `travel_min` / `travel_max` | X 11.275–86.275, Y 2.55–77.55, Z 0–5 | Safe gantry range. Any point outside it is rejected before motion |
| `scan_origin` | `null` | First grid point in grid coordinates; `null` centres the pattern in the grid |
| `pitch_mm`, `n_x`, `n_y` | 1.0, 2, 2 | Grid spacing and size |
| `z_offset_mm` | 0 | Gantry Z of the first level |
| `z_levels_mm` | 0, 1, 2, 3, 4 | Z levels visited at every X/Y position (trial values) |
| `speed_mm_s`, `accel_mm_s2` | 5, 10 | Motion for every move (firmware max 31.25 mm/s, 312.5 mm/s²) |
| `settle_s` | 0.5 | Wait after each move before reading `pos` and taking the picture |
| `pos_tolerance_mm` | 0.001 | Warn if `pos` is further than this from the target (1 step = 0.0003 mm) |
| `camera.enabled`, `camera.device` | `true`, `/dev/uv_microscope_cam` | Pi CSI camera (udev link; see [Camera](#camera)) |
| `camera.image_dir`, `camera.image_prefix` | `~/Pictures`, `test` | Picture names: `<image_dir>/<image_prefix>N.jpg` |
| `camera.skip_frames` | 5 | Frames dropped before the saved one (0 = exactly the plain ffmpeg command) |

After editing the installed config, rebuild (`colcon build --packages-select
astrobiology_actuators`), or pass the source file with `--config`.

### Coordinate model

All positions are in mm, in the gantry frame: home = the endstops = 0/0/0, and positive moves
away from them. Z = 0 is the clear position, with the tray furthest from the microscope; +Z moves
the tray toward it. The gantry moves the tray under a fixed microscope, so grid point *p* is under
the lens when the gantry is at

```
X, Y = microscope_offset − (grid_offset + p)
Z    = z_offset_mm + z level
```

Only `microscope_offset − grid_offset` matters. To measure it, `goto` the tray until the grid's
origin corner is focused under the lens, then read `ms pos`.

### Moving to the real 0–4 cm Z levels

1. Find how far Z can go safely. Run `ms goto --z 10 --speed 2`, then 20, 30, … and stop well
   before the tray reaches the lens.
2. Set `travel_max.z` to that value and `z_levels_mm: [0.0, 10.0, 20.0, 30.0, 40.0]`.
3. Run `ms scan --dry-run`. Any level beyond `travel_max.z` is refused with
   `target z=… is outside travel`.

## Serial port

The stepper ESP32 and the Raman spectrometer's RS232 adapter both use the same CH340 USB-serial
chip (`1a86:7523`). The Pi's rule in `/etc/udev/rules.d/99-mrt-serial.rules` matches any CH340, so
**`/dev/esp_steppers` currently points at the Raman port (`ttyUSB0`)**. The ESP32 is `ttyUSB1`,
hence the `--port /dev/ttyUSB1` in the alias above.

To fix it, match the ESP32 by its USB port, the same way the Raman rule does. This only works if
the ESP32 stays plugged into the same hub port:

```bash
udevadm info -q property -n /dev/ttyUSB1 | grep '^ID_PATH='   # confirm it ends in usb-0:1.1.4.3:1.0
```

```
SUBSYSTEM=="tty", ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.4.3:1.0", SYMLINK+="esp_steppers", MODE="0660", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/esp_steppers /dev/raman_spectroscope   # should point to different ttyUSBs
```

After that, `--port` can be dropped.

## Camera

The microscope camera is the Pi's CSI camera, driven by the MMAL `bcm2835-v4l2` driver. `v4l2-ctl
--list-devices` shows it as `mmal service 16.1 (platform:bcm2835-v4l2-0)`. Its `/dev/videoN`
number changes depending on which USB cameras are plugged in: it was `video2` with them, and
`video0` without. A udev rule gives it a fixed name. It matches on the attributes
`udevadm info -a -n /dev/videoN` reports (`ATTR{name}=="camera0"`, `ATTR{index}=="0"`).

`/etc/udev/rules.d/99-mrt-camera.rules`:

```
SUBSYSTEM=="video4linux", ATTR{name}=="camera0", ATTR{index}=="0", SYMLINK+="uv_microscope_cam", MODE="0660", GROUP="video"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/uv_microscope_cam
```

Check that it captures. The time limit stops a dead camera from hanging the terminal:

```bash
timeout 15 ffmpeg -loglevel error -y -f v4l2 -i /dev/uv_microscope_cam -frames:v 1 ~/test.jpg; echo "exit=$?"
```

`exit=0` means it works. `exit=124` means it timed out: the device opened but sent no frames.

**If `ffmpeg` hangs:** a capture that never gets a frame can leave the GPU firmware channel stuck.
After that, `vcgencmd` hangs too, and so does every later capture. Only a reboot clears it
(`kill -9` doesn't remove processes stuck in `D` state). After rebooting, run these before
touching the camera:

```bash
timeout 5 vcgencmd get_throttled   # want throttled=0x0 (no under-voltage)
timeout 5 vcgencmd get_camera      # want supported=1 detected=1
```

`detected=0` means the Pi doesn't see the camera: power off and reseat the ribbon cable at both
ends. A non-zero `throttled` value points at the power supply; the cameras and USB devices are
the first to misbehave when the 5 V rail sags. Until the camera works, run the scan with
`--no-images`.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `error: no reply within 2.0 s` on `ms pos` | Wrong port (probably the Raman adapter on `ttyUSB0`); use `--port /dev/ttyUSB1`. Also check nothing else has the port open: `fuser -v /dev/ttyUSB1` (a leftover `screen`/`picocom`, or `steppers_node`) |
| `error line too long or invalid; discarded` when typing in picocom | The firmware rejects any line containing control bytes. picocom sends Backspace and arrow keys as raw bytes, so retype the whole line without corrections |
| `< (boot) error line too long …` with `-v` | Harmless: a junk byte when the port opens. The script clears it before the first command |
| `error homing failed <axis>: endstop not found …` | Check the endstop in picocom with `status x` (released → `open`, pressed → `TRIGGERED`). X/Y are on GPIO35/34, which have no internal pull resistor; with the HIGH trigger level they need an external pull-down |
| `camera /dev/uv_microscope_cam not found` | The udev rule is missing or didn't match; see [Camera](#camera). Meanwhile find the CSI camera with `v4l2-ctl --list-devices` (`mmal service 16.1`) and pass `--camera /dev/videoN` |
| `ffmpeg` sits after its banner, or `vcgencmd` hangs | The camera isn't delivering frames and the GPU firmware channel is stuck; see [Camera](#camera) |
| `WARNING image failed: … Device or resource busy` | Another program has the camera open; close it |
| `target z=… is outside travel` | A Z level is beyond `travel_max.z`; see [Moving to the real 0–4 cm Z levels](#moving-to-the-real-04-cm-z-levels) |
| `calibrated is false` | Check the offsets with `--dry-run`, then set `calibrated: true` |

Manual serial session, if you need to bypass the script:

```bash
picocom -b 115200 --echo --imap lfcrlf /dev/ttyUSB1   # quit: Ctrl+A then Ctrl+X
```

Type `pos`, `help`, `status x`, or `g1 x 2 2 5` (X +2 mm at 2 mm/s). Never type `!`: it is an
instant emergency stop of every motor, even in the middle of a line.

## Tests

The tests use a fake serial port and a fake camera, so they need no hardware:

```bash
cd src/astrobiology_actuators && python3 -m pytest test/ -q
```

- `test_gantry_scan.py`: the serial link (`pos`, `home`, `move_to`, timeouts, emergency stop
  on exit) and the move order.
- `test_gantry_scan_plan.py`: the point order, the target formula, validation, the dry run,
  `run_scan` with the CSV, picture capture, and the command-line checks.
- `test_serial_commands.py`: command strings and reply parsing.
