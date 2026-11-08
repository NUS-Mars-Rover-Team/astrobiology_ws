"""Grid scan over the X/Y/Z gantry.  No ROS imports.

The CLI talks to the ESP32 stepper controller directly over serial, so
steppers_node must NOT be running (it holds the same port).  steppers_node
offers the same scan as a ROS service (/astrobiology/gantry/scan), driving
Gantry through its own port with QueuedSerial.

Subcommands:
    gantry_scan pos              print the gantry X/Y/Z position in mm
    gantry_scan home [x y z]     home onto the endstops (firmware order Z, Y, X)
    gantry_scan goto --x 1       absolute move in mm (axes not given stay put);
                                 Z moves alone: first toward 0, last away from 0
    gantry_scan scan             home, visit every point, log pos to a CSV
                                 and grab a camera frame (test1.jpg ...)
    gantry_scan scan --dry-run   print the scan points and moves (no port)

Scan geometry (config/gantry_scan.yaml): the gantry moves the cache tray
under a fixed microscope, so grid point p is under the lens when the gantry is at
    X, Y = microscope_offset - (grid_offset + p)
    Z    = z_offset + z level
Z is back at 0 (clear) whenever X/Y moves.
"""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import time

import yaml

from astrobiology_actuators.serial_commands import (
    DONE_RE,
    ESTOP_CHAR,
    home_command,
    move_mm_command,
    parse_pos,
    pos_command,
)

DEFAULT_PORT = '/dev/esp_steppers'
DEFAULT_BAUD = 115200
BOOT_WAIT_S = 2.0       # opening the port may reset the ESP32; wait out its banner
FLUSH_DRAIN_S = 0.2
POS_TIMEOUT_S = 2.0
# Firmware worst case per axis: 1,000,000-step search at 10 kHz = 100 s, plus backoffs
HOME_TIMEOUT_PER_AXIS_S = 120.0
MOVE_MARGIN_S = 2.0     # added to the estimated move time before giving up

# Firmware limits: 3200 steps/mm, 1..100000 steps/s, 1..1000000 steps/s^2
STEPS_PER_MM = 3200.0
HALF_STEP_MM = 0.5 / STEPS_PER_MM
MAX_SPEED_MM_S = 100000 / STEPS_PER_MM      # 31.25
MAX_ACCEL_MM_S2 = 1000000 / STEPS_PER_MM    # 312.5
DEFAULT_SPEED_MM_S = 5.0
DEFAULT_ACCEL_MM_S2 = 10.0

CONFIG_NAME = 'gantry_scan.yaml'
CAPTURE_TIMEOUT_S = 20.0
CAPTURE_ESTIMATE_S = 2.0   # typical time per picture, for estimate_scan_s

# Reply prefixes (lower-cased) that mean the firmware rejected or aborted something
_FAULT_PREFIXES = ('error', 'aborted', 'estop')


class GantryError(RuntimeError):
    """The firmware reported a fault, or an expected reply never arrived."""


class ScanConfigError(ValueError):
    """The scan config is invalid or would drive the gantry out of bounds."""


def safe_groups(current_z, target):
    """Split a target into the axis groups to move, in order.

    Z = 0 is the clear position (the firmware homes Z first to lift clear),
    so Z moves first when it is heading toward 0 and last when heading away:
    the tray lifts clear before travelling and travels before lowering.
    """
    xy = {axis: target[axis] for axis in 'xy' if axis in target}
    z = {'z': target['z']} if 'z' in target else {}
    groups = (z, xy) if z and z['z'] < current_z else (xy, z)
    return [group for group in groups if group]


class Gantry:
    """Line-based serial link to the stepper ESP32, gantry axes only.

    Pass ``ser`` (anything with write/readline/close)
    to skip opening a real port, e.g. in tests.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, ser=None,
                 log=print, verbose=False):
        self._log = log
        self._verbose = verbose
        self._moving = False
        self._ser = ser
        if ser is None:
            import serial
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self._drain(BOOT_WAIT_S)
        # End any half-received line on the ESP32.  Opening the port can inject
        # a junk byte, so the reply to this is often 'error line too long or
        # invalid; discarded' -- drain it rather than mistake it for a fault.
        self._ser.write(b'\n')
        self._drain(FLUSH_DRAIN_S)

    def _drain(self, duration):
        """Discard boot banner/leftover lines for duration s (shown with --verbose)."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            line = self._ser.readline().decode(errors='replace').strip()
            if line and self._verbose:
                self._log(f'< (boot) {line}')

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def send(self, cmd):
        if self._verbose:
            self._log(f'> {cmd}')
        self._ser.write((cmd + '\n').encode())

    def readline_until(self, predicate, timeout):
        """Read lines until predicate(line) returns something other than None.

        Lines that don't match are logged and skipped.  Fault lines raise
        GantryError, as does running out of time.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._ser.readline().decode(errors='replace').strip()
            if not line:
                continue
            if self._verbose:
                self._log(f'< {line}')
            if line.lower().startswith(_FAULT_PREFIXES):
                raise GantryError(line)
            result = predicate(line)
            if result is not None:
                return result
            if not self._verbose:
                self._log(f'esp: {line}')
        raise GantryError(f'no reply within {timeout:.1f} s')

    def pos(self, timeout=POS_TIMEOUT_S):
        """Return the firmware's {'x', 'y', 'z'} position in mm."""
        self.send(pos_command())
        p = self.readline_until(parse_pos, timeout)
        try:
            return {axis: p[axis] for axis in 'xyz'}
        except KeyError as e:
            raise GantryError(f'pos reply has no {e} axis') from None

    def home(self, axes=()):
        """Home the given axes (default all) and wait for 'done g28'.

        The firmware homes one axis at a time in Z, Y, X order and sets each
        homed axis to 0.  Raises GantryError on 'error homing failed ...'.
        """
        self.send(home_command(*axes))
        self._moving = True
        self.readline_until(lambda line: True if line == 'done g28' else None,
                            HOME_TIMEOUT_PER_AXIS_S * (len(axes) or 3))
        self._moving = False

    def move_to(self, target, speed=DEFAULT_SPEED_MM_S, accel=DEFAULT_ACCEL_MM_S2):
        """Move the axes in target ({'x': mm, ...}) to those absolute positions.

        All listed axes start together; returns once every one reports done.
        Axes already within half a step of their target aren't commanded.
        """
        if not 0 < speed <= MAX_SPEED_MM_S:
            raise GantryError(f'speed must be in (0, {MAX_SPEED_MM_S}] mm/s')
        if not 0 < accel <= MAX_ACCEL_MM_S2:
            raise GantryError(f'accel must be in (0, {MAX_ACCEL_MM_S2}] mm/s^2')
        for axis, mm in target.items():
            if mm < 0:
                raise GantryError(f'{axis}={mm} is behind the endstop (home = 0)')

        current = self.pos()
        deltas = {axis: mm - current[axis] for axis, mm in target.items()
                  if abs(mm - current[axis]) >= HALF_STEP_MM}
        if not deltas:
            return

        self._moving = True
        for axis, delta in deltas.items():
            self.send(move_mm_command(axis, delta, speed, accel))

        pending = set(deltas)

        def all_done(line):
            if line.startswith(('stopped', 'cancelled')):
                raise GantryError(line)
            m = DONE_RE.match(line)
            if m:
                pending.discard(m.group(1))
            return True if not pending else None

        # Cruise time plus one accel and one decel ramp
        timeout = max(map(abs, deltas.values())) / speed + speed / accel + MOVE_MARGIN_S
        self.readline_until(all_done, timeout)
        self._moving = False

    def move_safe(self, target, speed=DEFAULT_SPEED_MM_S, accel=DEFAULT_ACCEL_MM_S2):
        """move_to, but Z and XY never move at the same time (see safe_groups)."""
        current_z = self.pos()['z'] if 'z' in target else 0.0
        for group in safe_groups(current_z, target):
            self.move_to(group, speed, accel)

    def estop(self):
        """Instant stop of every motor.  Drivers are disabled: re-home after."""
        self._ser.write(ESTOP_CHAR)
        self._moving = False

    def close(self):
        if self._moving:
            self.estop()
            self._log('motion interrupted: sent ! (estop); re-home before the next run')
        self._ser.close()


class QueuedSerial:
    """Serial stand-in for Gantry when another owner (steppers_node) holds the port.

    Writes go to write(data); readline() returns the lines the owner put on
    ``lines`` (a queue.Queue of stripped str), or b'' after 0.1 s like a port
    timeout.  close() does nothing: the owner closes the real port.
    """

    def __init__(self, write, lines):
        self._write = write
        self._lines = lines

    def write(self, data):
        self._write(data)

    def readline(self):
        try:
            return (self._lines.get(timeout=0.1) + '\n').encode()
        except queue.Empty:
            return b''

    def close(self):
        pass


# -- Scan plan (pure, no serial) -----------------------------------------------

def _xyz(d, name, axes='xyz'):
    try:
        return {axis: float(d[axis]) for axis in axes}
    except (KeyError, TypeError, ValueError):
        raise ScanConfigError(f'{name} needs numeric {", ".join(axes)}') from None


@dataclass
class ScanConfig:
    """Scan geometry and motion settings.  Lengths in mm, gantry frame."""

    grid_offset: dict           # {x, y}: grid origin corner on the tray
    microscope_offset: dict     # {x, y}: lens axis
    grid_size: dict             # {x, y}
    travel_max: dict            # {x, y, z}: furthest safe gantry position
    travel_min: dict = None     # {x, y, z}: nearest safe gantry position; None = 0
    z_offset_mm: float = 0.0    # gantry Z of the first Z level
    scan_origin: dict = None    # {x, y} first pattern point in the grid; None = centred
    pitch_mm: float = 1.0
    n_x: int = 2
    n_y: int = 2
    z_levels_mm: tuple = (0.0, 10.0, 20.0, 30.0, 40.0)
    speed_mm_s: float = DEFAULT_SPEED_MM_S
    accel_mm_s2: float = DEFAULT_ACCEL_MM_S2
    settle_s: float = 0.5
    pos_tolerance_mm: float = 0.001
    calibrated: bool = False    # set true once the offsets are measured
    camera_enabled: bool = True
    camera_device: str = '/dev/uv_microscope_cam'
    image_dir: Path = Path('~/Pictures')    # <image_prefix><n>.jpg, overwritten every run
    image_prefix: str = 'test'
    skip_frames: int = 5            # frames dropped before the saved one (auto-exposure)

    def __post_init__(self):
        if self.travel_min is None:
            self.travel_min = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.image_dir = Path(self.image_dir).expanduser()

    @classmethod
    def from_dict(cls, d):
        try:
            return cls(
                grid_offset=_xyz(d['grid_offset'], 'grid_offset', 'xy'),
                microscope_offset=_xyz(d['microscope_offset'], 'microscope_offset', 'xy'),
                grid_size=_xyz(d['grid_size'], 'grid_size', 'xy'),
                travel_max=_xyz(d['travel_max'], 'travel_max'),
                travel_min=(None if d.get('travel_min') is None
                            else _xyz(d['travel_min'], 'travel_min')),
                z_offset_mm=float(d.get('z_offset_mm', 0.0)),
                scan_origin=(None if d.get('scan_origin') is None
                             else _xyz(d['scan_origin'], 'scan_origin', 'xy')),
                pitch_mm=float(d.get('pitch_mm', 1.0)),
                n_x=int(d.get('n_x', 2)),
                n_y=int(d.get('n_y', 2)),
                z_levels_mm=tuple(float(z) for z in d.get(
                    'z_levels_mm', cls.z_levels_mm)),
                speed_mm_s=float(d.get('speed_mm_s', DEFAULT_SPEED_MM_S)),
                accel_mm_s2=float(d.get('accel_mm_s2', DEFAULT_ACCEL_MM_S2)),
                settle_s=float(d.get('settle_s', 0.5)),
                pos_tolerance_mm=float(d.get('pos_tolerance_mm', 0.001)),
                calibrated=bool(d.get('calibrated', False)),
                **cls._camera_fields(d.get('camera') or {}),
            )
        except KeyError as e:
            raise ScanConfigError(f'config is missing {e}') from None
        except (TypeError, ValueError) as e:
            raise ScanConfigError(f'bad config value: {e}') from None

    @staticmethod
    def _camera_fields(cam):
        return {
            'camera_enabled': bool(cam.get('enabled', True)),
            'camera_device': str(cam.get('device', '/dev/uv_microscope_cam')),
            'image_dir': Path(str(cam.get('image_dir', '~/Pictures'))),
            'image_prefix': str(cam.get('image_prefix', 'test')),
            'skip_frames': int(cam.get('skip_frames', 5)),
        }

    @classmethod
    def from_yaml(cls, path):
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f) or {})

    def image_path(self, n):
        """Path of the n-th picture (1-based, in scan order)."""
        return self.image_dir / f'{self.image_prefix}{n}.jpg'

    def origin(self):
        """First pattern point in grid coordinates (centred if not set)."""
        if self.scan_origin is not None:
            return self.scan_origin
        return {axis: (self.grid_size[axis] - (n - 1) * self.pitch_mm) / 2
                for axis, n in (('x', self.n_x), ('y', self.n_y))}


@dataclass
class ScanPoint:
    index: int
    iz: int
    ix: int
    iy: int
    local: dict     # {x, y} in grid coordinates, z = the Z level
    target: dict    # {x, y, z} gantry position


def _serpentine(n_x, n_y):
    return [(ix, iy) for iy in range(n_y)
            for ix in (range(n_x) if iy % 2 == 0 else reversed(range(n_x)))]


def scan_points(cfg):
    """All scan points in visiting order.

    XY is the outer loop (a serpentine, so each XY hop is one pitch) and the
    Z levels are the inner loop: every Z level at one XY position, then the
    next position starting again from the first level.
    """
    origin = cfg.origin()
    points = []
    for ix, iy in _serpentine(cfg.n_x, cfg.n_y):
        for iz, z_level in enumerate(cfg.z_levels_mm):
            local = {'x': origin['x'] + ix * cfg.pitch_mm,
                     'y': origin['y'] + iy * cfg.pitch_mm,
                     'z': z_level}
            target = {axis: cfg.microscope_offset[axis]
                      - (cfg.grid_offset[axis] + local[axis]) for axis in 'xy'}
            target['z'] = cfg.z_offset_mm + z_level
            points.append(ScanPoint(len(points), iz, ix, iy, local, target))
    return points


def validate(cfg, points):
    """Raise ScanConfigError before any motion if the scan can't run safely."""
    if cfg.n_x < 1 or cfg.n_y < 1 or cfg.pitch_mm <= 0 or not cfg.z_levels_mm:
        raise ScanConfigError('need n_x, n_y >= 1, pitch_mm > 0 and at least one z level')
    if not 0 < cfg.speed_mm_s <= MAX_SPEED_MM_S:
        raise ScanConfigError(f'speed_mm_s must be in (0, {MAX_SPEED_MM_S}]')
    if not 0 < cfg.accel_mm_s2 <= MAX_ACCEL_MM_S2:
        raise ScanConfigError(f'accel_mm_s2 must be in (0, {MAX_ACCEL_MM_S2}]')
    if cfg.settle_s < 0:
        raise ScanConfigError('settle_s must be >= 0')
    for p in points:
        for axis in 'xy':
            if not 0 <= p.local[axis] <= cfg.grid_size[axis]:
                raise ScanConfigError(
                    f'point {p.index} {axis}={p.local[axis]:.3f} is outside the '
                    f'{cfg.grid_size[axis]:g} mm grid')
        for axis in 'xyz':
            lo, hi = max(cfg.travel_min[axis], 0.0), cfg.travel_max[axis]
            if not lo <= p.target[axis] <= hi:
                raise ScanConfigError(
                    f'point {p.index} target {axis}={p.target[axis]:.3f} is outside '
                    f'travel {lo:g}..{hi:g} mm')


HOME = {'x': 0.0, 'y': 0.0, 'z': 0.0}


def scan_moves(points, return_home=True):
    """Every stop of a scan: [(label, point, groups)].

    groups are the axis dicts to move in order (one move_to each).  Z is at 0
    (clear) whenever XY moves:
      - new XY position (incl. the first): Z to 0, XY, then Z to the level
      - same XY position: Z only, to the next level
      - last point -> home: Z to 0, then XY to 0
    point is None for the return home.
    """
    stops = []
    prev = None
    for p in points:
        xy = {'x': p.target['x'], 'y': p.target['y']}
        if prev is not None and (prev.target['x'], prev.target['y']) == (xy['x'], xy['y']):
            groups = [{'z': p.target['z']}]
        else:
            groups = [{'z': 0.0}, xy, {'z': p.target['z']}]
        stops.append((f'#{p.index}', p, groups))
        prev = p
    if return_home and prev is not None:
        stops.append(('home', None, [{'z': 0.0}, {'x': 0.0, 'y': 0.0}]))
    return stops


def dry_run_lines(cfg, points, return_home=True):
    """Printable scan plan with the g1 lines a scan from home would send."""
    lines = [' #  lvl ix iy    local x/y/z (mm)          target X/Y/Z (mm)'
             + ('        image' if cfg.camera_enabled else '')]
    for p in points:
        lines.append(
            f'{p.index:2d}  {p.iz:3d} {p.ix:2d} {p.iy:2d}  '
            f'{p.local["x"]:7.3f} {p.local["y"]:7.3f} {p.local["z"]:7.3f}   '
            f'{p.target["x"]:7.3f} {p.target["y"]:7.3f} {p.target["z"]:7.3f}'
            + (f'   {cfg.image_path(p.index + 1).name}' if cfg.camera_enabled else ''))

    lines += ['', 'moves (after g28, from 0/0/0):']
    current = dict(HOME)
    for label, _, groups in scan_moves(points, return_home):
        steps = []
        for group in groups:
            cmds = [move_mm_command(axis, mm - current[axis], cfg.speed_mm_s,
                                    cfg.accel_mm_s2)
                    for axis, mm in group.items()
                    if abs(mm - current[axis]) >= HALF_STEP_MM]
            if cmds:
                steps.append(' + '.join(cmds))
            current.update(group)
        lines.append(f'  {label:>4}: ' + ('  then  '.join(steps) or '(no move)'))
    return lines


def ffmpeg_command(device, path, skip_frames=0):
    """ffmpeg command that saves one frame from a V4L2 camera to path."""
    cmd = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'v4l2', '-i', device]
    if skip_frames > 0:
        cmd += ['-vf', f'select=gte(n\\,{skip_frames})']
    return cmd + ['-frames:v', '1', str(path)]


def capture_image(device, path, skip_frames=0):
    """Save one camera frame to path.  Returns None, or why it failed.

    Never raises: a failed picture is only a warning during a scan.  Any old
    file at path is removed first so a failure can't leave a previous run's
    picture behind under this run's name.
    """
    path = Path(path)
    try:
        path.unlink(missing_ok=True)
        result = subprocess.run(ffmpeg_command(device, path, skip_frames),
                                capture_output=True, text=True, timeout=CAPTURE_TIMEOUT_S)
    except FileNotFoundError:
        return 'ffmpeg not found'
    except subprocess.TimeoutExpired:
        return f'ffmpeg timed out after {CAPTURE_TIMEOUT_S:g} s'
    except OSError as e:
        return str(e)
    if result.returncode != 0:
        last = (result.stderr.strip().splitlines() or ['no output'])[-1]
        return f'ffmpeg exit {result.returncode}: {last}'
    if not path.exists():
        return 'ffmpeg wrote no file'
    return None


def camera_capture(cfg):
    """capture(n) -> (path, error) for run_scan, from the config's camera settings."""
    def capture(n):
        path = cfg.image_path(n)
        return path, capture_image(cfg.camera_device, path, cfg.skip_frames)
    return capture


CSV_FIELDS = (
    'index', 'iz', 'ix', 'iy', 'local_x', 'local_y', 'local_z',
    'target_x', 'target_y', 'target_z', 'pos_x', 'pos_y', 'pos_z',
    'error_mm', 'image', 'timestamp',
)


def run_scan(gantry, cfg, points, csv_file, home=True, return_home=True,
             capture=None, log=print, stop=None):
    """Move through every point, read pos after settling and log a CSV row.

    capture(n) -> (path, error), if given, is called at every point (n is
    1-based, in scan order) while the gantry is stationary; a failure is
    logged as a warning and the scan carries on.  csv_file is an open text
    file; rows are flushed as they are written so a Ctrl-C keeps everything
    recorded so far.  stop() is checked before every stop of the scan; once
    it returns True the scan raises GantryError('scan stopped').
    """
    writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
    writer.writeheader()
    if home:
        log('homing...')
        gantry.home()
    for label, p, groups in scan_moves(points, return_home):
        if stop is not None and stop():
            raise GantryError('scan stopped')
        for group in groups:
            gantry.move_to(group, cfg.speed_mm_s, cfg.accel_mm_s2)
        if p is None:
            log('back at 0/0/0')
            continue
        time.sleep(cfg.settle_s)
        pos = gantry.pos()
        error = max(abs(pos[axis] - p.target[axis]) for axis in 'xyz')
        image = ''
        if capture is not None:
            path, failure = capture(p.index + 1)
            if failure:
                log(f'{label:>4} WARNING image failed: {failure}')
            else:
                image = str(path)
        writer.writerow({
            'index': p.index, 'iz': p.iz, 'ix': p.ix, 'iy': p.iy,
            **{f'local_{k}': f'{v:.4f}' for k, v in p.local.items()},
            **{f'target_{k}': f'{v:.4f}' for k, v in p.target.items()},
            **{f'pos_{k}': f'{v:.4f}' for k, v in pos.items()},
            'error_mm': f'{error:.4f}',
            'image': image,
            'timestamp': datetime.now().isoformat(timespec='milliseconds'),
        })
        csv_file.flush()
        log(f'{label:>4}/{len(points)}  ({p.ix},{p.iy}) lvl {p.iz}  '
            f'X {pos["x"]:.4f}  Y {pos["y"]:.4f}  Z {pos["z"]:.4f}'
            + (f'  -> {Path(image).name}' if image else '')
            + (f'  WARNING off target by {error:.4f} mm'
               if error > cfg.pos_tolerance_mm else ''))


def estimate_scan_s(cfg, points, return_home=True):
    """Rough scan duration in s, excluding homing: moves, settling and pictures."""
    total = 0.0
    here = dict(HOME)
    for _, p, groups in scan_moves(points, return_home):
        for group in groups:
            dist = max(abs(mm - here[axis]) for axis, mm in group.items())
            if dist >= HALF_STEP_MM:
                total += dist / cfg.speed_mm_s + cfg.speed_mm_s / cfg.accel_mm_s2
            here.update(group)
        if p is not None:
            total += cfg.settle_s + (CAPTURE_ESTIMATE_S if cfg.camera_enabled else 0.0)
    return total


def prepare_scan(path):
    """Load and validate a config for a live scan: (cfg, points).

    Raises ScanConfigError (or OSError / yaml.YAMLError) if the scan can't run:
    bad config, calibrated false, or no camera/ffmpeg while pictures are on.
    """
    cfg = ScanConfig.from_yaml(path)
    points = scan_points(cfg)
    validate(cfg, points)
    if not cfg.calibrated:
        raise ScanConfigError(f'calibrated is false in {path}; check the offsets '
                              'with gantry_scan scan --dry-run, then set it to true')
    if cfg.camera_enabled:
        check_camera(cfg)
    return cfg, points


def default_config_path():
    """Installed share/ copy if built with colcon, else the source tree's."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('astrobiology_actuators')) / 'config'
    except Exception:   # not installed, or ROS not sourced
        path = Path(__file__).resolve().parents[1] / 'config'
    return path / CONFIG_NAME


# -- CLI ----------------------------------------------------------------------

def _cmd_pos(gantry, args):
    p = gantry.pos()
    print(f"X {p['x']:.4f}  Y {p['y']:.4f}  Z {p['z']:.4f}  (mm)")


def _cmd_home(gantry, args):
    gantry.home(tuple(args.axes))
    _cmd_pos(gantry, args)


def _cmd_goto(gantry, args):
    target = {axis: getattr(args, axis) for axis in 'xyz'
              if getattr(args, axis) is not None}
    gantry.move_safe(target, args.speed, args.accel)
    _cmd_pos(gantry, args)


def _load_scan(args):
    cfg = ScanConfig.from_yaml(args.config)
    if args.z_levels:
        cfg.z_levels_mm = tuple(args.z_levels)
    if args.no_images:
        cfg.camera_enabled = False
    if args.camera:
        cfg.camera_device = args.camera
    if args.image_dir:
        cfg.image_dir = args.image_dir.expanduser()
    points = scan_points(cfg)
    validate(cfg, points)
    print(f'config: {args.config}')
    print(f'{len(points)} points, Z levels {", ".join(f"{z:g}" for z in cfg.z_levels_mm)} mm '
          f'(+ z_offset {cfg.z_offset_mm:g}), speed {cfg.speed_mm_s:g} mm/s, '
          f'accel {cfg.accel_mm_s2:g} mm/s^2')
    if cfg.camera_enabled:
        print(f'images: {cfg.image_path(1)} .. {cfg.image_path(len(points)).name} '
              f'from {cfg.camera_device} (overwritten)')
    else:
        print('images: off')
    return cfg, points


def check_camera(cfg):
    """Fail before any motion if pictures can't possibly be taken."""
    if shutil.which('ffmpeg') is None:
        raise ScanConfigError('ffmpeg not found (sudo apt install ffmpeg), or use --no-images')
    if not Path(cfg.camera_device).exists():
        raise ScanConfigError(f'camera {cfg.camera_device} not found '
                              '(ls /dev/video*; pick one with --camera), or use --no-images')
    cfg.image_dir.mkdir(parents=True, exist_ok=True)


def _cmd_scan(gantry, args):
    out = args.out or Path('scans') / f'gantry_scan_{datetime.now():%Y%m%d_%H%M%S}.csv'
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f'logging to {out}')
    with open(out, 'w', newline='') as f:
        run_scan(gantry, args.cfg, args.points, f,
                 home=not args.skip_home, return_home=not args.no_return,
                 capture=camera_capture(args.cfg) if args.cfg.camera_enabled else None)
    print(f'done: {len(args.points)} points logged to {out}')


def _axis(name):
    # A type check, not choices=: Python 3.10 argparse rejects an empty
    # nargs='*' list against choices, so a bare 'home' would fail to parse.
    if name not in ('x', 'y', 'z'):
        raise argparse.ArgumentTypeError(f"invalid axis '{name}' (choose x, y, z)")
    return name


def build_parser():
    parser = argparse.ArgumentParser(
        prog='gantry_scan',
        description='Gantry jog tools and gantry grid scan (stop steppers_node first).')
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD)
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='print every line sent to / read from the ESP32')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('pos', help='print the X/Y/Z position').set_defaults(func=_cmd_pos)
    p = sub.add_parser('home', help='home onto the endstops (default: all axes)')
    p.add_argument('axes', nargs='*', type=_axis, metavar='{x,y,z}')
    p.set_defaults(func=_cmd_home)
    p = sub.add_parser('goto', help='absolute move in mm (axes not given stay put)')
    for axis in 'xyz':
        p.add_argument(f'--{axis}', type=float, metavar='MM')
    p.add_argument('--speed', type=float, default=DEFAULT_SPEED_MM_S, help='mm/s')
    p.add_argument('--accel', type=float, default=DEFAULT_ACCEL_MM_S2, help='mm/s^2')
    p.set_defaults(func=_cmd_goto)
    p = sub.add_parser('scan', help='run the gantry grid scan')
    p.add_argument('--config', type=Path, default=None,
                   help=f'scan config (default: installed {CONFIG_NAME})')
    p.add_argument('--dry-run', action='store_true',
                   help='print points and moves without opening the port')
    p.add_argument('--no-return', action='store_true',
                   help='stay at the last point instead of going back to 0/0/0')
    p.add_argument('--skip-home', action='store_true',
                   help="don't run g28 first (trust the current position)")
    p.add_argument('--z-levels', type=float, nargs='+', metavar='MM',
                   help='override z_levels_mm from the config, e.g. --z-levels 0 1 2')
    p.add_argument('--out', type=Path, default=None,
                   help='CSV path (default: scans/gantry_scan_<time>.csv)')
    p.add_argument('--no-images', action='store_true', help="don't take pictures")
    p.add_argument('--camera', metavar='DEVICE',
                   help='camera device, overriding the config (e.g. /dev/video0)')
    p.add_argument('--image-dir', type=Path, metavar='DIR',
                   help='where test1.jpg ... go, overriding the config')
    p.set_defaults(func=_cmd_scan)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == 'goto' and (args.x, args.y, args.z) == (None, None, None):
        parser.error('goto needs at least one of --x, --y, --z')
    if args.command == 'scan':
        args.config = args.config or default_config_path()
        try:
            args.cfg, args.points = _load_scan(args)
        except (OSError, yaml.YAMLError, ScanConfigError) as e:
            print(f'error: {e}', file=sys.stderr)
            return 1
        if args.dry_run:
            print()
            print('\n'.join(dry_run_lines(args.cfg, args.points,
                                           return_home=not args.no_return)))
            return 0
        if not args.cfg.calibrated:
            print(f'error: calibrated is false in {args.config}; check the offsets '
                  'with --dry-run, then set it to true', file=sys.stderr)
            return 1
        if args.cfg.camera_enabled:
            try:
                check_camera(args.cfg)
            except (OSError, ScanConfigError) as e:
                print(f'error: {e}', file=sys.stderr)
                return 1

    try:
        with Gantry(args.port, args.baud, verbose=args.verbose) as gantry:
            args.func(gantry, args)
    except (GantryError, OSError) as e:     # serial.SerialException is an OSError
        print(f'error: {e}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
