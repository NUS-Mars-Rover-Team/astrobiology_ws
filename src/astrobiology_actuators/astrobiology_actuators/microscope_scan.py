"""Microscope grid scan over the X/Y/Z gantry.  No ROS imports.

Talks to the ESP32 stepper controller directly over serial, so pump_node
must NOT be running (it holds the same port).

Subcommands:
    microscope_scan pos          print the gantry X/Y/Z position in mm
    microscope_scan home [x y z] home onto the endstops (firmware order Z, Y, X)
    microscope_scan goto --x 1   absolute move in mm (axes not given stay put)
"""

import argparse
import sys
import time

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

# Reply prefixes (lower-cased) that mean the firmware rejected or aborted something
_FAULT_PREFIXES = ('error', 'aborted', 'estop')


class GantryError(RuntimeError):
    """The firmware reported a fault, or an expected reply never arrived."""


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

    def estop(self):
        """Instant stop of every motor.  Drivers are disabled: re-home after."""
        self._ser.write(ESTOP_CHAR)
        self._moving = False

    def close(self):
        if self._moving:
            self.estop()
            self._log('motion interrupted: sent ! (estop); re-home before the next run')
        self._ser.close()


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
    gantry.move_to(target, args.speed, args.accel)
    _cmd_pos(gantry, args)


def _axis(name):
    # A type check, not choices=: Python 3.10 argparse rejects an empty
    # nargs='*' list against choices, so a bare 'home' would fail to parse.
    if name not in ('x', 'y', 'z'):
        raise argparse.ArgumentTypeError(f"invalid axis '{name}' (choose x, y, z)")
    return name


def build_parser():
    parser = argparse.ArgumentParser(
        prog='microscope_scan',
        description='Gantry jog tools and microscope grid scan (stop pump_node first).')
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
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == 'goto' and (args.x, args.y, args.z) == (None, None, None):
        parser.error('goto needs at least one of --x, --y, --z')

    import serial
    try:
        with Gantry(args.port, args.baud, verbose=args.verbose) as gantry:
            args.func(gantry, args)
    except (GantryError, serial.SerialException) as e:
        print(f'error: {e}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
