"""Centrifuge spins and carousel steps for firmware motor c.  No ROS imports.

The firmware drives c in revolutions, RPM and RPM/s (g1 c <rev> <RPM> <RPM/s>) at
STEPS_PER_REV motor steps per revolution, and rounds each move to whole steps.
"""

import math
import re

from astrobiology_actuators.serial_commands import parse_pos

# SixMotorController.ino MOTOR TABLE, c: 200 steps/rev x 1/8 microstep
STEPS_PER_REV = 1600
# Firmware MAX_SPEED_HZ (100000 steps/s) in RPM
FIRMWARE_MAX_RPM = 100000 * 60 / STEPS_PER_REV


def move_time_s(revs, rpm, accel_rpm_s):
    """Rough duration of a move: cruise plus one ramp up and one ramp down."""
    return abs(revs) / rpm * 60 + rpm / accel_rpm_s


def spin_plan(rpm, duration_s, max_rpm):
    """Revolutions for spinning at rpm for duration_s; ValueError if out of range."""
    if not (math.isfinite(rpm) and 0 < rpm <= min(max_rpm, FIRMWARE_MAX_RPM)):
        raise ValueError(f'rpm must be more than 0 and at most {min(max_rpm, FIRMWARE_MAX_RPM):g}')
    if not (math.isfinite(duration_s) and duration_s > 0):
        raise ValueError('duration_s must be more than 0')
    return rpm * duration_s / 60.0


class CarouselStepper:
    """Turns carousel angles into motor revolutions on the whole-step grid.

    Each turn is rounded to whole motor steps and the rounding left over is carried
    into the next turn, so repeated small steps (e.g. 60 x 1 degree) add up exactly
    instead of drifting.  gear_ratio is motor revolutions per carousel revolution.
    """

    def __init__(self, gear_ratio=1.0):
        self.gear_ratio = gear_ratio
        self._residual_steps = 0.0

    def turn(self, degrees):
        """Signed motor revolutions for turning the carousel by `degrees`."""
        exact = degrees / 360.0 * self.gear_ratio * STEPS_PER_REV + self._residual_steps
        steps = round(exact)
        self._residual_steps = exact - steps
        return steps / STEPS_PER_REV


_C_END_RE = re.compile(r'^(?:done|stopped)\s+c\s+steps=-?\d+\s+pos=(-?\d+(?:\.\d+)?)')
# Lines after which the firmware's step count for c no longer matches the wheel:
# an estop (driver off, the wheel coasts) or the ESP32 restarting (count back to 0)
_REFERENCE_LOST = ('aborted c', 'ESTOP all', 'Six motor controller ready')


def c_revs_update(line):
    """Motor c's position in revolutions reported by an ESP32 line, or None.

    From 'pos ... C:12.1669 ...' or 'done c steps=19467 pos=12.1669' (or 'stopped c ...').
    """
    p = parse_pos(line)
    if p is not None:
        return p.get('c')
    m = _C_END_RE.match(line)
    return float(m.group(1)) if m else None


def carousel_reference_lost(line):
    return line.startswith(_REFERENCE_LOST)


def carousel_angle(revs, gear_ratio=1.0, cw_positive=True):
    """Carousel angle in degrees clockwise from the reference, 0 <= angle < 360."""
    sign = 1 if cw_positive else -1
    return (sign * revs / gear_ratio * 360.0) % 360.0
