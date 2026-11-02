"""Serial command strings for the ESP32 stepper controller.  No ROS imports.

The ESP32 accepts line-based commands:
    g1 <motor> <signed_steps> <speed_hz> <accel_hz2>
    stop <motor|all>   estop [motor|all]
    enable <motor|all> on|off   status [motor|all]   zero <motor|all>
    pos   ->   pos Z:<mm> Y:<mm> X:<mm> C:<rev> P:<pulses> F:<pulses>
    !     (single byte, no newline) instant emergency stop of everything

Motor IDs: pump (p), centrifuge (c), flush (f), x, y, z.
"""

import re

ESTOP_CHAR = b'!'

_POS_LINE_RE = re.compile(r'^pos(?:\s+[A-Z]:-?\d+(?:\.\d+)?)+\s*$')
_POS_FIELD_RE = re.compile(r'([A-Z]):(-?\d+(?:\.\d+)?)')


def move_command(motor_name, steps, speed_hz, accel_hz2):
    """Build a G1 move command.  Steps are signed (positive = forward)."""
    return f'g1 {motor_name} {steps} {speed_hz} {accel_hz2}'


def stop_command(motor_name):
    """Smooth deceleration stop for a specific motor."""
    return f'stop {motor_name}'


def estop_command():
    """Emergency stop all motors."""
    return 'estop'


def pos_command():
    """Ask for every motor's coordinate (reply parsed by parse_pos)."""
    return 'pos'


def parse_pos(line):
    """Parse a 'pos Z:.. Y:.. X:..' reply into {'z': .., 'y': .., 'x': ..}.

    Returns None if the line is not a pos reply.  Field order doesn't matter.
    """
    if not _POS_LINE_RE.match(line):
        return None
    return {k.lower(): float(v) for k, v in _POS_FIELD_RE.findall(line)}
