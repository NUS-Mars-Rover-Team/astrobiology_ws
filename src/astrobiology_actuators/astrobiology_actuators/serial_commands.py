"""Serial command strings for the ESP32 stepper controller.  No ROS imports.

The ESP32 accepts line-based commands:
    g1 <motor> <signed_steps> <speed_hz> <accel_hz2>
    stop <motor|all>   estop [motor|all]
    enable <motor|all> on|off   status [motor|all]   zero <motor|all>
    pos   ->   pos Z:<mm> Y:<mm> X:<mm> C:<rev> P:<pulses> F:<pulses>
    g28 [x] [y] [z]   home onto endstops (always Z, Y, X order); ends with 'done g28'

The new firmware's g1 takes units for x/y/z: g1 <axis> <mm> <mm/s> <mm/s^2>
(relative), replying 'ok <axis> accepted' then 'done <axis> steps=<N> pos=<mm>'.
    !     (single byte, no newline) instant emergency stop of everything

Motor IDs: pump (p), centrifuge (c), flush (f), x, y, z.
"""

import re

ESTOP_CHAR = b'!'

_POS_LINE_RE = re.compile(r'^pos(?:\s+[A-Z]:-?\d+(?:\.\d+)?)+\s*$')
_POS_FIELD_RE = re.compile(r'([A-Z]):(-?\d+(?:\.\d+)?)')

# 'done x steps=3200 pos=1.0000' (not 'done g28', which has no steps=)
DONE_RE = re.compile(r'^done\s+(\S+)\s+steps=(-?\d+)')


def move_command(motor_name, distance, speed, accel):
    """Build a relative G1 move in the motor's firmware units (signed distance).

    Numbers are written fixed-point with up to 4 decimals, e.g. 'g1 c 0.1669 10 20'.
    """
    return f'g1 {motor_name} {_num(distance)} {_num(speed)} {_num(accel)}'


def stop_command(motor_name):
    """Smooth deceleration stop for a specific motor."""
    return f'stop {motor_name}'


def estop_command():
    """Emergency stop all motors."""
    return 'estop'


def _num(value):
    """Fixed-point number for the firmware, which rejects exponents (1e-05)."""
    text = f'{value:.4f}'.rstrip('0').rstrip('.')
    return '0' if text in ('', '-0') else text


def move_mm_command(axis, mm, speed_mm_s, accel_mm_s2):
    """Relative gantry move in mm (new firmware units), e.g. 'g1 x 1.5 5 10'."""
    return f'g1 {axis} {_num(mm)} {_num(speed_mm_s)} {_num(accel_mm_s2)}'


def pos_command():
    """Ask for every motor's coordinate (reply parsed by parse_pos)."""
    return 'pos'


def home_command(*axes):
    """Home the listed gantry axes (none = every axis with an endstop)."""
    return ' '.join(('g28',) + axes)


def parse_pos(line):
    """Parse a 'pos Z:.. Y:.. X:..' reply into {'z': .., 'y': .., 'x': ..}.

    Returns None if the line is not a pos reply.  Field order doesn't matter.
    """
    if not _POS_LINE_RE.match(line):
        return None
    return {k.lower(): float(v) for k, v in _POS_FIELD_RE.findall(line)}


_AXIS_END_RE = re.compile(r'^(?:done|stopped)\s+([xyz])\s+steps=-?\d+\s+pos=(-?\d+(?:\.\d+)?)')
_HOMED_RE = re.compile(r'^homed\s+([xyz])\b')


def gantry_position_update(line):
    """Gantry positions (mm) reported by an ESP32 line, e.g. {'x': 12.5}; None if none.

    'pos ...' gives X, Y and Z; 'done x ... pos=12.5' or 'stopped x ...' one axis;
    'homed z ...' sets that axis to 0.
    """
    p = parse_pos(line)
    if p is not None:
        return {axis: p[axis] for axis in 'xyz' if axis in p}
    m = _AXIS_END_RE.match(line)
    if m:
        return {m.group(1): float(m.group(2))}
    m = _HOMED_RE.match(line)
    if m:
        return {m.group(1): 0.0}
    return None


_HOMING_RE = re.compile(r'^homing\s+([xyz])\b')
_HOMING_FAILED_RE = re.compile(r'^error homing failed\s+([xyz])\b')
_ABORTED_RE = re.compile(r'^aborted\s+([xyz])\b')


def gantry_homed_update(line):
    """Which gantry axes an ESP32 line homes (True) or un-homes (False); None if none.

    'homed x' -> {'x': True}.  'homing x' (position being redefined), 'error
    homing failed x', 'aborted x' (estop) -> {'x': False}.  'ESTOP all' -> all False.
    """
    m = _HOMED_RE.match(line)
    if m:
        return {m.group(1): True}
    for regex in (_HOMING_RE, _HOMING_FAILED_RE, _ABORTED_RE):
        m = regex.match(line)
        if m:
            return {m.group(1): False}
    if line.startswith('ESTOP all'):
        return dict.fromkeys('xyz', False)
    return None
