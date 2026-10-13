"""Serial command strings for the pump ESP32 (FastAccelStepper firmware).  No ROS imports.

The ESP32 accepts line-based commands:
    speed <steps/s>   accel <steps/s^2>   dir cw|ccw
    time <seconds>    run   stop   estop
A dispense is a sequence: set speed, accel, direction, time, then run.

# TODO(firmware): add motor ID prefix when firmware supports 3 motors.
"""


def dispense_commands(speed_steps_s, accel_steps_s2, direction, time_s):
    """Return the list of serial commands to start a timed dispense."""
    return [
        f'speed {speed_steps_s}',
        f'accel {accel_steps_s2}',
        f'dir {direction}',
        f'time {time_s:.4f}',
        'run',
    ]


def stop_command():
    """Smooth deceleration stop."""
    return 'stop'


def estop_command():
    """Emergency stop (no deceleration)."""
    return 'estop'
