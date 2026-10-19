"""Serial command strings for the ESP32 stepper controller.  No ROS imports.

The ESP32 accepts line-based commands:
    g1 <motor> <signed_steps> <speed_hz> <accel_hz2>
    stop <motor|all>   estop [motor|all]
    enable <motor|all> on|off   status [motor|all]   zero <motor|all>

Motor IDs: pump (p), centrifuge (c), flush (f), x, y, z.
"""


def move_command(motor_name, steps, speed_hz, accel_hz2):
    """Build a G1 move command.  Steps are signed (positive = forward)."""
    return f'g1 {motor_name} {steps} {speed_hz} {accel_hz2}'


def stop_command(motor_name):
    """Smooth deceleration stop for a specific motor."""
    return f'stop {motor_name}'


def estop_command():
    """Emergency stop all motors."""
    return 'estop'
