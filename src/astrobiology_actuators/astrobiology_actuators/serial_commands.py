"""Serial command strings for the pump ESP32.  No ROS imports."""

# pump_id -> (motor_id, direction, speed_us)
PUMP_PROFILES = {
    1: (1, 'f', 30),
    2: (2, 'f', 50),
    3: (3, 'f', 200),
}


def dispense_command(pump_id, steps):
    """Return the serial command string for a dispense.

    Format: '<motor_id> <direction> <steps> <speed_us>'
    """
    motor_id, direction, speed_us = PUMP_PROFILES[pump_id]
    return f'{motor_id} {direction} {steps} {speed_us}'


def stop_command(pump_id):
    """Return the serial command to stop a specific pump motor.

    # TODO(firmware): verify per-motor stop command format with ESP32 firmware.
    """
    motor_id = PUMP_PROFILES[pump_id][0]
    return f'{motor_id} s'  # TODO(firmware)


def stop_all_command():
    """Return the serial command to stop all motors (used on shutdown)."""
    return 'x'
