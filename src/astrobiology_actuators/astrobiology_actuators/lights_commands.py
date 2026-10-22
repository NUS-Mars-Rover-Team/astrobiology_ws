"""Pure-Python helpers for the science lights serial protocol.

Protocol: send "<NAME> <0|1>\\n" to the ESP32, receive echo.
"""

# service_name -> firmware command name
LIGHTS = {
    'uv1':     'UV1',
    'uv2':     'UV2',
    'laser':   'LAS',
    'halogen': 'HAL',
}


def light_command(fw_name, on):
    """Build a light on/off command string (no newline)."""
    return f'{fw_name} {1 if on else 0}'


def all_off_commands():
    """Return the list of commands to turn all lights off."""
    return [light_command(fw, False) for fw in LIGHTS.values()]
