"""Pure-Python helpers for the science lights serial protocol.

Protocol (firmware/LEDdrill.ino): send "<name> on\\n" or "<name> off\\n",
receive "<NAME> ON" / "<NAME> OFF", or "Invalid command: ..." if not recognised.
"""

# service_name -> firmware command name
LIGHTS = {
    'uvm':     'uvm',
    'uvc':     'uvc',
    'laser':   'laser',
    'halogen': 'halogen',
}


def light_command(fw_name, on):
    """Build a light on/off command string (no newline)."""
    return f'{fw_name} {"on" if on else "off"}'


def all_off_commands():
    """Return the list of commands to turn all lights off."""
    return [light_command(fw, False) for fw in LIGHTS.values()]
