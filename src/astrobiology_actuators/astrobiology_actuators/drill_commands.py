"""Pure-Python helpers for the drill serial protocol.

Protocol (firmware/LAdrill.ino, 9600 baud): single characters, newlines ignored.
  f -> "Motor: FORWARD", b -> "Motor: BACKWARD", s -> "Motor: STOP (Coasting)",
  anything else -> "Invalid command. Use f, b, or s."
"""

START = 'b'   # drill runs backward
STOP = 's'    # PWM to 0 and driver disabled, so the motor coasts
