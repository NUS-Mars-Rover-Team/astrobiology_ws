"""Pump state machine.  No ROS imports — tested with plain pytest."""

from dataclasses import dataclass
from enum import Enum, auto

SAFETY_TIMEOUT_MARGIN_S = 2.0


class State(Enum):
    IDLE = auto()
    DISPENSING = auto()


@dataclass
class PumpConfig:
    pump_id: int
    motor_name: str         # firmware motor name (e.g. 'pump', 'flush')
    speed_steps_s: int      # motor speed in steps/s (from YAML)
    accel_steps_s2: int     # acceleration in steps/s² (from YAML)
    direction: str          # 'cw' or 'ccw' (from YAML)
    steps_per_ml: float     # for converting volume_ml -> step count (calibrate)
    max_volume_ml: float

    @property
    def direction_sign(self):
        """Return +1 for 'cw', -1 for 'ccw'."""
        return 1 if self.direction == 'cw' else -1


class PumpState:
    """State for a single peristaltic pump.

    Not thread-safe; the caller (pump_node) serialises access with a lock.
    """

    def __init__(self, config):
        self.config = config
        self.state = State.IDLE
        self._finish_time = 0.0
        self._commanded_steps = 0

    def dispense(self, volume_ml, now):
        """Try to start a dispense.

        Returns (success, message, expected_s).
        On success the caller reads commanded_steps, config.speed_steps_s,
        and config.accel_steps_s2 to build the g1 serial command.
        """
        if self.state == State.DISPENSING:
            return (False, 'busy', 0.0)
        if volume_ml <= 0.0:
            return (False, 'out of range', 0.0)
        if volume_ml > self.config.max_volume_ml:
            return (False, 'out of range', 0.0)

        steps = int(round(volume_ml * self.config.steps_per_ml))
        if steps <= 0:
            return (False, 'out of range', 0.0)

        self._commanded_steps = self.config.direction_sign * steps

        # Estimate run time (ignoring accel/decel ramps)
        time_s = steps / self.config.speed_steps_s

        self.state = State.DISPENSING
        self._finish_time = now + time_s + SAFETY_TIMEOUT_MARGIN_S
        return (True, f'dispensing {volume_ml:.2f} ml ({steps} steps)', time_s)

    def stop(self):
        """Force-stop.  Always succeeds."""
        was_busy = self.state == State.DISPENSING
        self.state = State.IDLE
        self._finish_time = 0.0
        self._commanded_steps = 0
        return (True, 'stopped' if was_busy else 'already idle')

    def complete(self):
        """Hardware reported completion (firmware 'done' / 'stopped')."""
        self.state = State.IDLE
        self._finish_time = 0.0
        self._commanded_steps = 0

    def tick(self, now):
        """Safety timeout.  Call periodically."""
        if self.state == State.DISPENSING and now >= self._finish_time:
            self.state = State.IDLE

    @property
    def commanded_steps(self):
        return self._commanded_steps

    @property
    def is_busy(self):
        return self.state == State.DISPENSING
