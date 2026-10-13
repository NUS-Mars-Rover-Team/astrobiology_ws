"""Pump state machine.  No ROS imports — tested with plain pytest."""

from dataclasses import dataclass
from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    DISPENSING = auto()


@dataclass
class PumpConfig:
    pump_id: int
    speed_steps_s: int      # motor speed in steps/s (from YAML)
    accel_steps_s2: int     # acceleration in steps/s² (from YAML)
    direction: str          # 'cw' or 'ccw' (from YAML)
    steps_per_ml: float     # for converting volume_ml -> run time (calibrate)
    max_volume_ml: float


class PumpState:
    """State for a single peristaltic pump.

    Not thread-safe; the caller (pump_node) serialises access with a lock.
    """

    def __init__(self, config):
        self.config = config
        self.state = State.IDLE
        self._finish_time = 0.0

    def dispense(self, volume_ml, now):
        """Try to start a dispense.

        Returns (success, message, expected_s).
        The caller must check the shared-busy constraint across all pumps
        *before* calling this method.  On success the caller reads
        config.speed_steps_s / accel_steps_s2 / direction and expected_s
        to build the serial commands.
        """
        if self.state == State.DISPENSING:
            return (False, 'busy', 0.0)
        if volume_ml <= 0.0:
            return (False, 'out of range', 0.0)
        if volume_ml > self.config.max_volume_ml:
            return (False, 'out of range', 0.0)

        # time_s = volume * (steps/ml) / (steps/s)
        time_s = volume_ml * self.config.steps_per_ml / self.config.speed_steps_s
        if time_s <= 0.0:
            return (False, 'out of range', 0.0)

        self.state = State.DISPENSING
        self._finish_time = now + time_s
        return (True, f'dispensing {volume_ml:.2f} ml ({time_s:.3f} s)', time_s)

    def stop(self):
        """Force-stop.  Always succeeds."""
        was_busy = self.state == State.DISPENSING
        self.state = State.IDLE
        self._finish_time = 0.0
        return (True, 'stopped' if was_busy else 'already idle')

    def complete(self):
        """Hardware reported completion ("Stopped." / "EMERGENCY STOP")."""
        self.state = State.IDLE
        self._finish_time = 0.0

    def tick(self, now):
        """Advance timer-based completion.  Call periodically."""
        if self.state == State.DISPENSING and now >= self._finish_time:
            self.state = State.IDLE

    @property
    def is_busy(self):
        return self.state == State.DISPENSING
