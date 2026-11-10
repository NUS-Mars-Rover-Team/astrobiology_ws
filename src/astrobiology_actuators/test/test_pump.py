import pytest

from astrobiology_actuators.pump import (
    PumpConfig, PumpState, State, SAFETY_TIMEOUT_MARGIN_S, move_time_s,
)


@pytest.fixture
def config():
    # speed=5000 steps/s, accel=5000 steps/s², steps_per_ml=1000 → 1 ml = 1000 steps.
    # Full speed is reached after 5000 steps of ramp (2500 up + 2500 down).
    return PumpConfig(
        pump_id=1, motor_name='pump', speed_steps_s=5000, accel_steps_s2=5000,
        direction='cw', steps_per_ml=1000.0, max_volume_ml=20.0,
    )


@pytest.fixture
def pump(config):
    return PumpState(config)


# -- Dispense success --------------------------------------------------------

def test_dispense_success(pump):
    ok, msg, expected_s = pump.dispense(5.0, now=0.0)
    assert ok is True
    # 5000 steps: 5000/5000 + ramps 5000/5000 = 2.0 s
    assert expected_s == pytest.approx(2.0)
    assert pump.is_busy


def test_dispense_expected_s(pump):
    ok, _, expected_s = pump.dispense(10.0, now=0.0)
    assert ok is True
    # 10000 steps: 10000/5000 + 5000/5000 = 3.0 s
    assert expected_s == pytest.approx(3.0)


def test_dispense_computes_steps(pump):
    pump.dispense(5.0, now=0.0)
    # 5 ml * 1000 steps/ml = 5000 steps, cw = positive
    assert pump.commanded_steps == 5000


def test_dispense_ccw_negative_steps():
    cfg = PumpConfig(
        pump_id=1, motor_name='pump', speed_steps_s=5000, accel_steps_s2=400,
        direction='ccw', steps_per_ml=1000.0, max_volume_ml=20.0,
    )
    p = PumpState(cfg)
    p.dispense(3.0, now=0.0)
    assert p.commanded_steps == -3000


# -- Direction sign ----------------------------------------------------------

def test_direction_sign_cw(config):
    assert config.direction_sign == 1


def test_direction_sign_ccw():
    cfg = PumpConfig(
        pump_id=1, motor_name='pump', speed_steps_s=5000, accel_steps_s2=400,
        direction='ccw', steps_per_ml=1000.0, max_volume_ml=20.0,
    )
    assert cfg.direction_sign == -1


# -- Dispense rejection: volume out of range ---------------------------------

def test_dispense_zero_volume(pump):
    ok, msg, _ = pump.dispense(0.0, now=0.0)
    assert ok is False
    assert 'out of range' in msg


def test_dispense_negative_volume(pump):
    ok, msg, _ = pump.dispense(-1.0, now=0.0)
    assert ok is False


def test_dispense_over_max(pump):
    ok, msg, _ = pump.dispense(25.0, now=0.0)
    assert ok is False
    assert 'out of range' in msg


# -- Dispense rejection: busy ------------------------------------------------

def test_dispense_busy_self(pump):
    pump.dispense(5.0, now=0.0)
    ok, msg, _ = pump.dispense(1.0, now=0.5)
    assert ok is False
    assert 'busy' in msg


# -- Timer-based safety timeout ----------------------------------------------

def test_tick_completes(pump):
    pump.dispense(5.0, now=0.0)  # expected_s = 2.0, timeout = 2.0 + margin
    timeout = 2.0 + SAFETY_TIMEOUT_MARGIN_S
    pump.tick(now=timeout - 0.1)
    assert pump.is_busy
    pump.tick(now=timeout)
    assert not pump.is_busy


def test_tick_noop_when_idle(pump):
    pump.tick(now=100.0)
    assert not pump.is_busy


# -- Stop --------------------------------------------------------------------

def test_stop_while_dispensing(pump):
    pump.dispense(5.0, now=0.0)
    ok, msg = pump.stop()
    assert ok is True
    assert 'stopped' in msg
    assert not pump.is_busy
    assert pump.commanded_steps == 0


def test_stop_while_idle(pump):
    ok, msg = pump.stop()
    assert ok is True
    assert 'idle' in msg


# -- Hardware completion -----------------------------------------------------

def test_complete(pump):
    pump.dispense(5.0, now=0.0)
    pump.complete()
    assert not pump.is_busy
    assert pump.commanded_steps == 0


def test_complete_when_idle(pump):
    pump.complete()  # should not raise
    assert not pump.is_busy


# -- State after operations --------------------------------------------------

def test_dispense_after_stop(pump):
    pump.dispense(5.0, now=0.0)
    pump.stop()
    ok, _, _ = pump.dispense(3.0, now=1.0)
    assert ok is True


def test_dispense_after_complete(pump):
    pump.dispense(5.0, now=0.0)
    pump.complete()
    ok, _, _ = pump.dispense(3.0, now=1.0)
    assert ok is True


# -- Default dose and ramp timing --------------------------------------------

def _cfg(**kw):
    base = dict(pump_id=1, motor_name='f', speed_steps_s=30000, accel_steps_s2=5000,
                direction='ccw', steps_per_ml=1000.0, max_volume_ml=50.0,
                default_steps=100000)
    base.update(kw)
    return PumpConfig(**base)


def test_default_dose_when_no_volume():
    p = PumpState(_cfg())
    ok, msg, expected_s = p.dispense(0.0, now=0.0)
    assert ok and p.commanded_steps == -100000
    assert 'default dose' in msg
    # never reaches 30000/s in 100000 steps: 2*sqrt(100000/5000)
    assert expected_s == pytest.approx(2 * 20 ** 0.5)


def test_default_dose_ignores_volume():
    p = PumpState(_cfg())
    ok, _, _ = p.dispense(2.0, now=0.0)
    assert ok and p.commanded_steps == -100000


def test_no_default_dose_refuses_zero():
    p = PumpState(_cfg(default_steps=0))
    ok, msg, _ = p.dispense(0.0, now=0.0)
    assert not ok and msg == 'out of range'


def test_move_time_trapezoid():
    # pump: 150000 steps at 20000/s, 5000/s²: 7.5 s cruise-equivalent + 4 s of ramps
    assert move_time_s(150000, 20000, 5000) == pytest.approx(11.5)
