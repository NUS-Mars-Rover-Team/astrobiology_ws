import pytest

from astrobiology_actuators.pump import PumpConfig, PumpState, State, SAFETY_TIMEOUT_MARGIN_S


@pytest.fixture
def config():
    # speed=5000 steps/s, steps_per_ml=1000 → 1 ml = 1000 steps
    return PumpConfig(
        pump_id=1, motor_name='pump', speed_steps_s=5000, accel_steps_s2=400,
        direction='cw', steps_per_ml=1000.0, max_volume_ml=20.0,
    )


@pytest.fixture
def pump(config):
    return PumpState(config)


# -- Dispense success --------------------------------------------------------

def test_dispense_success(pump):
    ok, msg, expected_s = pump.dispense(5.0, now=0.0)
    assert ok is True
    # 5 ml * 1000 steps/ml / 5000 steps/s = 1.0 s
    assert expected_s == pytest.approx(1.0)
    assert pump.is_busy


def test_dispense_expected_s(pump):
    ok, _, expected_s = pump.dispense(10.0, now=0.0)
    assert ok is True
    # 10 ml * 1000 / 5000 = 2.0 s
    assert expected_s == pytest.approx(2.0)


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
    pump.dispense(5.0, now=0.0)  # expected_s = 1.0, timeout = 1.0 + margin
    timeout = 1.0 + SAFETY_TIMEOUT_MARGIN_S
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
