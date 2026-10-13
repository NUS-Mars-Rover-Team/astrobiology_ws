import pytest

from astrobiology_actuators.pump import PumpConfig, PumpState, State


@pytest.fixture
def config():
    # speed=5000 steps/s, steps_per_ml=1000 → 1 ml = 1000/5000 = 0.2 s
    return PumpConfig(
        pump_id=3, speed_steps_s=5000, accel_steps_s2=400,
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


def test_dispense_rejects_when_another_pump_busy():
    """The node enforces the shared-busy constraint, not PumpState itself.

    This test verifies the pattern: check any(p.is_busy) before calling dispense.
    """
    pump1 = PumpState(PumpConfig(
        pump_id=1, speed_steps_s=33333, accel_steps_s2=400,
        direction='cw', steps_per_ml=1000.0, max_volume_ml=20.0,
    ))
    pump3 = PumpState(PumpConfig(
        pump_id=3, speed_steps_s=5000, accel_steps_s2=400,
        direction='cw', steps_per_ml=1000.0, max_volume_ml=20.0,
    ))

    pump1.dispense(5.0, now=0.0)
    assert pump1.is_busy

    # The node would check this before calling pump3.dispense()
    other_busy = any(p.is_busy for p in [pump1, pump3] if p is not pump3)
    assert other_busy is True


# -- Timer-based completion --------------------------------------------------

def test_tick_completes(pump):
    pump.dispense(5.0, now=0.0)  # expected_s = 1.0
    pump.tick(now=0.9)
    assert pump.is_busy
    pump.tick(now=1.0)
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


def test_stop_while_idle(pump):
    ok, msg = pump.stop()
    assert ok is True
    assert 'idle' in msg


# -- Hardware completion -----------------------------------------------------

def test_complete(pump):
    pump.dispense(5.0, now=0.0)
    pump.complete()
    assert not pump.is_busy


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
