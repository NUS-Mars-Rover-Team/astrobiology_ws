import pytest

from astrobiology_actuators.centrifuge import (
    FIRMWARE_MAX_RPM,
    STEPS_PER_REV,
    CarouselStepper,
    c_revs_update,
    carousel_angle,
    carousel_reference_lost,
    move_time_s,
    spin_plan,
)
from astrobiology_actuators.serial_commands import move_command


def test_spin_plan_revolutions():
    # GUI default: 1300 RPM for 5 min
    assert spin_plan(1300, 300, max_rpm=3000) == pytest.approx(6500)


@pytest.mark.parametrize('rpm, duration', [(0, 300), (-5, 300), (3001, 300), (1300, 0),
                                           (float('nan'), 300), (1300, float('inf'))])
def test_spin_plan_rejects(rpm, duration):
    with pytest.raises(ValueError):
        spin_plan(rpm, duration, max_rpm=3000)


def test_spin_plan_capped_by_firmware():
    assert FIRMWARE_MAX_RPM == 3750
    with pytest.raises(ValueError, match='at most 3750'):
        spin_plan(3800, 10, max_rpm=10000)


def test_move_time():
    # 6500 rev at 1300 RPM = 300 s, plus 13 s of ramps at 100 RPM/s
    assert move_time_s(6500, 1300, 100) == pytest.approx(313)


def test_carousel_60_degrees_lands_on_whole_steps():
    c = CarouselStepper()
    revs = [c.turn(60) for _ in range(6)]
    for r in revs:
        assert (r * STEPS_PER_REV) == pytest.approx(round(r * STEPS_PER_REV))
    assert sum(revs) == pytest.approx(1.0)          # a full turn, no drift
    assert move_command('c', revs[0], 10, 20) == 'g1 c 0.1669 10 20'


def test_carousel_small_steps_do_not_drift():
    c = CarouselStepper()
    total = sum(c.turn(1) for _ in range(360))
    assert total == pytest.approx(1.0)
    c = CarouselStepper()
    assert sum(c.turn(-5) for _ in range(72)) == pytest.approx(-1.0)


def test_carousel_gear_ratio():
    c = CarouselStepper(gear_ratio=3.0)
    assert c.turn(120) == pytest.approx(1.0)


def test_firmware_rounds_sent_revs_to_the_intended_steps():
    # 4 decimals of rev is 0.16 step, so the firmware's llround recovers the step count
    c = CarouselStepper()
    for _ in range(100):
        revs = c.turn(1)
        sent = float(move_command('c', revs, 10, 20).split()[2])
        assert round(sent * STEPS_PER_REV) == round(revs * STEPS_PER_REV)


def test_c_revs_update():
    assert c_revs_update('pos Z:0.0000 Y:0.0000 X:0.0000 C:12.1669 P:0 F:0') == pytest.approx(12.1669)
    assert c_revs_update('done c steps=267 pos=0.1669') == pytest.approx(0.1669)
    assert c_revs_update('stopped c steps=-22 pos=-0.0138') == pytest.approx(-0.0138)
    assert c_revs_update('done x steps=3200 pos=1.0000') is None
    assert c_revs_update('ok c accepted') is None


def test_carousel_angle_wraps():
    assert carousel_angle(0.1669) == pytest.approx(60.08, abs=0.01)
    assert carousel_angle(6500.0) == pytest.approx(0.0)
    assert carousel_angle(-0.0138) == pytest.approx(360 - 4.968, abs=0.01)
    assert carousel_angle(0.25, cw_positive=False) == pytest.approx(270.0)
    assert carousel_angle(1.5, gear_ratio=3.0) == pytest.approx(180.0)


def test_carousel_reference_lost():
    assert carousel_reference_lost('aborted c; physical position may be lost')
    assert carousel_reference_lost('ESTOP all (!); physical position may be lost')
    assert carousel_reference_lost('Six motor controller ready; type diag to check ISR rate.')
    assert not carousel_reference_lost('aborted x; physical position may be lost')
    assert not carousel_reference_lost('done c steps=267 pos=0.1669')
