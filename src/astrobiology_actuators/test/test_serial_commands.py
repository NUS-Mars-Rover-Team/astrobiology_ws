from astrobiology_actuators.serial_commands import (
    PUMP_PROFILES,
    dispense_command,
    stop_all_command,
    stop_command,
)


def test_pump3_dispense():
    assert dispense_command(3, 100000) == '3 f 100000 200'


def test_pump1_dispense():
    assert dispense_command(1, 500) == '1 f 500 30'


def test_pump2_dispense():
    assert dispense_command(2, 1000) == '2 f 1000 50'


def test_stop_command():
    assert stop_command(3) == '3 s'
    assert stop_command(1) == '1 s'
    assert stop_command(2) == '2 s'


def test_stop_all():
    assert stop_all_command() == 'x'


def test_all_pump_ids_present():
    assert set(PUMP_PROFILES.keys()) == {1, 2, 3}
