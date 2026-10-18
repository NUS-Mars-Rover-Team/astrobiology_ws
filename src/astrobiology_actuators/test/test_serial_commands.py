from astrobiology_actuators.serial_commands import (
    estop_command,
    move_command,
    stop_command,
)


def test_move_command():
    assert move_command('pump', 5000, 800, 400) == 'g1 pump 5000 800 400'


def test_move_command_negative_steps():
    assert move_command('pump', -3200, 800, 400) == 'g1 pump -3200 800 400'


def test_move_command_short_name():
    assert move_command('p', 100, 5000, 400) == 'g1 p 100 5000 400'


def test_stop():
    assert stop_command('pump') == 'stop pump'


def test_estop():
    assert estop_command() == 'estop'
