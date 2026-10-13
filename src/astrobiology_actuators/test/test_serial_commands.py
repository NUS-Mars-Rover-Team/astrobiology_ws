from astrobiology_actuators.serial_commands import (
    dispense_commands,
    estop_command,
    stop_command,
)


def test_dispense_commands():
    cmds = dispense_commands(5000, 400, 'cw', 1.0)
    assert cmds == ['speed 5000', 'accel 400', 'dir cw', 'time 1.0000', 'run']


def test_dispense_commands_ccw():
    cmds = dispense_commands(20000, 800, 'ccw', 2.5)
    assert cmds[2] == 'dir ccw'
    assert cmds[3] == 'time 2.5000'


def test_dispense_commands_length():
    cmds = dispense_commands(5000, 400, 'cw', 1.0)
    assert len(cmds) == 5
    assert cmds[-1] == 'run'


def test_stop():
    assert stop_command() == 'stop'


def test_estop():
    assert estop_command() == 'estop'
