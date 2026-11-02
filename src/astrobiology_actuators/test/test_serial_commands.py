from astrobiology_actuators.serial_commands import (
    estop_command,
    move_command,
    parse_pos,
    pos_command,
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


def test_pos_command():
    assert pos_command() == 'pos'


def test_parse_pos_firmware_line():
    line = 'pos Z:40.0000 Y:24.5000 X:25.5000 C:0.0000 P:0 F:0'
    assert parse_pos(line) == {
        'z': 40.0, 'y': 24.5, 'x': 25.5, 'c': 0.0, 'p': 0.0, 'f': 0.0}


def test_parse_pos_negative():
    assert parse_pos('pos Z:-0.0003 Y:0.0000 X:-12.5000')['z'] == -0.0003


def test_parse_pos_rejects_other_lines():
    assert parse_pos('done x steps=3200 pos=1.0000') is None
    assert parse_pos('ok x accepted') is None
    assert parse_pos('pos') is None
