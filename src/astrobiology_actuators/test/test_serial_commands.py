from astrobiology_actuators.serial_commands import (
    DONE_RE,
    estop_command,
    home_command,
    move_mm_command,
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


def test_home_command():
    assert home_command() == 'g28'
    assert home_command('x', 'y') == 'g28 x y'


def test_move_mm_command():
    assert move_mm_command('x', 1.0, 5, 10) == 'g1 x 1 5 10'
    assert move_mm_command('z', -2.5, 2.5, 10.0) == 'g1 z -2.5 2.5 10'


def test_move_mm_command_never_uses_exponents():
    assert move_mm_command('y', 0.0003125, 0.00001, 300) == 'g1 y 0.0003 0 300'
    assert 'e' not in move_mm_command('y', 1e-5, 1e-5, 1e-5).split(' ', 2)[2]


def test_done_re():
    m = DONE_RE.match('done x steps=3200 pos=1.0000')
    assert m.group(1) == 'x' and m.group(2) == '3200'
    assert DONE_RE.match('done g28') is None


def test_gantry_position_update():
    from astrobiology_actuators.serial_commands import gantry_position_update as upd
    assert upd('pos Z:3.0000 Y:2.0000 X:1.0000 C:0.5000 P:0 F:0') == {'x': 1.0, 'y': 2.0, 'z': 3.0}
    assert upd('done x steps=40000 pos=12.5000') == {'x': 12.5}
    assert upd('stopped z steps=-100 pos=-0.0313') == {'z': -0.0313}
    assert upd('homed y steps=0') == {'y': 0.0}
    assert upd('done p steps=3200 pos=3200') is None      # not a gantry axis
    assert upd('ok x accepted') is None


def test_gantry_homed_update():
    from astrobiology_actuators.serial_commands import gantry_homed_update as upd
    assert upd('homed z steps=0') == {'z': True}
    assert upd('homing y') == {'y': False}
    assert upd('error homing failed x: endstop not found within max travel') == {'x': False}
    assert upd('aborted z; physical position may be lost') == {'z': False}
    assert upd('ESTOP all (!); physical position may be lost') == {'x': False, 'y': False, 'z': False}
    assert upd('aborted p; physical position may be lost') is None
    assert upd('done x steps=3200 pos=1.0000') is None
