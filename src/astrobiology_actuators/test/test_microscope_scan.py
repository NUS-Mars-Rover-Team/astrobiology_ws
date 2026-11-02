import pytest

from astrobiology_actuators import microscope_scan

from astrobiology_actuators.microscope_scan import Gantry, GantryError, build_parser


class FakeSerial:
    """Scripted ESP32: each command line written queues its canned reply lines."""

    def __init__(self, replies=None):
        self.replies = replies or {}
        self.written = []
        self._rx = []
        self.closed = False

    def write(self, data):
        self.written.append(data)
        cmd = data.decode().strip()
        self._rx.extend(self.replies.get(cmd, []))

    def readline(self):
        return (self._rx.pop(0) + '\r\n').encode() if self._rx else b''

    def close(self):
        self.closed = True


def make_gantry(replies=None):
    ser = FakeSerial(replies)
    return Gantry(ser=ser, log=lambda msg: None), ser


def test_pos_returns_xyz():
    g, ser = make_gantry({
        'pos': ['pos Z:40.0000 Y:24.5000 X:25.5000 C:0.0000 P:0 F:0']})
    assert g.pos() == {'x': 25.5, 'y': 24.5, 'z': 40.0}
    assert ser.written[-1] == b'pos\n'


def test_startup_drains_reply_to_flush_newline():
    # Junk from opening the port makes the ESP32 reject the flush newline
    g, _ = make_gantry({
        '': ['error line too long or invalid; discarded'],
        'pos': ['pos Z:0.0000 Y:0.0000 X:2.0000']})
    assert g.pos()['x'] == 2.0


def test_pos_skips_unrelated_lines():
    g, _ = make_gantry({
        'pos': ['done x steps=3200 pos=1.0000', 'pos Z:0.0000 Y:0.0000 X:1.0000']})
    assert g.pos()['x'] == 1.0


def test_pos_raises_on_firmware_error():
    g, _ = make_gantry({'pos': ['error unknown command; type help']})
    with pytest.raises(GantryError, match='unknown command'):
        g.pos()


def test_pos_times_out():
    g, _ = make_gantry()
    with pytest.raises(GantryError, match='no reply'):
        g.pos(timeout=0.05)


def test_close_sends_estop_only_while_moving():
    g, ser = make_gantry()
    g.close()
    assert b'!' not in ser.written and ser.closed

    g, ser = make_gantry()
    g._moving = True
    g.close()
    assert ser.written[-1] == b'!'


HOME_ALL_REPLY = [
    'ok g28 accepted: z y x',
    'homing z', 'homed z steps=0',
    'homing y', 'homed y steps=0',
    'homing x', 'homed x steps=0',
    'done g28',
]


def test_home_waits_for_done_g28():
    g, ser = make_gantry({'g28': HOME_ALL_REPLY})
    g.home()
    assert ser.written[-1] == b'g28\n'
    g.close()
    assert b'!' not in ser.written


def test_home_selected_axes():
    g, ser = make_gantry({'g28 x': ['ok g28 accepted: x', 'homing x',
                                    'homed x steps=0', 'done g28']})
    g.home(('x',))
    assert ser.written[-1] == b'g28 x\n'


def test_home_failure_raises_and_estops_on_close():
    g, ser = make_gantry({'g28': [
        'ok g28 accepted: z y x', 'homing z',
        'error homing failed z: endstop not found within max travel']})
    with pytest.raises(GantryError, match='endstop not found'):
        g.home()
    g.close()
    assert ser.written[-1] == b'!'


def test_cli_home_without_axes_parses():
    assert build_parser().parse_args(['home']).axes == []


def test_cli_home_axes():
    assert build_parser().parse_args(['home', 'z', 'x']).axes == ['z', 'x']


def test_cli_home_rejects_bad_axis():
    with pytest.raises(SystemExit):
        build_parser().parse_args(['home', 'c'])


POS_ORIGIN = ['pos Z:0.0000 Y:0.0000 X:0.0000 C:0.0000 P:0 F:0']


def g1_lines(ser):
    return [w.decode().strip() for w in ser.written if w.startswith(b'g1')]


def test_move_to_sends_relative_mm_and_waits_for_done():
    g, ser = make_gantry({
        'pos': POS_ORIGIN,
        'g1 x 1 5 10': ['ok x accepted', 'done x steps=3200 pos=1.0000']})
    g.move_to({'x': 1.0}, speed=5, accel=10)
    assert g1_lines(ser) == ['g1 x 1 5 10']
    g.close()
    assert b'!' not in ser.written


def test_move_to_is_relative_to_current_position():
    g, ser = make_gantry({
        'pos': ['pos Z:0.0000 Y:0.0000 X:2.0000'],
        'g1 x -1 5 10': ['ok x accepted', 'done x steps=3200 pos=1.0000']})
    g.move_to({'x': 1.0})
    assert g1_lines(ser) == ['g1 x -1 5 10']


def test_move_to_waits_for_every_axis():
    g, ser = make_gantry({
        'pos': POS_ORIGIN,
        'g1 x 1 5 10': ['ok x accepted'],
        'g1 y 2 5 10': ['ok y accepted', 'done y steps=6400 pos=2.0000',
                        'done x steps=3200 pos=1.0000']})
    g.move_to({'x': 1.0, 'y': 2.0})
    assert g1_lines(ser) == ['g1 x 1 5 10', 'g1 y 2 5 10']


def test_move_to_skips_axes_already_there():
    g, ser = make_gantry({'pos': ['pos Z:3.0000 Y:0.0000 X:1.0001']})
    g.move_to({'x': 1.0, 'z': 3.0})
    assert g1_lines(ser) == []


def test_move_to_times_out_and_estops(monkeypatch):
    monkeypatch.setattr(microscope_scan, 'MOVE_MARGIN_S', 0.0)
    g, ser = make_gantry({
        'pos': POS_ORIGIN,
        'g1 x 0.01 30 300': ['ok x accepted']})     # never reports done
    with pytest.raises(GantryError, match='no reply'):
        g.move_to({'x': 0.01}, speed=30, accel=300)
    g.close()
    assert ser.written[-1] == b'!'


def test_move_to_raises_on_busy():
    g, ser = make_gantry({'pos': POS_ORIGIN, 'g1 x 1 5 10': ['error busy: x']})
    with pytest.raises(GantryError, match='busy'):
        g.move_to({'x': 1.0})


def test_move_to_raises_on_stopped():
    g, _ = make_gantry({'pos': POS_ORIGIN,
                        'g1 x 1 5 10': ['ok x accepted', 'stopped x steps=100 pos=0.0313']})
    with pytest.raises(GantryError, match='stopped'):
        g.move_to({'x': 1.0})


@pytest.mark.parametrize('target, speed, accel', [
    ({'x': -0.5}, 5, 10),       # behind the endstop
    ({'x': 1.0}, 40, 10),       # over 31.25 mm/s
    ({'x': 1.0}, 5, 0),
])
def test_move_to_rejects_before_sending(target, speed, accel):
    g, ser = make_gantry({'pos': POS_ORIGIN})
    with pytest.raises(GantryError):
        g.move_to(target, speed, accel)
    assert ser.written == [b'\n']


def test_cli_goto_requires_an_axis(capsys):
    with pytest.raises(SystemExit):
        microscope_scan.main(['goto'])
    assert 'at least one' in capsys.readouterr().err


def test_cli_goto_parses_axes():
    args = build_parser().parse_args(['goto', '--x', '1', '--z', '2.5', '--speed', '2'])
    assert (args.x, args.y, args.z, args.speed, args.accel) == (1.0, None, 2.5, 2.0, 10.0)
