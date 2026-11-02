import pytest

from astrobiology_actuators.microscope_scan import Gantry, GantryError


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
