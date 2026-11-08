import threading

import pytest

from raman_spectrometer import btc100
from raman_spectrometer.btc100 import (
    FAST_BAUD,
    POWER_ON_BAUD,
    Btc100,
    Btc100Error,
    parse_values,
    raman_shift,
    saturated,
    wavelengths,
)


class FakeUnit:
    """Scripted BTC100: echoes and ACKs commands, but only at the baud it's listening on."""

    def __init__(self, unit_baud=FAST_BAUD, counts=None, silent=False):
        self.unit_baud = unit_baud
        self.baudrate = FAST_BAUD
        self.counts = counts if counts is not None else [1500 + i % 7 for i in range(2048)]
        self.silent = silent
        self.commands = []
        self.closed = False
        self._rx = bytearray()
        self._partial = b''

    # host -> unit
    def write(self, data):
        if self.silent or self.baudrate != self.unit_baud:
            return
        self._partial += data
        while b'\r' in self._partial:
            cmd, self._partial = self._partial.split(b'\r', 1)
            self._handle(cmd.decode())

    def _handle(self, cmd):
        self.commands.append(cmd)
        if cmd == '':
            return
        if cmd == 'S':
            body = '\r\n'.join(f'{c:05d}' for c in self.counts)
            self._rx += f'S\r\nACK\r\n{body}\r\n'.encode() + b'\x00'
            return
        self._rx += f'{cmd}\r\nACK\r\n'.encode()
        if cmd == 'K0':
            self.unit_baud = FAST_BAUD

    # unit -> host
    @property
    def in_waiting(self):
        return len(self._rx)

    def read(self, n=1):
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def reset_input_buffer(self):
        self._rx.clear()

    def flush(self):
        pass

    def close(self):
        self.closed = True


@pytest.fixture(autouse=True)
def quick(monkeypatch):
    monkeypatch.setattr(btc100, 'CMD_GAP_S', 0.0)
    monkeypatch.setattr(btc100, 'DRAIN_QUIET_S', 0.01)
    monkeypatch.setattr(btc100, 'ACK_TIMEOUT_S', 0.05)
    monkeypatch.setattr(btc100.time, 'sleep', lambda s: None)


def test_parse_values_skips_ack_and_echo():
    assert parse_values(b'S\r\nACK\r\n01505\r\n00007\r\n\x00') == [1505, 7]


def test_connect_at_fast_baud_does_not_switch():
    unit = FakeUnit(unit_baud=FAST_BAUD)
    dev = Btc100('fake', ser=unit)
    assert not dev.switched_baud
    assert 'K0' not in unit.commands


def test_connect_switches_a_power_on_unit_to_fast_baud():
    unit = FakeUnit(unit_baud=POWER_ON_BAUD)
    dev = Btc100('fake', ser=unit)
    assert dev.switched_baud
    assert 'K0' in unit.commands
    assert unit.baudrate == FAST_BAUD


def test_connect_without_reply_raises_and_closes():
    unit = FakeUnit(silent=True)
    with pytest.raises(Btc100Error, match='no reply'):
        Btc100('fake', ser=unit)
    assert unit.closed


def test_configure_sends_ascii_integration_and_averages():
    unit = FakeUnit()
    dev = Btc100('fake', ser=unit)
    unit.commands.clear()
    dev.configure(250, 3)
    assert unit.commands == ['', 'a', 'I250', 'A3']


@pytest.mark.parametrize('ms, n', [(49, 1), (65001, 1), (100, 0)])
def test_configure_rejects_out_of_range(ms, n):
    dev = Btc100('fake', ser=FakeUnit())
    with pytest.raises(ValueError):
        dev.configure(ms, n)


def test_scan_returns_the_pixel_counts():
    counts = list(range(2048))
    dev = Btc100('fake', ser=FakeUnit(counts=counts))
    dev.configure(100, 1)
    assert dev.scan() == counts


def test_short_scan_raises():
    dev = Btc100('fake', ser=FakeUnit(counts=[1, 2, 3]))
    dev.configure(100, 1)
    with pytest.raises(Btc100Error, match='got 3 numbers'):
        dev.scan(pixels=2048)


def test_stopped_scan_returns_none():
    dev = Btc100('fake', ser=FakeUnit())
    stop = threading.Event()
    stop.set()
    assert dev.scan(stop=stop) is None


def test_wavelength_calibration():
    nm = wavelengths()
    assert len(nm) == 2048
    assert nm[0] == pytest.approx(355.3)
    assert 620 < nm[-1] < 630


def test_raman_shift_at_532_nm():
    assert raman_shift([532.0], 532.0) == [pytest.approx(0.0, abs=1e-9)]
    assert raman_shift([550.0], 532.0)[0] == pytest.approx(615.2, abs=0.1)
    assert raman_shift([500.0], 532.0)[0] < 0   # anti-Stokes side


def test_saturated():
    assert saturated([0, 64999, 65000, 65535]) == 2
