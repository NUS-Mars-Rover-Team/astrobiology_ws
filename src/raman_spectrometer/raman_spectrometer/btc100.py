"""B&W Tek BTC100-2S Raman spectrometer over RS232.  No ROS imports.

Ported from the bench script ~/Documents/NUSMRT/btc100_test/btc100_capture.py.
Protocol (Syonyk's blog / Spectrum Studio help file):
  8N1, no flow control; 9600 baud at power-on.  CR-terminated ASCII commands:
    I{ms}  integration time (50-65000)     A{n}  number of scans to average
    a      ASCII output mode               S     trigger a scan and return data
    ?X     query a setting                 K0    switch to 115200 baud until power-off
  Commands are echoed and ACKed.  A scan replies 'S\\r\\nACK\\r\\n', stays silent for the
  integration time x averages, then sends one number per pixel and a trailing NUL byte.
  Abandoning a scan doesn't stop the unit: it sends the rest and ignores commands until
  done, so drain() before the next command.

Wavelength calibration: lambda(p) = C0 + C1*p + C2*p^2 + C3*p^3 [nm], as in Spectrum Studio.
"""

import re
import time

POWER_ON_BAUD = 9600
FAST_BAUD = 115200
PIXELS = 2048
SATURATION = 65000          # counts at or above this are clipped

# From a CFL viewed through white paper, 2026-10-22: Hg 404.66/435.83/546.07/576.96/579.07 nm
# and Eu 611.6 nm, fit RMS ~0.04 nm.  Covers ~355-627 nm.  Recalibrate if the input optics change.
DEFAULT_COEFFS = (355.3, 0.1864838, -2.323853e-05, -1.515545e-09)

EOL = '\r'
CMD_GAP_S = 0.1             # pause after each command
DRAIN_QUIET_S = 0.3         # the line counts as idle after this long without a byte
ACK_TIMEOUT_S = 0.5         # how long responds() waits for an ACK
NO_REPLY_HINT = 'Check power (5 V only), the cable and the port.'


class Btc100Error(RuntimeError):
    pass


def parse_values(raw):
    """Every integer in a raw reply (pixel values; ACK/echo lines have none)."""
    return [int(v) for v in re.findall(rb'-?\d+', raw)]


def wavelengths(coeffs=DEFAULT_COEFFS, pixels=PIXELS):
    """Wavelength in nm of each pixel."""
    c0, c1, c2, c3 = coeffs
    return [c0 + c1 * p + c2 * p ** 2 + c3 * p ** 3 for p in range(pixels)]


def raman_shift(wavelengths_nm, laser_nm):
    """Raman shift in cm^-1 of each wavelength for the given excitation (negative below it)."""
    return [1e7 / laser_nm - 1e7 / w for w in wavelengths_nm]


def saturated(counts):
    """Number of clipped pixels."""
    return sum(1 for c in counts if c >= SATURATION)


class Btc100:
    """Serial link to the spectrometer; switches it to 115200 baud on connect if needed.

    Pass ``ser`` (anything with write/read/in_waiting/reset_input_buffer/flush/close
    and a baudrate attribute) to skip opening a real port, e.g. in tests.
    """

    def __init__(self, port, ser=None):
        if ser is None:
            import serial
            ser = serial.Serial(port, FAST_BAUD, bytesize=8, parity='N', stopbits=1,
                                timeout=0.05, xonxoff=False, rtscts=False)
        self._ser = ser
        self.switched_baud = False   # True if connect had to send K0
        self._scan_time_s = 0.1
        try:
            self._connect()
        except Exception:
            self._ser.close()
            raise

    def _send(self, cmd):
        self._ser.write((cmd + EOL).encode('ascii'))
        self._ser.flush()
        time.sleep(CMD_GAP_S)

    def _read(self):
        return self._ser.read(self._ser.in_waiting or 1)

    def drain(self):
        """Wait until the line is quiet (e.g. the tail of an abandoned scan)."""
        limit = time.monotonic() + 15000 * 10 / self._ser.baudrate + 1   # one full ASCII scan
        last_rx = time.monotonic()
        while time.monotonic() < limit and time.monotonic() - last_rx < DRAIN_QUIET_S:
            if self._read():
                last_rx = time.monotonic()

    def _responds(self):
        """True if the unit ACKs a query at the port's current baud rate."""
        self.drain()
        self._send('')   # clear any partial command
        self._ser.reset_input_buffer()
        self._send('?K')
        reply = bytearray()
        deadline = time.monotonic() + ACK_TIMEOUT_S
        while b'ACK' not in reply and time.monotonic() < deadline:
            reply += self._read()
        return b'ACK' in reply

    def _connect(self):
        if self._responds():
            return
        self._ser.baudrate = POWER_ON_BAUD
        if not self._responds():
            raise Btc100Error(f'no reply at {FAST_BAUD} or {POWER_ON_BAUD} baud. {NO_REPLY_HINT}')
        self._send('K0')
        time.sleep(0.3)   # let the ACK out at the old rate before switching
        self._ser.baudrate = FAST_BAUD
        if not self._responds():
            raise Btc100Error(f'the unit answered at {POWER_ON_BAUD} baud but not at '
                              f'{FAST_BAUD} after K0')
        self.switched_baud = True

    def configure(self, integration_ms, averages):
        """ASCII output, integration time (50-65000 ms) and number of scans to average."""
        if not 50 <= integration_ms <= 65000:
            raise ValueError('integration time must be 50-65000 ms')
        if averages < 1:
            raise ValueError('averages must be at least 1')
        self.drain()
        # A bare terminator first: leftover bytes otherwise prefix the next command,
        # which the unit then echoes without ACK and ignores.
        for cmd in ('', 'a', f'I{integration_ms}', f'A{averages}'):
            self._send(cmd)
        time.sleep(0.2)
        self._ser.reset_input_buffer()   # drop the echoes and ACKs
        self._scan_time_s = integration_ms * averages / 1000.0

    def _read_scan(self, expected, stop=None):
        """Read a scan reply.

        The unit ACKs at once, then stays silent for the whole integration time, so the
        idle timer only starts once digits arrive.  Once `expected` complete numbers are
        in, the reply ends at the trailing NUL (or after a short idle if none comes).
        """
        buf = bytearray()
        deadline = time.monotonic() + self._scan_time_s + 5
        last_rx = None
        got_data = got_all = False
        while stop is None or not stop.is_set():
            chunk = self._read()
            now = time.monotonic()
            if chunk:
                buf += chunk
                last_rx = now
                if not got_data and any(48 <= b <= 57 for b in chunk):
                    got_data = True
                if not got_all:
                    # Only count numbers followed by a separator; the last may be partial
                    got_all = len(re.findall(rb'\d+(?=\D)', buf)) >= expected
                if got_all and b'\x00' in chunk:
                    break
            elif not got_data and now > deadline:
                break
            elif got_data and now - last_rx > (0.2 if got_all else 1.5):
                break
        return bytes(buf)

    def scan(self, pixels=PIXELS, stop=None):
        """Trigger one scan and return its `pixels` counts; None if `stop` was set first."""
        self._ser.reset_input_buffer()
        self._send('S')
        raw = self._read_scan(pixels, stop)
        if stop is not None and stop.is_set():
            return None
        values = parse_values(raw)
        if not values:
            raise Btc100Error(f'no reply to the scan. {NO_REPLY_HINT}')
        if len(values) < pixels:
            raise Btc100Error(f'got {len(values)} numbers, expected {pixels}')
        return values[-pixels:]   # leading numbers would be echo/header

    def close(self):
        self._ser.close()
