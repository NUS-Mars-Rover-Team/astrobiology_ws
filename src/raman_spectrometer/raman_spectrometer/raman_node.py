"""ROS 2 node for the B&W Tek BTC100 Raman spectrometer.

While running it scans back to back and publishes astrobiology_msgs/RamanSpectrum on
`spectrum` after each scan (/raman/spectrum with the launch file's namespace).
Services (std_srvs/Trigger, in the same namespace):
    start        begin scanning (the node starts stopped unless autostart is true)
    stop         stop; the scan in progress is abandoned
    set_dark     use the latest raw scan as the dark, subtracted from later scans
    clear_dark   stop subtracting it
The node never switches the 532 nm laser: use /astrobiology/laser/set (lights_node).
"""

import array
import threading

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import Trigger

from astrobiology_msgs.msg import RamanSpectrum
from raman_spectrometer import btc100

# name: (default, description)
PARAMETERS = {
    'port': ('/dev/raman_spectroscope', 'Serial port of the spectrometer (CH340 RS232 adapter)'),
    'integration_ms': (100, 'Integration time per scan, 50-65000 ms'),
    'averages': (1, 'Scans the unit averages into each published spectrum'),
    'pixels': (btc100.PIXELS, 'Detector pixels'),
    'laser_nm': (532.0, 'Excitation laser wavelength for the Raman shift axis'),
    'coeffs': (list(btc100.DEFAULT_COEFFS),
               'Wavelength calibration C0..C3: lambda(p) = C0 + C1 p + C2 p^2 + C3 p^3 [nm]'),
    'frame_id': ('raman', 'frame_id of published spectra'),
    'autostart': (False, 'Start scanning when the node starts'),
}


def _validate(v):
    if not 50 <= v['integration_ms'] <= 65000:
        return 'integration_ms must be 50-65000'
    if v['averages'] < 1:
        return 'averages must be at least 1'
    if v['pixels'] < 1:
        return 'pixels must be at least 1'
    if not v['laser_nm'] > 0:
        return 'laser_nm must be positive'
    if len(v['coeffs']) != 4:
        return 'coeffs needs 4 values (C0..C3)'
    return None


class RamanNode(Node):

    def __init__(self):
        super().__init__('raman_node')
        for name, (default, description) in PARAMETERS.items():
            self.declare_parameter(name, default, ParameterDescriptor(description=description))
        values = {name: self.get_parameter(name).value for name in PARAMETERS}
        error = _validate(values)
        if error:
            raise ValueError(f'Invalid parameters: {error}')

        # Guards _values, _dark and _last_raw; _values is replaced, never mutated
        self._lock = threading.Lock()
        self._values = values
        self._dark = None        # (counts, integration_ms, averages) of the stored dark
        self._last_raw = None    # (counts, integration_ms, averages) of the latest scan
        self._reconfigure = threading.Event()   # integration/averages changed
        self._reconnect = threading.Event()     # port changed
        self._running = threading.Event()
        self._abort_scan = threading.Event()
        self._shutdown = threading.Event()
        if values['autostart']:
            self._running.set()

        # Reliable: works for both reliable and best-effort subscribers at ~1 Hz
        self._pub = self.create_publisher(RamanSpectrum, 'spectrum', QoSProfile(depth=5))
        self.create_service(Trigger, 'start', self._on_start)
        self.create_service(Trigger, 'stop', self._on_stop)
        self.create_service(Trigger, 'set_dark', self._on_set_dark)
        self.create_service(Trigger, 'clear_dark', self._on_clear_dark)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._thread = threading.Thread(target=self._acquire_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'Raman node ready on {values["port"]} '
            f'({"scanning" if values["autostart"] else "stopped; call start"})')

    def shutdown(self):
        self._shutdown.set()
        self._abort_scan.set()
        self._thread.join(timeout=3.0)

    # -- Parameters and services -----------------------------------------------

    def _on_set_parameters(self, params):
        with self._lock:
            values = dict(self._values)
        for p in params:
            if p.name in values:
                values[p.name] = p.value
        error = _validate(values)
        if error:
            return SetParametersResult(successful=False, reason=error)
        changed = {p.name for p in params}
        with self._lock:
            old = self._values
            self._values = values
            settings_changed = any(values[k] != old[k] for k in ('integration_ms', 'averages'))
            if settings_changed and self._dark is not None:
                self._dark = None
                self.get_logger().warn('integration_ms/averages changed: dark cleared; set it again')
        if settings_changed:
            self._reconfigure.set()
        if 'port' in changed and values['port'] != old['port']:
            self._reconnect.set()
        return SetParametersResult(successful=True)

    def _on_start(self, request, response):
        response.success = True
        if self._running.is_set():
            response.message = 'already scanning'
        else:
            self._running.set()
            response.message = 'scanning (switch the laser with /astrobiology/laser/set)'
        return response

    def _on_stop(self, request, response):
        self._running.clear()
        self._abort_scan.set()
        response.success = True
        response.message = 'stopped'
        return response

    def _on_set_dark(self, request, response):
        with self._lock:
            last = self._last_raw
            if last is not None:
                self._dark = last
        response.success = last is not None
        response.message = (
            f'dark set from the latest scan ({last[1]} ms x {last[2]})' if last is not None else
            'no scan yet: start, take a scan with the laser off, then set_dark')
        return response

    def _on_clear_dark(self, request, response):
        with self._lock:
            self._dark = None
        response.success = True
        response.message = 'dark cleared'
        return response

    # -- Acquisition -------------------------------------------------------------

    def _acquire_loop(self):
        dev = None
        log = self.get_logger()
        while not self._shutdown.is_set():
            if not self._running.is_set():
                self._shutdown.wait(0.2)
                continue
            with self._lock:
                v = self._values

            if self._reconnect.is_set() and dev is not None:
                dev.close()
                dev = None
            if dev is None:
                self._reconnect.clear()
                try:
                    dev = btc100.Btc100(v['port'])
                except (btc100.Btc100Error, OSError) as e:   # SerialException is an OSError
                    log.error(f'cannot open the spectrometer on {v["port"]}: {e}',
                              throttle_duration_sec=10.0)
                    self._shutdown.wait(2.0)
                    continue
                log.info(f'connected to {v["port"]}'
                         + (f' (switched it to {btc100.FAST_BAUD} baud)' if dev.switched_baud
                            else ''))
                self._reconfigure.set()

            try:
                if self._reconfigure.is_set():
                    self._reconfigure.clear()
                    dev.configure(v['integration_ms'], v['averages'])
                    log.info(f'integration {v["integration_ms"]} ms x {v["averages"]}')
                self._abort_scan.clear()
                counts = dev.scan(v['pixels'], stop=self._abort_scan)
            except btc100.Btc100Error as e:
                log.warn(f'scan skipped: {e}', throttle_duration_sec=5.0)
                if 'no reply' in str(e):   # unit unplugged or powered off: reconnect
                    dev.close()
                    dev = None
                continue
            except OSError as e:
                log.error(f'serial error: {e}; reconnecting', throttle_duration_sec=5.0)
                dev.close()
                dev = None
                continue

            if counts is None:   # stopped mid-scan: the unit finishes it; drain before the next
                self._reconfigure.set()
                continue
            self._publish(counts, v)

        if dev is not None:
            dev.close()

    def _publish(self, counts, v):
        integration, averages = v['integration_ms'], v['averages']
        with self._lock:
            self._last_raw = (counts, integration, averages)
            dark = self._dark
        subtract = (dark is not None and dark[1:] == (integration, averages)
                    and len(dark[0]) == len(counts))
        values = [c - d for c, d in zip(counts, dark[0])] if subtract else counts
        wavelengths = btc100.wavelengths(v['coeffs'], len(counts))
        raw_max = max(counts)
        n_sat = btc100.saturated(counts)

        msg = RamanSpectrum()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = v['frame_id']
        msg.raman_shift = array.array('f', btc100.raman_shift(wavelengths, v['laser_nm']))
        msg.wavelengths = array.array('f', wavelengths)
        msg.counts = array.array('f', values)
        msg.dark_subtracted = subtract
        msg.laser_nm = float(v['laser_nm'])
        msg.integration_ms = int(integration)
        msg.averages = int(averages)
        msg.raw_max = max(0, int(raw_max))
        msg.saturated_pixels = n_sat
        self._pub.publish(msg)

        if n_sat:
            self.get_logger().warn(f'{n_sat} pixels saturated (raw max {raw_max}); '
                                   'lower integration_ms or the laser power',
                                   throttle_duration_sec=5.0)
        else:
            self.get_logger().info(f'scan: raw max {raw_max}'
                                   + (', dark subtracted' if subtract else ''),
                                   throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = RamanNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
