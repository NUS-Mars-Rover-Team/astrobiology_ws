"""ROS 2 node bridging stepper move/stop services to the ESP32 over serial.

Services use StepperMove: the caller passes steps, speed_hz, accel_hz2
which map directly to the firmware command: g1 <motor> <steps> <speed> <accel>

It also runs gantry jobs in the background, sharing this node's serial port:
/astrobiology/gantry/home homes Z, Y, X; /astrobiology/gantry/scan runs the
grid scan (gantry_scan.py); /astrobiology/gantry/stop ends either.  Gantry
move services are rejected while a job runs.
"""

from datetime import datetime
from pathlib import Path
import queue
import re
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml

from astrobiology_interfaces.srv import Command, StepperMove

from astrobiology_actuators import gantry_scan as gs
from astrobiology_actuators.serial_commands import (
    estop_command,
    move_command,
    stop_command,
)

# service_name → (firmware_cmd_name, firmware_feedback_name)
MOTORS = {
    'pump':       ('p', 'pump'),
    'flush':      ('f', 'flush'),
    'centrifuge': ('c', 'centrifuge'),
    'x_gantry':   ('x', 'x'),
    'y_gantry':   ('y', 'y'),
    'z_gantry':   ('z', 'z'),
}
GANTRY = ('x_gantry', 'y_gantry', 'z_gantry')

# Firmware feedback patterns
_DONE_RE = re.compile(r'^done\s+(\S+)\s+steps=(-?\d+)')
_STOPPED_RE = re.compile(r'^stopped\s+(\S+)\s+steps=(-?\d+)')
_CANCEL_RE = re.compile(r'^cancelled\s+(\S+)')
_ABORT_RE = re.compile(r'^aborted\s+(\S+)')
_OK_RE = re.compile(r'^ok\s+(\S+)\s+accepted')
_ERR_BUSY_RE = re.compile(r'^error\s+busy:\s*(\S+)')


class SteppersNode(Node):

    def __init__(self):
        super().__init__('steppers_node')

        # Parameters
        self.declare_parameter('port', '/dev/esp_steppers')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('scan_config', '')       # '' = installed gantry_scan.yaml
        self.declare_parameter('scan_dir', '~/scans')   # where scan CSVs are written

        self._dry_run = self.get_parameter('dry_run').value
        self._busy = {name: False for name in MOTORS}
        # Reverse lookup: firmware feedback name → service name.  The firmware
        # replies with the short id (done f ...); older builds used the full name.
        self._feedback_to_service = {}
        for svc, (cmd, fb) in MOTORS.items():
            self._feedback_to_service[cmd] = svc
            self._feedback_to_service[fb] = svc

        # Serial port
        self._lock = threading.Lock()
        self._ser = None
        self._connected = False

        # Gantry job: 'homing' or 'scan' while one runs, and the queue the
        # read loop copies every ESP32 line into for it
        self._gantry_job = None
        self._gantry_lines = None
        self._gantry_stop = threading.Event()

        if not self._dry_run:
            self._open_serial()

        # Services — one move + stop pair per motor
        cb = ReentrantCallbackGroup()
        for name in MOTORS:
            self.create_service(
                StepperMove, f'astrobiology/{name}/move',
                lambda req, resp, n=name: self._on_move(req, resp, n),
                callback_group=cb,
            )
            self.create_service(
                Trigger, f'astrobiology/{name}/stop',
                lambda req, resp, n=name: self._on_stop(req, resp, n),
                callback_group=cb,
            )
        self.create_service(Command, 'astrobiology/gantry/home',
                            self._on_home, callback_group=cb)
        self.create_service(Command, 'astrobiology/gantry/scan',
                            self._on_scan, callback_group=cb)
        self.create_service(Trigger, 'astrobiology/gantry/stop',
                            self._on_gantry_stop, callback_group=cb)

        # Background serial read thread
        if self._ser:
            threading.Thread(target=self._read_loop, daemon=True).start()

        mode = 'dry_run' if self._dry_run else 'live'
        self.get_logger().info(
            f'Stepper node started ({mode}), motors: {list(MOTORS.keys())}')

    def _open_serial(self):
        try:
            import serial
        except ImportError:
            self.get_logger().error('pyserial not installed; falling back to dry_run')
            self._dry_run = True
            return

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        try:
            self._ser = serial.Serial(port, baud, timeout=0.5)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self._ser.write(b'\n')
            self._connected = True
            self.get_logger().info(f'Connected to {port}')
        except serial.SerialException as e:
            self._ser = None
            self.get_logger().error(f'Cannot open {port}: {e}')

    # -- Service callbacks ---------------------------------------------------

    def _on_move(self, req, resp, service_name):
        with self._lock:
            if service_name in GANTRY and self._gantry_job is not None:
                resp.success = False
                resp.message = f'busy: gantry {self._gantry_job} in progress'
                return resp
            if self._busy[service_name]:
                resp.success = False
                resp.message = 'busy'
                return resp
            self._busy[service_name] = True

        cmd_name = MOTORS[service_name][0]
        cmd = move_command(cmd_name, req.steps, req.speed_hz, req.accel_hz2)
        self._send_serial(cmd)
        self.get_logger().info(cmd)

        resp.success = True
        resp.message = cmd
        return resp

    def _on_stop(self, req, resp, service_name):
        cmd_name = MOTORS[service_name][0]
        self._send_serial(stop_command(cmd_name))
        with self._lock:
            self._busy[service_name] = False
        self.get_logger().info(f'stop {cmd_name}')
        resp.success = True
        resp.message = f'stop {cmd_name}'
        return resp

    def _on_home(self, req, resp):
        resp.success = False
        resp.expected_s = 0.0
        if not self._start_gantry_job('homing', self._home, (), resp):
            return resp
        try:   # the scan config's travel limits give an upper bound
            cfg = gs.ScanConfig.from_yaml(self._scan_config_path())
            resp.expected_s = float(gs.estimate_home_s(cfg))
        except (gs.ScanConfigError, OSError, yaml.YAMLError):
            pass
        resp.success = True
        resp.message = 'homing Z, then Y, then X'
        self.get_logger().info(f'gantry homing: Z, then Y, then X (up to ~{resp.expected_s:.0f} s)')
        return resp

    def _on_scan(self, req, resp):
        resp.success = False
        resp.expected_s = 0.0
        path = self._scan_config_path()
        try:
            cfg, points = gs.prepare_scan(path)
        except (gs.ScanConfigError, OSError, yaml.YAMLError) as e:
            resp.message = f'{path}: {e}'
            return resp
        out = (Path(self.get_parameter('scan_dir').value).expanduser()
               / f'gantry_scan_{datetime.now():%Y%m%d_%H%M%S}.csv')
        if not self._start_gantry_job('scan', self._scan, (cfg, points, out), resp):
            return resp

        resp.success = True
        resp.message = f'scanning {len(points)} points, logging to {out}'
        resp.expected_s = float(gs.estimate_scan_s(cfg, points))
        self.get_logger().info(f'gantry scan: {resp.message} '
                               f'(~{resp.expected_s:.0f} s plus homing)')
        return resp

    def _on_gantry_stop(self, req, resp):
        job = self._gantry_job
        if job is None:
            resp.success = False
            resp.message = 'no gantry scan or homing in progress'
            return resp
        self._gantry_stop.set()
        for axis in 'xyz':
            self._send_serial(stop_command(axis))
        resp.success = True
        resp.message = f'stopping gantry {job} (gantry decelerates, then halts)'
        return resp

    # -- Gantry jobs (homing, scan) ------------------------------------------

    def _scan_config_path(self):
        return self.get_parameter('scan_config').value or str(gs.default_config_path())

    def _start_gantry_job(self, job, target, args, resp):
        """Run target(gantry, *args) in the background unless the gantry is busy.

        On refusal fills resp.message and returns False.
        """
        if self._ser is None:
            resp.message = 'no serial connection to the ESP32 (dry_run, or the port failed to open)'
            return False
        with self._lock:
            busy = [n for n in GANTRY if self._busy[n]]
            if self._gantry_job is not None or busy:
                resp.message = 'busy: ' + (f'gantry {self._gantry_job} in progress'
                                           if self._gantry_job else ', '.join(busy))
                return False
            self._gantry_job = job
            self._gantry_stop.clear()
            self._gantry_lines = queue.Queue()
            threading.Thread(target=self._run_gantry_job, args=(job, target, args),
                             daemon=True).start()
        return True

    def _run_gantry_job(self, job, target, args):
        log = self.get_logger()
        gantry = None
        try:
            gantry = gs.Gantry(ser=gs.QueuedSerial(self._write_raw, self._gantry_lines),
                               log=log.debug)
            target(gantry, *args)
        except gs.GantryError as e:
            if self._gantry_stop.is_set():
                log.warn(f'gantry {job} stopped on request'
                         + ('; position not set, home again' if job == 'homing' else ''))
            else:
                log.error(f'gantry {job} failed: {e}')
                if gantry is not None:
                    gantry.close()   # sends ! (estop) if a move was in progress
        except OSError as e:
            log.error(f'gantry {job} failed: {e}')
        finally:
            with self._lock:
                self._gantry_lines = None
                self._gantry_job = None

    def _home(self, gantry):
        gantry.home()
        # A stop cancels homing, but the firmware still ends with 'done g28'
        if self._gantry_stop.is_set():
            raise gs.GantryError('homing stopped')
        self.get_logger().info('gantry homed: X/Y/Z = 0')

    def _scan(self, gantry, cfg, points, out):
        log = self.get_logger()
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, 'w', newline='') as f:
            gs.run_scan(gantry, cfg, points, f,
                        capture=gs.camera_capture(cfg) if cfg.camera_enabled else None,
                        log=log.info, stop=self._gantry_stop.is_set)
        log.info(f'gantry scan done: {len(points)} points logged to {out}')

    # -- Serial I/O ----------------------------------------------------------

    def _write_raw(self, data):
        if self._ser:
            with self._lock:
                self._ser.write(data)

    def _send_serial(self, cmd):
        if self._dry_run:
            self.get_logger().info(f'[dry_run] {cmd}')
            return
        self._write_raw((cmd + '\n').encode())

    def _read_loop(self):
        import serial as pyserial

        while rclpy.ok():
            try:
                line = self._ser.readline().decode(errors='replace').strip()
            except pyserial.SerialException as e:
                self.get_logger().error(f'serial read failed: {e}')
                self._connected = False
                return
            if not line:
                continue

            gantry_lines = self._gantry_lines   # a running gantry job reads its replies here
            if gantry_lines is not None:
                gantry_lines.put(line)

            m = _DONE_RE.match(line)
            if m:
                self._clear_busy(m.group(1))
                self.get_logger().info(f'done {m.group(1)} steps={m.group(2)}')
                continue

            m = _STOPPED_RE.match(line)
            if m:
                self._clear_busy(m.group(1))
                self.get_logger().info(f'stopped {m.group(1)} steps={m.group(2)}')
                continue

            m = _CANCEL_RE.match(line)
            if m:
                self._clear_busy(m.group(1))
                self.get_logger().info(f'cancelled {m.group(1)}')
                continue

            m = _ABORT_RE.match(line)
            if m:
                self._clear_busy(m.group(1))
                self.get_logger().warn(
                    f'aborted {m.group(1)}; physical position may be lost')
                continue

            m = _OK_RE.match(line)
            if m:
                self.get_logger().debug(f'{m.group(1)} accepted')
                continue

            m = _ERR_BUSY_RE.match(line)
            if m:
                self.get_logger().error(f'firmware busy: {m.group(1)}')
                continue

            if line.startswith('error'):
                self.get_logger().error(f'esp: {line}')
            else:
                self.get_logger().info(f'esp: {line}')

    def _clear_busy(self, firmware_name):
        # Firmware feedback uses short ids (p, f, c, x, y, z) or, on older builds,
        # full names; _busy keys are service names (pump, ..., x_gantry, ...)
        svc = self._feedback_to_service.get(firmware_name)
        if svc is None:
            return
        with self._lock:
            self._busy[svc] = False


def main(args=None):
    rclpy.init(args=args)
    node = SteppersNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node._ser:
            node._ser.write(b'estop\n')
            node._ser.close()
        node.destroy_node()
        rclpy.shutdown()
