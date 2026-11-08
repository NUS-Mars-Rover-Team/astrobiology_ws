"""ROS 2 node bridging stepper move/stop services to the ESP32 over serial.

Services use StepperMove: the caller passes steps, speed_hz, accel_hz2
which map directly to the firmware command: g1 <motor> <steps> <speed> <accel>

It also runs the microscope grid scan (microscope_scan.py) as a service:
/astrobiology/microscope/scan starts it in the background and
/astrobiology/microscope/stop ends it.  The scan shares this node's serial
port; gantry move services are rejected while it runs.
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

from astrobiology_actuators import microscope_scan as ms
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
        self.declare_parameter('scan_config', '')       # '' = installed microscope_scan.yaml
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

        # Microscope scan: the thread while one runs, and the queue the read
        # loop copies every ESP32 line into for it
        self._scan_thread = None
        self._scan_lines = None
        self._scan_stop = threading.Event()

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
        self.create_service(Command, 'astrobiology/microscope/scan',
                            self._on_scan, callback_group=cb)
        self.create_service(Trigger, 'astrobiology/microscope/stop',
                            self._on_scan_stop, callback_group=cb)

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
            if service_name in GANTRY and self._scan_thread is not None:
                resp.success = False
                resp.message = 'busy: microscope scan running'
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

    def _on_scan(self, req, resp):
        resp.success = False
        resp.expected_s = 0.0
        if self._ser is None:
            resp.message = 'no serial connection to the ESP32 (dry_run, or the port failed to open)'
            return resp

        path = self.get_parameter('scan_config').value or str(ms.default_config_path())
        try:
            cfg, points = ms.prepare_scan(path)
        except (ms.ScanConfigError, OSError, yaml.YAMLError) as e:
            resp.message = f'{path}: {e}'
            return resp
        out = (Path(self.get_parameter('scan_dir').value).expanduser()
               / f'microscope_scan_{datetime.now():%Y%m%d_%H%M%S}.csv')

        with self._lock:
            busy = [n for n in GANTRY if self._busy[n]]
            if self._scan_thread is not None or busy:
                resp.message = 'busy: ' + ('microscope scan running' if self._scan_thread
                                           else ', '.join(busy))
                return resp
            self._scan_stop.clear()
            self._scan_lines = queue.Queue()
            self._scan_thread = threading.Thread(
                target=self._run_scan, args=(cfg, points, out), daemon=True)
            self._scan_thread.start()

        resp.success = True
        resp.message = f'scanning {len(points)} points, logging to {out}'
        resp.expected_s = float(ms.estimate_scan_s(cfg, points))
        self.get_logger().info(f'microscope scan: {resp.message} '
                               f'(~{resp.expected_s:.0f} s plus homing)')
        return resp

    def _on_scan_stop(self, req, resp):
        if self._scan_thread is None:
            resp.success = False
            resp.message = 'no microscope scan running'
            return resp
        self._scan_stop.set()
        for axis in 'xyz':
            self._send_serial(stop_command(axis))
        resp.success = True
        resp.message = 'stopping microscope scan (gantry decelerates, then halts)'
        return resp

    def _run_scan(self, cfg, points, out):
        log = self.get_logger()
        gantry = None
        try:
            gantry = ms.Gantry(ser=ms.QueuedSerial(self._write_raw, self._scan_lines),
                               log=log.debug)
            out.parent.mkdir(parents=True, exist_ok=True)
            with open(out, 'w', newline='') as f:
                ms.run_scan(gantry, cfg, points, f,
                            capture=ms.camera_capture(cfg) if cfg.camera_enabled else None,
                            log=log.info, stop=self._scan_stop.is_set)
            log.info(f'microscope scan done: {len(points)} points logged to {out}')
        except ms.GantryError as e:
            if self._scan_stop.is_set():
                log.warn(f'microscope scan stopped on request; {out} has the points so far')
            else:
                log.error(f'microscope scan failed: {e}')
                if gantry is not None:
                    gantry.close()   # sends ! (estop) if a move was in progress
        except OSError as e:
            log.error(f'microscope scan failed: {e}')
        finally:
            with self._lock:
                self._scan_lines = None
                self._scan_thread = None

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

            scan_lines = self._scan_lines   # the running scan reads its replies here
            if scan_lines is not None:
                scan_lines.put(line)

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
