"""ROS 2 node bridging stepper move/stop services to the ESP32 over serial.

Pump, flush and centrifuge use StepperMove: the caller passes steps, speed_hz,
accel_hz2, which map directly to the firmware command: g1 <motor> <steps> <speed> <accel>

The two peristaltic pumps, pump (p, extraction) and flush (f, dosing), also take a
volume: /astrobiology/<pump|flush>/dispense (Dispense) pumps volume_ml using that
pump's pulses_per_ml parameter and stops by itself; <pump|flush>/stop ends it early.

gantry/{x,y,z}/move use MoveAxis in mm: relative (jog by position_mm) or absolute
(go to position_mm).  The axis must be homed and the target within 0..travel_max
(gantry_scan.yaml); the move runs at jog_speed_mm_s and the service replies at once.

It also runs gantry jobs in the background, sharing this node's serial port:
/astrobiology/gantry/home homes Z, Y, X; /astrobiology/gantry/scan runs the
grid scan (gantry_scan.py); /astrobiology/gantry/handover runs the cache
handover (home, X out, Y out, home Y, home X); /astrobiology/gantry/stop ends
any of them.  Gantry move services are rejected while a job runs.

/astrobiology/gantry/position (geometry_msgs/Point, mm) publishes the gantry
X/Y/Z from every position the ESP32 reports; the node asks for it with 'pos'
at position_rate_hz while no gantry job is running (a job reports its own).
"""

from datetime import datetime
from pathlib import Path
import queue
import re
import threading
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml

from astrobiology_interfaces.srv import Command, Dispense, MoveAxis, StepperMove

from astrobiology_actuators import gantry_scan as gs
from astrobiology_actuators.pump import PumpConfig, PumpState
from astrobiology_actuators.serial_commands import (
    estop_command,
    gantry_homed_update,
    gantry_position_update,
    move_command,
    move_mm_command,
    pos_command,
    stop_command,
)

# service_name → (firmware_cmd_name, firmware_feedback_name)
MOTORS = {
    'pump':       ('p', 'pump'),
    'flush':      ('f', 'flush'),
    'centrifuge': ('c', 'centrifuge'),
    'gantry/x':   ('x', 'x'),
    'gantry/y':   ('y', 'y'),
    'gantry/z':   ('z', 'z'),
}
GANTRY = ('gantry/x', 'gantry/y', 'gantry/z')

# Dispensing pumps: MOTORS key -> role on the rover, and their parameters
PUMPS = {'pump': 'extraction', 'flush': 'dosing'}
PUMP_DEFAULTS = {
    'pulses_per_ml': 1000.0,   # placeholder until calibrated
    'speed_hz': 800,
    'accel_hz2': 400,
    'direction': 'cw',         # 'ccw' reverses the pump
    'max_ml': 50.0,
}

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
        self.declare_parameter('position_rate_hz', 2.0)   # 0 = don't poll 'pos'
        self.declare_parameter('handover_x_mm', 53.0)     # cache handover position
        self.declare_parameter('handover_y_mm', 105.0)
        self.declare_parameter('jog_speed_mm_s', 5.0)     # gantry/{x,y,z}/move
        self.declare_parameter('jog_accel_mm_s2', 10.0)
        # MOTORS key -> (label for messages, PumpState) for the dispensing pumps
        self._pumps = {motor: (f'{motor} ({role})', PumpState(self._pump_config(i, motor)))
                       for i, (motor, role) in enumerate(PUMPS.items(), start=1)}

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

        # Last gantry X/Y/Z (mm) the ESP32 reported; published once all three are known
        self._position = {}
        # Gantry axes homed since this node started (position means something)
        self._homed = set()
        self._position_pub = self.create_publisher(Point, 'astrobiology/gantry/position', 10)

        if not self._dry_run:
            self._open_serial()

        # Services — one move + stop pair per motor
        cb = ReentrantCallbackGroup()
        for name in MOTORS:
            if name in GANTRY:
                self.create_service(
                    MoveAxis, f'astrobiology/{name}/move',
                    lambda req, resp, n=name: self._on_gantry_move(req, resp, n),
                    callback_group=cb,
                )
            else:
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
        for motor in PUMPS:   # their stop is the <motor>/stop service above
            self.create_service(
                Dispense, f'astrobiology/{motor}/dispense',
                lambda req, resp, m=motor: self._on_dispense(req, resp, m),
                callback_group=cb,
            )
        self.create_timer(1.0, self._tick_pumps)
        self.create_service(Command, 'astrobiology/gantry/home',
                            self._on_home, callback_group=cb)
        self.create_service(Command, 'astrobiology/gantry/scan',
                            self._on_scan, callback_group=cb)
        self.create_service(Command, 'astrobiology/gantry/handover',
                            self._on_handover, callback_group=cb)
        self.create_service(Trigger, 'astrobiology/gantry/stop',
                            self._on_gantry_stop, callback_group=cb)

        # Background serial read thread
        if self._ser:
            threading.Thread(target=self._read_loop, daemon=True).start()
            rate = self.get_parameter('position_rate_hz').value
            if rate > 0:
                self.create_timer(1.0 / rate, self._poll_position)

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

    def _on_gantry_move(self, req, resp, service_name):
        axis = MOTORS[service_name][0]
        name = axis.upper()
        resp.success = False
        resp.expected_s = 0.0
        if self._ser is None:
            resp.message = 'no serial connection to the ESP32 (dry_run, or the port failed to open)'
            return resp
        job = self._gantry_job   # checked again under the lock below
        if job is not None:
            resp.message = f'busy: gantry {job} in progress'
            return resp
        if axis not in self._homed:
            resp.message = f'{name} is not homed; call /astrobiology/gantry/home first'
            return resp
        current = self._position.get(axis)
        if current is None:
            resp.message = f'no {name} position from the ESP32 yet'
            return resp
        try:
            limit = gs.ScanConfig.from_yaml(self._scan_config_path()).travel_max[axis]
        except (gs.ScanConfigError, OSError, yaml.YAMLError) as e:
            resp.message = f'cannot read travel limits: {e}'
            return resp
        offset = float(req.position_mm)
        target = round(current + offset if req.relative else offset, 4)
        if not 0.0 <= target <= limit:
            resp.message = f'{name} {target:g} mm is outside 0..{limit:g} mm'
            return resp

        delta = target - current
        speed = float(self.get_parameter('jog_speed_mm_s').value)
        accel = float(self.get_parameter('jog_accel_mm_s2').value)
        with self._lock:
            if self._gantry_job is not None:
                resp.message = f'busy: gantry {self._gantry_job} in progress'
                return resp
            if self._busy[service_name]:
                resp.message = f'busy: {name} is still moving'
                return resp
            if abs(delta) < gs.HALF_STEP_MM:
                resp.success = True
                resp.message = f'{name} already at {target:g} mm'
                return resp
            self._busy[service_name] = True   # cleared by the ESP32's done/stopped line

        self._send_serial(move_mm_command(axis, delta, speed, accel))
        resp.success = True
        resp.message = f'{name} {current:g} -> {target:g} mm'
        resp.expected_s = abs(delta) / speed + speed / accel
        self.get_logger().info(f'{service_name}: {resp.message}')
        return resp

    def _on_stop(self, req, resp, service_name):
        cmd_name = MOTORS[service_name][0]
        self._send_serial(stop_command(cmd_name))
        with self._lock:
            self._busy[service_name] = False
            if service_name in self._pumps:
                self._pumps[service_name][1].stop()
        self.get_logger().info(f'stop {cmd_name}')
        resp.success = True
        resp.message = f'stop {cmd_name}'
        return resp

    # -- Dispensing pumps -----------------------------------------------------

    def _pump_config(self, pump_id, motor):
        for key, default in PUMP_DEFAULTS.items():
            self.declare_parameter(f'{motor}.{key}', default)
        value = lambda key: self.get_parameter(f'{motor}.{key}').value  # noqa: E731
        return PumpConfig(
            pump_id=pump_id, motor_name=MOTORS[motor][0],
            speed_steps_s=int(value('speed_hz')), accel_steps_s2=int(value('accel_hz2')),
            direction=value('direction'), steps_per_ml=float(value('pulses_per_ml')),
            max_volume_ml=float(value('max_ml')),
        )

    def _on_dispense(self, req, resp, motor):
        svc, pump = self._pumps[motor]
        resp.expected_s = 0.0
        with self._lock:
            if self._busy[motor]:
                resp.success = False
                resp.message = f'busy: {svc} is still running'
                return resp
            ok, message, expected_s = pump.dispense(float(req.volume_ml), time.monotonic())
            if not ok:
                resp.success = False
                resp.message = (f'volume must be more than 0 and at most '
                                f'{pump.config.max_volume_ml:g} mL'
                                if message == 'out of range' else message)
                return resp
            self._busy[motor] = True   # cleared by the ESP32's done/stopped line
            steps = pump.commanded_steps

        cfg = pump.config
        cmd = move_command(cfg.motor_name, steps, cfg.speed_steps_s, cfg.accel_steps_s2)
        self._send_serial(cmd)
        resp.success = True
        resp.message = message
        resp.expected_s = float(expected_s)
        self.get_logger().info(f'{svc}: {message} -> {cmd}')
        return resp

    def _tick_pumps(self):
        # Safety net: a dispense with no done line from the ESP32 is freed after its
        # expected time plus a margin
        now = time.monotonic()
        with self._lock:
            for motor, (svc, pump) in self._pumps.items():
                if not pump.is_busy:
                    continue
                pump.tick(now)
                if not pump.is_busy:
                    self._busy[motor] = False
                    self.get_logger().warn(f'{svc}: no done from the ESP32 in time; marked idle')

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

    def _on_handover(self, req, resp):
        resp.success = False
        resp.expected_s = 0.0
        x = float(self.get_parameter('handover_x_mm').value)
        y = float(self.get_parameter('handover_y_mm').value)
        if x < 0 or y < 0:
            resp.message = f'handover position x={x:g}, y={y:g} is behind the endstops'
            return resp
        if not self._start_gantry_job('handover', self._handover, (x, y), resp):
            return resp
        try:   # the scan config's travel limits bound the first homing
            cfg = gs.ScanConfig.from_yaml(self._scan_config_path())
            resp.expected_s = float(gs.estimate_handover_s(cfg, x, y))
        except (gs.ScanConfigError, OSError, yaml.YAMLError):
            pass
        resp.success = True
        resp.message = f'cache handover: home, X to {x:g}, Y to {y:g}, home Y, home X'
        self.get_logger().info(f'{resp.message} (up to ~{resp.expected_s:.0f} s)')
        return resp

    def _on_gantry_stop(self, req, resp):
        job = self._gantry_job
        if job is None:
            resp.success = False
            resp.message = 'no gantry job (homing, scan, handover) in progress'
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
                         + ('; position not set, home again' if job in ('homing', 'handover')
                            else ''))
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

    def _handover(self, gantry, x, y):
        gs.run_handover(gantry, x, y, log=self.get_logger().info,
                        stop=self._gantry_stop.is_set)
        self.get_logger().info('cache handover done: X/Y/Z = 0')

    def _scan(self, gantry, cfg, points, out):
        log = self.get_logger()
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, 'w', newline='') as f:
            gs.run_scan(gantry, cfg, points, f,
                        capture=gs.camera_capture(cfg) if cfg.camera_enabled else None,
                        log=log.info, stop=self._gantry_stop.is_set)
        log.info(f'gantry scan done: {len(points)} points logged to {out}')

    # -- Gantry position -----------------------------------------------------

    def _poll_position(self):
        # A gantry job queries pos itself; extra replies would only add noise
        if self._gantry_job is None:
            self._send_serial(pos_command())

    def _update_position(self, update):
        self._position.update(update)
        if len(self._position) == 3:
            self._position_pub.publish(Point(
                x=self._position['x'], y=self._position['y'], z=self._position['z']))

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

            homed = gantry_homed_update(line)
            if homed:
                for axis, is_homed in homed.items():
                    (self._homed.add if is_homed else self._homed.discard)(axis)

            update = gantry_position_update(line)
            if update:
                self._update_position(update)
                if line.startswith('pos'):
                    continue   # polled at position_rate_hz; don't log each reply

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
        # full names; _busy keys are service names (pump, ..., gantry/x, ...)
        svc = self._feedback_to_service.get(firmware_name)
        if svc is None:
            return
        with self._lock:
            self._busy[svc] = False
            if svc in self._pumps:
                self._pumps[svc][1].complete()


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
