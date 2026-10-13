"""ROS 2 node bridging pump dispense/stop services to the pump ESP32 over serial.

The ESP32 runs FastAccelStepper firmware accepting line-based commands:
    speed <steps/s>   accel <steps/s^2>   dir cw|ccw
    time <seconds>    run   stop   estop
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from astrobiology_interfaces.srv import Dispense
from std_srvs.srv import Trigger

from astrobiology_actuators.pump import PumpConfig, PumpState
from astrobiology_actuators.serial_commands import (
    dispense_commands,
    stop_command,
)

PUMP_IDS = [1, 2, 3]


class PumpNode(Node):

    def __init__(self):
        super().__init__('pump_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('dry_run', False)
        for pid in PUMP_IDS:
            self.declare_parameter(f'pump{pid}.speed_steps_s', 5000)
            self.declare_parameter(f'pump{pid}.accel_steps_s2', 400)
            self.declare_parameter(f'pump{pid}.direction', 'cw')
            self.declare_parameter(f'pump{pid}.steps_per_ml', 1000.0)
            self.declare_parameter(f'pump{pid}.max_volume_ml', 20.0)

        self._dry_run = self.get_parameter('dry_run').value

        # Build pump states from config
        self._pumps = {}
        for pid in PUMP_IDS:
            config = PumpConfig(
                pump_id=pid,
                speed_steps_s=self.get_parameter(f'pump{pid}.speed_steps_s').value,
                accel_steps_s2=self.get_parameter(f'pump{pid}.accel_steps_s2').value,
                direction=self.get_parameter(f'pump{pid}.direction').value,
                steps_per_ml=self.get_parameter(f'pump{pid}.steps_per_ml').value,
                max_volume_ml=self.get_parameter(f'pump{pid}.max_volume_ml').value,
            )
            self._pumps[pid] = PumpState(config)

        # Serial port
        self._lock = threading.Lock()
        self._ser = None
        self._connected = False

        if not self._dry_run:
            self._open_serial()

        # Services
        cb = ReentrantCallbackGroup()
        for pid in PUMP_IDS:
            self.create_service(
                Dispense,
                f'astrobiology/pump{pid}/dispense',
                lambda req, resp, p=pid: self._on_dispense(req, resp, p),
                callback_group=cb,
            )
            self.create_service(
                Trigger,
                f'astrobiology/pump{pid}/stop',
                lambda req, resp, p=pid: self._on_stop(req, resp, p),
                callback_group=cb,
            )

        # Tick timer at 5 Hz
        self.create_timer(0.2, self._tick, callback_group=cb)

        # Background serial read thread
        if self._ser:
            threading.Thread(target=self._read_loop, daemon=True).start()

        mode = 'dry_run' if self._dry_run else 'live'
        self.get_logger().info(f'Pump node started ({mode})')

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
            time.sleep(2.0)             # ESP32 resets on port open
            self._ser.reset_input_buffer()
            self._ser.write(b'\n')      # flush any partial line in firmware
            self._connected = True
            self.get_logger().info(f'Connected to {port}')
        except serial.SerialException as e:
            self._ser = None
            self.get_logger().error(f'Cannot open {port}: {e}')

    # -- Service callbacks ---------------------------------------------------

    def _on_dispense(self, req, resp, pump_id):
        now = time.monotonic()
        with self._lock:
            # Shared-busy: only one pump at a time on the shared ESP32
            other_busy = any(
                p.is_busy for pid, p in self._pumps.items() if pid != pump_id
            )
            if other_busy:
                resp.success = False
                resp.message = 'busy'
                resp.expected_s = 0.0
                self.get_logger().warn(
                    f'pump{pump_id}: rejected (another pump is busy)')
                return resp

            pump = self._pumps[pump_id]
            success, message, expected_s = pump.dispense(req.volume_ml, now)

        if success:
            cfg = pump.config
            cmds = dispense_commands(
                cfg.speed_steps_s, cfg.accel_steps_s2, cfg.direction, expected_s,
            )
            self._send_serial_sequence(cmds)
            self.get_logger().info(f'pump{pump_id}: {message}')
        else:
            self.get_logger().warn(f'pump{pump_id}: rejected: {message}')

        resp.success = success
        resp.message = message
        resp.expected_s = expected_s
        return resp

    def _on_stop(self, req, resp, pump_id):
        with self._lock:
            pump = self._pumps[pump_id]
            success, message = pump.stop()

        self._send_serial(stop_command())
        self.get_logger().info(f'pump{pump_id}: {message}')

        resp.success = success
        resp.message = message
        return resp

    # -- Serial I/O ----------------------------------------------------------

    def _send_serial(self, cmd):
        if self._dry_run:
            self.get_logger().info(f'[dry_run] {cmd}')
            return
        if self._ser:
            with self._lock:
                self._ser.write((cmd + '\n').encode())

    def _send_serial_sequence(self, cmds):
        """Send a list of commands, one per line."""
        if self._dry_run:
            for cmd in cmds:
                self.get_logger().info(f'[dry_run] {cmd}')
            return
        if self._ser:
            with self._lock:
                for cmd in cmds:
                    self._ser.write((cmd + '\n').encode())

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
            self.get_logger().info(f'esp: {line}')
            # "Stopped.", "Stopped after N legs.", or "EMERGENCY STOP"
            if line.startswith('Stopped') or line == 'EMERGENCY STOP':
                with self._lock:
                    for pump in self._pumps.values():
                        if pump.is_busy:
                            pump.complete()

    # -- Timer ---------------------------------------------------------------

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            for pump in self._pumps.values():
                pump.tick(now)


def main(args=None):
    rclpy.init(args=args)
    node = PumpNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node._ser:
            node._ser.write(b'estop\n')  # emergency stop on shutdown
            node._ser.close()
        node.destroy_node()
        rclpy.shutdown()
