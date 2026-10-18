"""ROS 2 node bridging stepper move/stop services to the ESP32 over serial.

Services use StepperMove: the caller passes steps, speed_hz, accel_hz2
which map directly to the firmware command: g1 <motor> <steps> <speed> <accel>
"""

import re
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from astrobiology_interfaces.srv import StepperMove

from astrobiology_actuators.serial_commands import (
    estop_command,
    move_command,
    stop_command,
)

MOTORS = ['pump', 'flush']

# Firmware feedback patterns
_DONE_RE = re.compile(r'^done\s+(\S+)\s+steps=(-?\d+)')
_STOPPED_RE = re.compile(r'^stopped\s+(\S+)\s+steps=(-?\d+)')
_CANCEL_RE = re.compile(r'^cancelled\s+(\S+)')
_ABORT_RE = re.compile(r'^aborted\s+(\S+)')
_OK_RE = re.compile(r'^ok\s+(\S+)\s+accepted')
_ERR_BUSY_RE = re.compile(r'^error\s+busy:\s*(\S+)')


class PumpNode(Node):

    def __init__(self):
        super().__init__('pump_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('dry_run', False)

        self._dry_run = self.get_parameter('dry_run').value
        self._busy = {name: False for name in MOTORS}

        # Serial port
        self._lock = threading.Lock()
        self._ser = None
        self._connected = False

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

        # Background serial read thread
        if self._ser:
            threading.Thread(target=self._read_loop, daemon=True).start()

        mode = 'dry_run' if self._dry_run else 'live'
        self.get_logger().info(
            f'Pump node started ({mode}), motors: {MOTORS}')

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

    def _on_move(self, req, resp, motor_name):
        with self._lock:
            if self._busy[motor_name]:
                resp.success = False
                resp.message = 'busy'
                return resp
            self._busy[motor_name] = True

        cmd = move_command(motor_name, req.steps, req.speed_hz, req.accel_hz2)
        self._send_serial(cmd)
        self.get_logger().info(cmd)

        resp.success = True
        resp.message = cmd
        return resp

    def _on_stop(self, req, resp, motor_name):
        self._send_serial(stop_command(motor_name))
        with self._lock:
            self._busy[motor_name] = False
        self.get_logger().info(f'stop {motor_name}')
        resp.success = True
        resp.message = f'stop {motor_name}'
        return resp

    # -- Serial I/O ----------------------------------------------------------

    def _send_serial(self, cmd):
        if self._dry_run:
            self.get_logger().info(f'[dry_run] {cmd}')
            return
        if self._ser:
            with self._lock:
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

    def _clear_busy(self, motor_name):
        with self._lock:
            if motor_name in self._busy:
                self._busy[motor_name] = False


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
            node._ser.write(b'estop\n')
            node._ser.close()
        node.destroy_node()
        rclpy.shutdown()
