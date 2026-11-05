"""ROS 2 node bridging drill start/stop services to the drill Arduino over serial.

Services are std_srvs/Trigger: astrobiology/drill/start and astrobiology/drill/stop.
Protocol (firmware/LAdrill.ino, 9600 baud): send "b" to start, "s" to stop;
receive "Motor: BACKWARD" / "Motor: STOP (Coasting)".
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from astrobiology_actuators.drill_commands import START, STOP


class DrillNode(Node):

    def __init__(self):
        super().__init__('drill_node')

        self.declare_parameter('port', '/dev/uno_drill')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('dry_run', False)

        self._dry_run = self.get_parameter('dry_run').value
        self._lock = threading.Lock()
        self._ser = None

        if not self._dry_run:
            self._open_serial()

        cb = ReentrantCallbackGroup()
        self.create_service(
            Trigger, 'astrobiology/drill/start',
            lambda req, resp: self._on_command(resp, START, 'start'),
            callback_group=cb,
        )
        self.create_service(
            Trigger, 'astrobiology/drill/stop',
            lambda req, resp: self._on_command(resp, STOP, 'stop'),
            callback_group=cb,
        )

        if self._ser:
            threading.Thread(target=self._read_loop, daemon=True).start()

        mode = 'dry_run' if self._dry_run else 'live'
        self.get_logger().info(f'Drill node started ({mode})')

    def _open_serial(self):
        try:
            import serial
        except ImportError:
            self.get_logger().error(
                'pyserial not installed; falling back to dry_run')
            self._dry_run = True
            return

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        try:
            self._ser = serial.Serial(port, baud, timeout=0.5)
            time.sleep(2.0)   # opening the port resets the Arduino
            self._ser.reset_input_buffer()
            self.get_logger().info(f'Connected to {port}')
        except serial.SerialException as e:
            self._ser = None
            self.get_logger().error(f'Cannot open {port}: {e}')

    # -- Service callback ------------------------------------------------------

    def _on_command(self, resp, cmd, action):
        self._send_serial(cmd)
        self.get_logger().info(f'drill {action}')
        resp.success = True
        resp.message = cmd
        return resp

    # -- Serial I/O ------------------------------------------------------------

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
                return
            if not line:
                continue
            if line.startswith('Invalid command'):
                self.get_logger().warn(f'arduino: {line}')
            else:
                self.get_logger().info(f'arduino: {line}')


def main(args=None):
    rclpy.init(args=args)
    node = DrillNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._send_serial(STOP)   # never leave the drill spinning
        if node._ser:
            node._ser.close()
        node.destroy_node()
        rclpy.shutdown()
