"""ROS 2 node bridging science-light on/off services to the ESP32 over serial.

Each light is exposed as a std_srvs/SetBool service.
Protocol: send "<NAME> <0|1>\n", receive echo "<NAME> <val>\n".
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool

from astrobiology_actuators.lights_commands import (
    LIGHTS,
    all_off_commands,
    light_command,
)


class LightsNode(Node):

    def __init__(self):
        super().__init__('lights_node')

        self.declare_parameter('port', '/dev/esp_lights')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('dry_run', False)

        self._dry_run = self.get_parameter('dry_run').value
        self._lock = threading.Lock()
        self._ser = None

        if not self._dry_run:
            self._open_serial()

        cb = ReentrantCallbackGroup()
        for name in LIGHTS:
            self.create_service(
                SetBool, f'astrobiology/{name}/set',
                lambda req, resp, n=name: self._on_set(req, resp, n),
                callback_group=cb,
            )

        if self._ser:
            threading.Thread(target=self._read_loop, daemon=True).start()

        mode = 'dry_run' if self._dry_run else 'live'
        self.get_logger().info(
            f'Lights node started ({mode}), lights: {list(LIGHTS.keys())}')

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
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.get_logger().info(f'Connected to {port}')
        except serial.SerialException as e:
            self._ser = None
            self.get_logger().error(f'Cannot open {port}: {e}')

    # -- Service callback ------------------------------------------------------

    def _on_set(self, req, resp, light_name):
        cmd = light_command(LIGHTS[light_name], req.data)
        self._send_serial(cmd)
        self.get_logger().info(
            f'{light_name} -> {"ON" if req.data else "OFF"}')
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
            self.get_logger().debug(f'esp: {line}')

    # -- Shutdown --------------------------------------------------------------

    def _shutdown_lights(self):
        for cmd in all_off_commands():
            self._send_serial(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LightsNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown_lights()
        if node._ser:
            node._ser.close()
        node.destroy_node()
        rclpy.shutdown()
