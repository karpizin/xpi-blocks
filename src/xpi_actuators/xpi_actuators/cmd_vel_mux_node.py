"""cmd_vel multiplexer with priorities and an emergency-stop latch.

Each source publishes Twist on its own topic (``cmd_vel/rc``, ``cmd_vel/web`` ...; see
``inputs``). The node streams the command of the highest-priority source that is active
to ``output_topic`` (``cmd_vel_safe``), which diff_drive_node (or the robot's drive node)
listens to. Nothing active, or the emergency stop engaged -> zero.

Emergency stop:
* ``estop`` (std_msgs/Bool): ``true`` engages the latch; ``false`` does nothing.
* ``estop_gpio``: a physical stop button. Normally closed by default, so a broken wire
  also stops the robot.
* ``~/release_estop`` (std_srvs/Trigger) releases it; refused while the button is still
  pressed. After a release every source must send a zero command before it is obeyed again.
* ``~/estop_active`` (std_msgs/Bool, latched) and ``~/active_input`` (std_msgs/String) report
  the state.
"""
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from xpi_actuators.cmd_vel_mux import TwistMux, parse_inputs
from xpi_commons.hardware import hardware_unavailable

# Highest priority first: a person holding the RC transmitter or gamepad beats everything,
# then the web/BLE teleop, then autonomous sources.
DEFAULT_INPUTS = [
    'rc:cmd_vel/rc:100:0.5',
    'gamepad:cmd_vel/gamepad:90:0.5',
    'web:cmd_vel/web:80:0.5',
    'ble:cmd_vel/ble:70:0.5',
    'gesture:cmd_vel/gesture:60:0.5',
    'agent:cmd_vel/agent:50:1.0',
    'swarm:cmd_vel/swarm:40:0.5',
]


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')
        self.declare_parameter('inputs', DEFAULT_INPUTS)   # 'name:topic:priority:timeout'
        self.declare_parameter('output_topic', 'cmd_vel_safe')
        self.declare_parameter('publish_rate', 20.0)       # Hz
        self.declare_parameter('estop_topic', 'estop')
        self.declare_parameter('estop_gpio', -1)           # BCM pin of a stop button, -1 = none
        self.declare_parameter('estop_normally_closed', True)  # NC button to GND: open circuit = stop
        self.declare_parameter('estop_on_start', False)    # start latched; release before driving
        self.declare_parameter('mock_hardware', False)

        self.inputs = parse_inputs(self.get_parameter('inputs').value)
        self.mux = TwistMux(self.inputs)
        rate = float(self.get_parameter('publish_rate').value)
        if rate <= 0:
            raise ValueError('publish_rate must be > 0')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.out_pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.estop_pub = self.create_publisher(Bool, '~/estop_active', latched)
        self.active_pub = self.create_publisher(String, '~/active_input', latched)
        for inp in self.inputs:
            self.create_subscription(
                Twist, inp.topic, lambda msg, name=inp.name: self._on_input(name, msg), 10)
        self.create_subscription(Bool, self.get_parameter('estop_topic').value, self._on_estop, 10)
        self.create_service(Trigger, '~/release_estop', self._on_release)

        self.button = None
        self._setup_button()
        if self.get_parameter('estop_on_start').value:
            self.mux.engage_estop('estop_on_start')

        self._active = None   # last ~/active_input label; the first tick always publishes
        self._publish_estop_state()
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'cmd_vel mux: ' + ', '.join(f'{i.name}({i.priority}) {i.topic}' for i in self.inputs) +
            f" -> {self.get_parameter('output_topic').value}")

    # ---------------------------------------------------------------- stop button
    def _setup_button(self):
        pin = self.get_parameter('estop_gpio').value
        if pin < 0:
            return
        from gpiozero import Button, Device
        if self.get_parameter('mock_hardware').value or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            from gpiozero.pins.mock import MockFactory
            if not isinstance(Device.pin_factory, MockFactory):
                Device.pin_factory = MockFactory()
        try:
            self.button = Button(pin, pull_up=True, bounce_time=0.02)
        except Exception as e:
            hardware_unavailable(self, f'Emergency stop button on GPIO{pin}', e)
        self.nc = bool(self.get_parameter('estop_normally_closed').value)
        if self.nc:
            self.button.when_released = self._on_button   # circuit opened: pressed or wire cut
        else:
            self.button.when_pressed = self._on_button
        if self._button_says_stop():
            self.mux.engage_estop(f'stop button on GPIO{pin}')

    def _button_says_stop(self):
        if self.button is None:
            return False
        return (not self.button.is_pressed) if self.nc else self.button.is_pressed

    def _on_button(self, *_):
        self._engage(f"stop button on GPIO{self.get_parameter('estop_gpio').value}")

    # ---------------------------------------------------------------- callbacks
    def _on_input(self, name, msg):
        self.mux.update(name, msg.linear.x, msg.angular.z, time.monotonic())

    def _on_estop(self, msg):
        if msg.data:
            self._engage(f"{self.get_parameter('estop_topic').value} topic")

    def _engage(self, reason):
        if self.mux.engage_estop(reason):
            self.get_logger().error(f'EMERGENCY STOP engaged ({reason}). Release with ~/release_estop.')
            self.out_pub.publish(Twist())
            self._publish_estop_state()

    def _on_release(self, request, response):
        if self._button_says_stop():
            response.success = False
            response.message = 'Stop button is still pressed (or its wire is open): release it first.'
        elif not self.mux.estop_engaged:
            response.success = True
            response.message = 'Emergency stop was not engaged.'
        else:
            self.mux.release_estop()
            response.success = True
            response.message = 'Emergency stop released. Each source must send a stop before it can drive.'
            self.get_logger().warn(response.message)
            self._publish_estop_state()
        return response

    def _publish_estop_state(self):
        self.estop_pub.publish(Bool(data=self.mux.estop_engaged))

    def _tick(self):
        name, v, w = self.mux.select(time.monotonic())
        out = Twist()
        out.linear.x, out.angular.z = float(v), float(w)
        self.out_pub.publish(out)
        label = 'estop' if self.mux.estop_engaged else (name or 'none')
        if label != self._active:
            self._active = label
            self.active_pub.publish(String(data=label))

    def destroy_node(self):
        try:
            self.out_pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        if self.button is not None:
            self.button.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
