import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Int32
from sensor_msgs.msg import Temperature, RelativeHumidity, Illuminance, BatteryState
from xpi_llm.llm_clients import DEFAULT_GEMINI_MODEL, llm_client_factory
from xpi_llm.command_queue import CommandQueue
from xpi_llm.tool_args import as_bool, as_float, as_int, relay_topic
import json
import math
import os
import threading
import time

class MCPAgentNode(Node):
    """
    ROS2 Node implementing Model Context Protocol logic.
    Aggregates sensor data (Resources) and provides actuator control (Tools).
    """

    def __init__(self):
        super().__init__('mcp_agent_node')

        # 1. Parameters
        self.declare_parameter('llm_model', DEFAULT_GEMINI_MODEL)
        self.declare_parameter('api_key', '')
        self.declare_parameter('command_topic', '~/command')
        self.declare_parameter('response_topic', '~/response')
        # Motion safety: limits enforced in code, whatever the LLM asks for
        # Defaults match tb6612.launch.py / smart_motor.launch.py, which name the driver 'motor_driver'.
        # Running the driver with `ros2 run` instead? Use /tb6612_driver_node/motor_a/cmd_vel.
        self.declare_parameter('motor_a_topic', '/motor_driver/motor_a/cmd_vel')
        self.declare_parameter('motor_b_topic', '/motor_driver/motor_b/cmd_vel')
        self.declare_parameter('max_move_speed', 0.5)      # |speed| limit for move_robot (0..1)
        self.declare_parameter('max_move_duration', 5.0)   # s, upper bound for one move_robot call
        self.declare_parameter('move_keepalive_hz', 10.0)  # re-publish rate; motor drivers stop if commands stop
        self.declare_parameter('max_dac_voltage', 3.3)     # V, upper bound for set_analog_voltage
        
        self.llm_model = self.get_parameter('llm_model').value
        api_key = self.get_parameter('api_key').value or os.environ.get('GEMINI_API_KEY')

        # 2. LLM Client
        self.llm = llm_client_factory('gemini', api_key=api_key, model=self.llm_model)

        # 3. Context (MCP Resources)
        self.robot_state = {
            "sensors": {},
            "perception": {"scene": "Unknown"},
            "actuators": {"relays": [False]*16}
        }
        
        # 4. Subscribers for Resources
        self.create_subscription(Int32, '/scd4x/co2', lambda m: self._update_resource('co2_ppm', m.data), 10)
        self.create_subscription(Float32, '/audio_level/db', lambda m: self._update_resource('noise_db', m.data), 10)
        self.create_subscription(Temperature, '/aht20/temperature', lambda m: self._update_resource('temp_c', m.temperature), 10)
        self.create_subscription(RelativeHumidity, '/aht20/humidity', lambda m: self._update_resource('hum_pct', m.relative_humidity), 10)
        self.create_subscription(Illuminance, '/bh1750/illuminance', lambda m: self._update_resource('light_lux', m.illuminance), 10)
        self.create_subscription(String, '/veml6070/uv_index_level', lambda m: self._update_resource('uv_risk', m.data), 10)
        self.create_subscription(BatteryState, '/ina219/battery_state', self._battery_callback, 10)
        
        # SGP30 Resources
        self.create_subscription(Int32, '/sgp30/eco2', lambda m: self._update_resource('eco2_sgp30_ppm', m.data), 10)
        self.create_subscription(Int32, '/sgp30/tvoc', lambda m: self._update_resource('tvoc_sgp30_ppb', m.data), 10)
        
        # HTU21D / SHT20 Resources
        self.create_subscription(Temperature, '/htu21d/temperature', lambda m: self._update_resource('temp_c_precise', m.temperature), 10)
        self.create_subscription(RelativeHumidity, '/htu21d/humidity', lambda m: self._update_resource('hum_pct_precise', m.relative_humidity * 100.0), 10)
        
        # Perception subscription
        self.create_subscription(String, '/vlm/scene_description', lambda m: self._update_perception('scene', m.data), 10)

        # 5. Publishers for Tools
        # Relay N -> /relay_node/cmd (N=0) or /relay_node_<N>/cmd, as in tool_calling_node
        self.relay_pubs = {i: self.create_publisher(Bool, relay_topic(i), 10) for i in range(16)}
        self.display_pub = self.create_publisher(String, '/tft_display/draw_commands', 10)
        self.led_pub = self.create_publisher(Int32, '/ws2812/effect', 10)
        self.motor_a_pub = self.create_publisher(Float32, self.get_parameter('motor_a_topic').value, 10)
        self.motor_b_pub = self.create_publisher(Float32, self.get_parameter('motor_b_topic').value, 10)
        self.max_move_speed = min(1.0, abs(float(self.get_parameter('max_move_speed').value)))
        self.max_move_duration = max(0.0, float(self.get_parameter('max_move_duration').value))
        self.move_period = 1.0 / max(1.0, float(self.get_parameter('move_keepalive_hz').value))
        self._move_lock = threading.Lock()
        self._move_cancel = None   # threading.Event of the move in progress
        self.dac_pub = self.create_publisher(Float32, '/mcp4725/cmd_voltage', 10)
        self.max_dac_voltage = max(0.0, float(self.get_parameter('max_dac_voltage').value))
        
        # VLM Trigger Tool
        self.vlm_trigger_pub = self.create_publisher(String, '/vlm_observer_node/trigger', 10)

        # 6. User Interaction
        self.cmd_sub = self.create_subscription(String, self.get_parameter('command_topic').value, self.command_callback, 10)
        self.resp_pub = self.create_publisher(String, self.get_parameter('response_topic').value, 10)

        self.commands = CommandQueue(
            self._process, on_error=lambda e: self.get_logger().error(f"LLM Error: {e}"))

        self.get_logger().info(f"MCP Agent initialized using {self.llm.get_model_name()}")

    def _update_resource(self, name, value):
        self.robot_state["sensors"][name] = value

    def _update_perception(self, name, value):
        self.robot_state["perception"][name] = value

    def _battery_callback(self, msg):
        self.robot_state["sensors"]["battery"] = {
            "voltage": round(msg.voltage, 2),
            "current": round(msg.current, 3)
        }

    def _define_mcp_tools(self):
        return [
            {
                "type": "function",
                "function": {
                    "name": "control_relay",
                    "description": "Turn a specific relay ON or OFF.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "relay_id": {"type": "integer", "description": "Relay number, 0 to 15.", "minimum": 0, "maximum": 15},
                            "state": {"type": "boolean", "description": "true = ON, false = OFF."}
                        },
                        "required": ["relay_id", "state"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_visual_update",
                    "description": "Ask the camera/VLM to analyze the current scene. Use this if you need to know what is in front of the robot.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "custom_prompt": {"type": "string", "description": "Specific question for the VLM (e.g. 'Is there a person?')"}
                        }
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "move_robot",
                    "description": "Drive the robot.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "speed": {"type": "number", "description": "-1.0 (reverse) to 1.0 (forward); 0 stops. Limited by the robot's max_move_speed."},
                            "duration": {"type": "number", "description": "Seconds, 0 to max_move_duration (default 1.0)."}
                        },
                        "required": ["speed"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "set_led_effect",
                    "description": "Set LED effect (0-100).",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "effect_id": {"type": "integer", "minimum": 0, "maximum": 100}
                        },
                        "required": ["effect_id"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "set_analog_voltage",
                    "description": "Output a specific analog voltage level using the DAC. Useful for audio level control or analog industrial equipment.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "voltage": {"type": "number", "description": "Target voltage (0.0 to 3.3V)"}
                        },
                        "required": ["voltage"]
                    }
                }
            }
        ]

    def command_callback(self, msg):
        # One command at a time, in order: a "stop" is never overtaken by an earlier "go"
        if not self.commands.submit(msg.data):
            self.get_logger().warn("Too many commands waiting; dropping this one.")
            self.resp_pub.publish(String(data="Busy: too many commands waiting, try again."))

    def _process(self, user_text):
        system_prompt = (
            f"You are the brain of an XPI-based robot. Current State:\n"
            f"{json.dumps(self.robot_state, indent=2)}\n\n"
            f"Instructions: Use tools if needed. To see what's happening, call 'get_visual_update'."
        )
        
        full_prompt = f"{system_prompt}\n\nUser: {user_text}"
        
        try:
            text_resp, tool_call = self.llm.generate(full_prompt, tools=self._define_mcp_tools())

            if tool_call:
                result = self._execute_tool(tool_call)
                self.resp_pub.publish(String(data=f"Action: {result}"))
            else:
                self.resp_pub.publish(String(data=text_resp))
        except Exception as e:
            self.get_logger().error(f"LLM Error: {e}")
            self.resp_pub.publish(String(data=f"Error: {str(e)}"))

    def _execute_tool(self, tool_call):
        """Run one tool call. Arguments are validated here, whatever the model sent."""
        name = tool_call.get("name")
        args = tool_call.get("arguments") or {}
        if not isinstance(args, dict):
            return f"Rejected {name}: arguments must be an object."

        if name == "get_visual_update":
            prompt = args.get("custom_prompt", "")
            self.vlm_trigger_pub.publish(String(data=prompt if isinstance(prompt, str) else ""))
            return "Requested visual update from VLM."

        if name == "control_relay":
            relay_id = as_int(args.get("relay_id"), 0, 15)
            state = as_bool(args.get("state"))
            if relay_id is None or state is None:
                return (f"Rejected control_relay: relay_id must be an integer 0-15 and state true/false "
                        f"(got {args.get('relay_id')!r}, {args.get('state')!r}).")
            self.relay_pubs[relay_id].publish(Bool(data=state))
            return f"Relay {relay_id} -> {'ON' if state else 'OFF'}"

        if name == "move_robot":
            return self._move_robot(args.get("speed"), args.get("duration", 1.0))

        if name == "set_led_effect":
            effect = as_int(args.get("effect_id"), 0, 100)
            if effect is None:
                return f"Rejected set_led_effect: effect_id must be an integer 0-100 (got {args.get('effect_id')!r})."
            self.led_pub.publish(Int32(data=effect))
            return f"LED Effect -> {effect}"

        if name == "set_analog_voltage":
            voltage = as_float(args.get("voltage"), 0.0, self.max_dac_voltage)
            if voltage is None:
                return (f"Rejected set_analog_voltage: voltage must be 0-{self.max_dac_voltage} V "
                        f"(got {args.get('voltage')!r}).")
            self.dac_pub.publish(Float32(data=voltage))
            return f"DAC output set to {voltage}V"

        return f"Unknown tool: {name}"

    # --- Motion (move_robot) -------------------------------------------------
    @staticmethod
    def _finite(value):
        try:
            v = float(value)
        except (TypeError, ValueError):
            return None
        return v if math.isfinite(v) else None

    def _publish_speed(self, speed):
        msg = Float32(data=float(speed))
        self.motor_a_pub.publish(msg)
        self.motor_b_pub.publish(msg)

    def _cancel_move(self):
        with self._move_lock:
            if self._move_cancel is not None:
                self._move_cancel.set()
                self._move_cancel = None

    def _move_robot(self, speed, duration):
        speed = self._finite(speed)
        duration = self._finite(duration if duration is not None else 1.0)
        if speed is None or duration is None:
            self._cancel_move()
            self._publish_speed(0.0)
            return "Rejected move_robot: speed and duration must be finite numbers. Robot stopped."

        req_speed, req_duration = speed, duration
        speed = max(-self.max_move_speed, min(self.max_move_speed, speed))
        duration = max(0.0, min(self.max_move_duration, duration))

        # A new command always replaces the one in progress.
        self._cancel_move()
        if speed == 0.0 or duration == 0.0:
            self._publish_speed(0.0)
            return "Robot stopped."

        cancel = threading.Event()
        with self._move_lock:
            self._move_cancel = cancel

        def drive():
            deadline = time.monotonic() + duration
            try:
                # Stream the command: motor drivers stop on their own if it stops arriving.
                while not cancel.is_set() and time.monotonic() < deadline:
                    self._publish_speed(speed)
                    cancel.wait(min(self.move_period, max(0.0, deadline - time.monotonic())))
            finally:
                # Only the move that still owns the motors sends the stop.
                with self._move_lock:
                    owner = self._move_cancel is cancel
                    if owner:
                        self._move_cancel = None
                if owner or not cancel.is_set():
                    self._publish_speed(0.0)

        threading.Thread(target=drive, daemon=True).start()
        note = ""
        if (req_speed, req_duration) != (speed, duration):
            note = f" (requested {req_speed} for {req_duration}s, limited to max speed {self.max_move_speed}, max {self.max_move_duration}s)"
        return f"Driving at {speed} for {duration}s{note}"

    def destroy_node(self):
        self.commands.stop()
        self._cancel_move()
        self._publish_speed(0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MCPAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
