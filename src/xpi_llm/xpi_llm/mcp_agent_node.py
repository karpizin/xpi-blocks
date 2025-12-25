import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Int32, Float32MultiArray
from sensor_msgs.msg import Temperature, RelativeHumidity, Illuminance, BatteryState
from xpi_llm.llm_clients import llm_client_factory, LLMClient
import json
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
        self.declare_parameter('llm_model', 'gemini-1.5-flash')
        self.declare_parameter('api_key', '')
        self.declare_parameter('command_topic', '~/command')
        self.declare_parameter('response_topic', '~/response')
        
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
        
        # Perception subscription
        self.create_subscription(String, '/vlm/scene_description', lambda m: self._update_perception('scene', m.data), 10)

        # 5. Publishers for Tools
        self.relay_pub = self.create_publisher(Bool, '/relay_node/cmd', 10)
        self.display_pub = self.create_publisher(String, '/tft_display/draw_commands', 10)
        self.led_pub = self.create_publisher(Int32, '/ws2812/effect', 10)
        self.motor_a_pub = self.create_publisher(Float32, '/tb6612/motor_a/cmd_vel', 10)
        self.motor_b_pub = self.create_publisher(Float32, '/tb6612/motor_b/cmd_vel', 10)
        
        # VLM Trigger Tool
        self.vlm_trigger_pub = self.create_publisher(String, '/vlm_observer_node/trigger', 10)

        # 6. User Interaction
        self.cmd_sub = self.create_subscription(String, self.get_parameter('command_topic').value, self.command_callback, 10)
        self.resp_pub = self.create_publisher(String, self.get_parameter('response_topic').value, 10)

        self.get_logger().info(f"MCP Agent initialized using {self.llm_model}")

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
                            "relay_id": {"type": "integer"},
                            "state": {"type": "boolean"}
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
                            "speed": {"type": "number", "description": "-1.0 to 1.0"},
                            "duration": {"type": "number"}
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
                            "effect_id": {"type": "integer"}
                        },
                        "required": ["effect_id"]
                    }
                }
            }
        ]

    def command_callback(self, msg):
        threading.Thread(target=self._process, args=(msg.data,)).start()

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
        name = tool_call["name"]
        args = tool_call["arguments"]
        
        if name == "get_visual_update":
            prompt = args.get("custom_prompt", "")
            msg = String(data=prompt)
            self.vlm_trigger_pub.publish(msg)
            return "Requested visual update from VLM."

        if name == "control_relay":
            msg = Bool(data=args["state"])
            self.relay_pub.publish(msg)
            return f"Relay {args.get('relay_id', 0)} -> {args['state']}"
        
        if name == "move_robot":
            speed = float(args["speed"])
            duration = float(args.get("duration", 1.0))
            msg = Float32(data=speed)
            self.motor_a_pub.publish(msg)
            self.motor_b_pub.publish(msg)
            
            def stop_later():
                time.sleep(duration)
                self.motor_a_pub.publish(Float32(data=0.0))
                self.motor_b_pub.publish(Float32(data=0.0))
            
            threading.Thread(target=stop_later).start()
            return f"Driving at {speed} for {duration}s"

        if name == "set_led_effect":
            self.led_pub.publish(Int32(data=int(args["effect_id"])))
            return f"LED Effect -> {args['effect_id']}"
        
        return "Unknown tool"

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
