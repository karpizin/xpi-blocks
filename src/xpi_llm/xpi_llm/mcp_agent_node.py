import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Int32, Float32MultiArray
from sensor_msgs.msg import Temperature, RelativeHumidity
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
        self.declare_parameter('llm_model', 'gemini-3.0-flash')
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
            "actuators": {"relays": [False]*16}
        }
        
        # 4. Subscribers for Resources
        # We subscribe to common sensor topics to build context
        self.create_subscription(Int32, '/scd4x/co2', lambda m: self._update_resource('co2', m.data), 10)
        self.create_subscription(Float32, '/audio_level/db', lambda m: self._update_resource('noise_db', m.data), 10)
        self.create_subscription(Temperature, '/aht20/temperature', lambda m: self._update_resource('temp', m.temperature), 10)
        self.create_subscription(RelativeHumidity, '/aht20/humidity', lambda m: self._update_resource('hum', m.relative_humidity), 10)

        # 5. Publishers for Tools
        self.relay_pub = self.create_publisher(Bool, '/relay_node/cmd', 10)
        self.display_pub = self.create_publisher(String, '/tft_display/draw_commands', 10)
        self.led_pub = self.create_publisher(Int32, '/ws2812/effect', 10)

        # 6. User Interaction
        self.cmd_sub = self.create_subscription(String, self.get_parameter('command_topic').value, self.command_callback, 10)
        self.resp_pub = self.create_publisher(String, self.get_parameter('response_topic').value, 10)

        self.get_logger().info(f"MCP Agent initialized using {self.llm_model}")

    def _update_resource(self, name, value):
        self.robot_state["sensors"][name] = value

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
                            "relay_id": {"type": "integer", "description": "Relay index (0-15)"},
                            "state": {"type": "boolean", "description": "True for ON, False for OFF"}
                        },
                        "required": ["relay_id", "state"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "display_text",
                    "description": "Show a message on the robot's TFT display.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "text": {"type": "string", "description": "The message to display"},
                            "color": {"type": "string", "description": "Color name or HEX"}
                        },
                        "required": ["text"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "display_alert",
                    "description": "Show a high-priority warning on the screen with a red background.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "message": {"type": "string", "description": "The alert message"}
                        },
                        "required": ["message"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "set_led_effect",
                    "description": "Set a visual effect on the RGB LED strip. Examples: 1=Rainbow, 2=Breath, 3=Strobe, 0=Off.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "effect_id": {"type": "integer", "description": "ID of the effect from 0 to 100"}
                        },
                        "required": ["effect_id"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "clear_display",
                    "description": "Wipe the screen and set it to a solid color.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "color": {"type": "string", "description": "Background color (default: black)"}
                        }
                    }
                }
            }
        ]

    def command_callback(self, msg):
        threading.Thread(target=self._process, args=(msg.data,)).start()

    def _process(self, user_text):
        # Build System Prompt with real-time Resource context
        system_prompt = (
            f"You are the brain of an XPI-based robot. Current Environment Context (Resources):\n"
            f"{json.dumps(self.robot_state, indent=2)}\n\n"
            f"Instructions: Respond to the user and use tools if necessary. "
            f"If sensors indicate danger (e.g. high CO2 > 1000ppm or high noise), warn the user immediately."
        )
        
        full_prompt = f"{system_prompt}\n\nUser: {user_text}"
        
        text_resp, tool_call = self.llm.generate(full_prompt, tools=self._define_mcp_tools())

        if tool_call:
            result = self._execute_tool(tool_call)
            self.resp_pub.publish(String(data=f"Action: {result}"))
        else:
            self.resp_pub.publish(String(data=text_resp))

    def _execute_tool(self, tool_call):
        name = tool_call["name"]
        args = tool_call["arguments"]
        
        self.get_logger().info(f"Executing Tool: {name} with {args}")

        if name == "control_relay":
            msg = Bool()
            msg.data = args["state"]
            self.relay_pub.publish(msg)
            return f"Relay {args.get('relay_id')} set to {args['state']}"
        
        if name == "display_text":
            cmd = {
                "commands": [
                    {"type": "clear", "color": "black"},
                    {"type": "text", "content": args["text"], "x": 10, "y": 80, "size": 24, "color": args.get("color", "white")}
                ]
            }
            self.display_pub.publish(String(data=json.dumps(cmd)))
            return f"Displayed text: {args['text']}"

        if name == "display_alert":
            cmd = {
                "commands": [
                    {"type": "clear", "color": "red"},
                    {"type": "text", "content": "!!! ALERT !!!", "x": 10, "y": 40, "size": 30, "color": "white"},
                    {"type": "text", "content": args["message"], "x": 10, "y": 120, "size": 20, "color": "yellow"}
                ]
            }
            self.display_pub.publish(String(data=json.dumps(cmd)))
            return f"Alert displayed: {args['message']}"

        if name == "set_led_effect":
            msg = Int32()
            msg.data = args["effect_id"]
            self.led_pub.publish(msg)
            return f"LED Effect set to {args['effect_id']}"

        if name == "clear_display":
            cmd = {
                "commands": [
                    {"type": "clear", "color": args.get("color", "black")}
                ]
            }
            self.display_pub.publish(String(data=json.dumps(cmd)))
            return "Display cleared"
        
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
