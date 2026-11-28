import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from xpi_llm.llm_clients import DEFAULT_GEMINI_MODEL, llm_client_factory, LLMClient
from xpi_llm.command_queue import CommandQueue
from xpi_llm.tool_args import as_bool, as_float, as_int, relay_topic
import os

class ToolCallingNode(Node):
    """
    ROS2 Node that interprets natural language commands and calls ROS2 services/topics
    based on LLM's tool calling capabilities.
    """

    def __init__(self):
        super().__init__('llm_tool_calling_node')

        # 1. Declare Parameters
        self.declare_parameter('llm_client_type', 'gemini') # openrouter, gemini, ollama
        self.declare_parameter('llm_api_key', '')
        self.declare_parameter('llm_model', DEFAULT_GEMINI_MODEL)  # '' = the client's default
        self.declare_parameter('ollama_host', 'http://localhost:11434')
        self.declare_parameter('llm_temperature', 0.1) # Keep low for reliable tool calls
        self.declare_parameter('llm_max_tokens', 200)
        self.declare_parameter('command_topic', '~/command')
        self.declare_parameter('response_topic', '~/response')
        # Topic of the PCA9685 driver: /pwm_driver/cmd with pca9685.launch.py,
        # /pca9685_node/cmd with `ros2 run`.
        self.declare_parameter('pwm_topic', '/pwm_driver/cmd')
        
        # 2. Read Parameters
        self.llm_client_type = self.get_parameter('llm_client_type').value
        self.llm_api_key = (self.get_parameter('llm_api_key').value
                            or os.environ.get('GEMINI_API_KEY' if self.llm_client_type == 'gemini'
                                              else 'OPENROUTER_API_KEY', ''))
        self.llm_model = self.get_parameter('llm_model').value
        self.ollama_host = self.get_parameter('ollama_host').value
        self.llm_temperature = self.get_parameter('llm_temperature').value
        self.llm_max_tokens = self.get_parameter('llm_max_tokens').value
        self.command_topic = self.get_parameter('command_topic').value
        self.response_topic = self.get_parameter('response_topic').value

        # 3. Initialize LLM Client
        self.llm_client: LLMClient = None
        try:
            if self.llm_client_type == 'ollama':
                self.llm_client = llm_client_factory(
                    self.llm_client_type, host=self.ollama_host, model=self.llm_model
                )
            else:
                self.llm_client = llm_client_factory(
                    self.llm_client_type, api_key=self.llm_api_key, model=self.llm_model
                )
            self.get_logger().info(f'LLM Client initialized: {self.llm_client.get_model_name()}')
        except ValueError as e:
            self.get_logger().error(f'Failed to initialize LLM client: {e}. '
                                    'Please check client type, API key, and model parameters.')
            self.llm_client = None
        except ImportError as e:
            self.get_logger().error(f'Missing LLM library: {e}. '
                                    'Please install it (e.g., pip install google-genai).')
            self.llm_client = None

        # 4. Define available tools (ROS2 interfaces)
        self.tools = self._define_tools()
        self.get_logger().info(f"Loaded {len(self.tools)} tools for LLM interaction.")
        
        # 5. Publishers for actuators
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        self.relay_publishers = {}
        for i in range(16): # Assuming up to 16 relays controllable by relay_node
            # We'll use relay_node as a base name, then allow remapping for specific instances
            self.relay_publishers[i] = self.create_publisher(Bool, relay_topic(i), qos_profile)

        self.pca9685_publisher = self.create_publisher(
            Float32MultiArray, self.get_parameter('pwm_topic').value, qos_profile)

        # Commands are handled one at a time, in order (see command_queue.py)
        self.commands = CommandQueue(
            self._process_command_with_llm,
            on_error=lambda e: self.get_logger().error(f'Error during LLM command processing: {e}'))
        
        # 6. Subscribers and Publishers for user interaction
        self.command_subscription = self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            qos_profile
        )
        self.response_publisher = self.create_publisher(String, self.response_topic, qos_profile)
        self.get_logger().info(f'Listening for commands on {self.command_topic}.')

    def _define_tools(self):
        """
        Defines the tools (functions) that the LLM can call.
        Format must match the LLM API's tool definition schema.
        """
        tools_list = []

        # Tool 1: Control a Relay
        tools_list.append(
            {
                "type": "function",
                "function": {
                    "name": "set_relay_state",
                    "description": "Sets the state of a specific relay connected to a GPIO pin. Relays are typically used for switching ON/OFF lights, pumps, or other high-power devices.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "relay_id": {"type": "integer", "description": "The ID of the relay to control (0 to 15). Corresponds to a specific relay_node instance.", "minimum": 0, "maximum": 15},
                            "state": {"type": "boolean", "description": "The desired state: true for ON, false for OFF."}
                        },
                        "required": ["relay_id", "state"]
                    }
                }
            }
        )

        # Tool 2: Set PCA9685 PWM Channel (e.g., for Servos or LED brightness)
        tools_list.append(
            {
                "type": "function",
                "function": {
                    "name": "set_pwm_channel",
                    "description": "Sets the PWM duty cycle for a specific channel on the PCA9685 driver. Used for controlling servos or LED brightness.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "channel_id": {"type": "integer", "description": "The channel ID on the PCA9685 to control (0 to 15).", "minimum": 0, "maximum": 15},
                            "duty_cycle": {"type": "number", "description": "The desired PWM duty cycle, normalized from 0.0 to 1.0 (0.0 for off, 1.0 for full on). For servos, typical values are around 0.05 to 0.10.", "minimum": 0.0, "maximum": 1.0}
                        },
                        "required": ["channel_id", "duty_cycle"]
                    }
                }
            }
        )
        return tools_list

    def command_callback(self, msg: String):
        """Callback for incoming user commands."""
        self.get_logger().info(f'Received command: "{msg.data}"')
        if not self.llm_client:
            self.response_publisher.publish(String(data="Error: LLM client not initialized."))
            return
        
        # The LLM call runs on the worker thread, not in the ROS callback
        if not self.commands.submit(msg.data):
            self.get_logger().warn('Too many commands waiting; dropping this one.')
            self.response_publisher.publish(String(data="Busy: too many commands waiting, try again."))

    def _process_command_with_llm(self, user_command: str):
        """Internal method to call LLM and execute tools, run in a separate thread."""
        try:
            # First, ask LLM to decide on a tool or respond with text
            text_response, tool_call_data = self.llm_client.generate(
                user_command,
                tools=self.tools,
                temperature=self.llm_temperature,
                max_tokens=self.llm_max_tokens
            )

            if tool_call_data:
                self.get_logger().info(f"LLM proposed tool call: {tool_call_data}")
                response_text = self._execute_tool_call(tool_call_data)
                self.response_publisher.publish(String(data=response_text))
            elif text_response:
                self.get_logger().info(f"LLM responded with text: {text_response}")
                self.response_publisher.publish(String(data=text_response))
            else:
                self.get_logger().warn("LLM returned no text and no tool call.")
                self.response_publisher.publish(String(data="LLM could not process your request."))

        except Exception as e:
            self.get_logger().error(f'Error during LLM command processing: {e}')
            self.response_publisher.publish(String(data=f"Error processing command: {e}"))

    def _execute_tool_call(self, tool_call: dict) -> str:
        """Executes the specified tool call."""
        tool_name = tool_call.get("name")
        arguments = tool_call.get("arguments", {})

        self.get_logger().info(f"Executing tool: {tool_name} with args: {arguments}")

        if not isinstance(arguments, dict):
            return f"Invalid arguments for {tool_name}: expected an object."
        try:
            if tool_name == "set_relay_state":
                # Gemini sends numbers as floats (5.0), Ollama often as strings: coerce, then check
                relay_id = as_int(arguments.get("relay_id"), 0, 15)
                state = as_bool(arguments.get("state"))
                if relay_id is None:
                    return f"Invalid relay_id: {arguments.get('relay_id')!r}. Must be an integer between 0 and 15."
                if state is None:
                    return f"Invalid state for relay: {arguments.get('state')!r}. Must be true or false."
                self.relay_publishers[relay_id].publish(Bool(data=state))
                return f"Successfully set relay {relay_id} to {'ON' if state else 'OFF'}."

            elif tool_name == "set_pwm_channel":
                channel_id = as_int(arguments.get("channel_id"), 0, 15)
                duty_cycle = as_float(arguments.get("duty_cycle"), 0.0, 1.0)
                if channel_id is None:
                    return f"Invalid channel_id: {arguments.get('channel_id')!r}. Must be an integer between 0 and 15."
                if duty_cycle is None:
                    return f"Invalid duty_cycle: {arguments.get('duty_cycle')!r}. Must be a number between 0.0 and 1.0."
                # Only this channel: NaN means "leave unchanged" to the PCA9685 driver.
                # (A full array of zeros used to switch off every other servo and LED.)
                data = [float('nan')] * (channel_id + 1)
                data[channel_id] = duty_cycle
                self.pca9685_publisher.publish(Float32MultiArray(data=data))
                return f"Successfully set PCA9685 channel {channel_id} to duty cycle {duty_cycle:.3f}."

            else:
                return f"Unknown tool: {tool_name}."
        except Exception as e:
            self.get_logger().error(f"Error executing tool '{tool_name}': {e}")
            return f"Error executing tool '{tool_name}': {e}"

    def destroy_node(self):
        self.commands.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ToolCallingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
