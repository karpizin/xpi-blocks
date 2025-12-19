#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, DataTable, Label, Static
from textual.containers import Container

# Supported message types
from std_msgs.msg import Float32, Int32, String
from sensor_msgs.msg import Temperature, Illuminance, BatteryState, Range, FluidPressure, RelativeHumidity

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('xpi_top_monitor')
        self.sensor_data = {} # Key: Topic, Value: String representation
        self.subs = {} # Key: Topic, Value: Subscription
        
        # Mapping of type strings to classes
        self.type_map = {
            'sensor_msgs/msg/Temperature': Temperature,
            'sensor_msgs/msg/Illuminance': Illuminance,
            'sensor_msgs/msg/BatteryState': BatteryState,
            'sensor_msgs/msg/Range': Range,
            'sensor_msgs/msg/FluidPressure': FluidPressure,
            'sensor_msgs/msg/RelativeHumidity': RelativeHumidity,
            'std_msgs/msg/Float32': Float32,
            'std_msgs/msg/Int32': Int32,
            'std_msgs/msg/String': String
        }

    def scan_topics(self):
        # Discover topics
        topic_names_and_types = self.get_topic_names_and_types()
        
        for topic_name, topic_types in topic_names_and_types:
            if topic_name in self.subs:
                continue # Already subscribed
            
            # Simple filter: check if any type matches our supported list
            # Usually only one type per topic
            msg_type_str = topic_types[0]
            
            if msg_type_str in self.type_map:
                msg_class = self.type_map[msg_type_str]
                self.create_dynamic_sub(topic_name, msg_class)

    def create_dynamic_sub(self, topic, msg_class):
        self.get_logger().info(f"Subscribing to {topic}")
        self.subs[topic] = self.create_subscription(
            msg_class,
            topic,
            lambda msg, t=topic: self.data_callback(t, msg),
            10
        )

    def data_callback(self, topic, msg):
        # Format data nicely based on type
        val_str = "???"
        
        if hasattr(msg, 'temperature'):
            val_str = f"{msg.temperature:.2f} °C"
        elif hasattr(msg, 'illuminance'):
            val_str = f"{msg.illuminance:.1f} Lux"
        elif hasattr(msg, 'relative_humidity'):
            val_str = f"{msg.relative_humidity:.1f} %"
        elif hasattr(msg, 'voltage'): # BatteryState
            val_str = f"{msg.voltage:.2f} V | {msg.current:.3f} A"
        elif hasattr(msg, 'range'):
            val_str = f"{msg.range:.2f} m"
        elif hasattr(msg, 'fluid_pressure'):
            val_str = f"{msg.fluid_pressure/100:.2f} hPa"
        elif hasattr(msg, 'data'): # Float32/Int32/String
            if isinstance(msg.data, float):
                val_str = f"{msg.data:.3f}"
            else:
                val_str = str(msg.data)
        
        self.sensor_data[topic] = val_str

class XPITopApp(App):
    CSS = """
    DataTable {
        height: 100%;
        border: solid green;
    }
    """
    BINDINGS = [("q", "quit", "Quit")]

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

    def compose(self) -> ComposeResult:
        yield Header()
        yield DataTable()
        yield Footer()

    def on_mount(self) -> None:
        table = self.query_one(DataTable)
        table.add_columns("Topic", "Value")
        self.set_interval(0.5, self.update_table) # UI Update 2Hz
        self.set_interval(2.0, self.ros_node.scan_topics) # Topic Scan 0.5Hz

    def update_table(self) -> None:
        table = self.query_one(DataTable)
        
        # Get data snapshot
        data = self.ros_node.sensor_data.copy()
        
        # Update rows
        # Naive approach: clear and redraw (for simplicity in v1)
        # Better: Update cells by key. Textual DataTable uses row_keys.
        
        table.clear()
        for topic in sorted(data.keys()):
            table.add_row(topic, data[topic])

def main(args=None):
    rclpy.init(args=args)
    monitor_node = SensorMonitor()
    
    # Run ROS in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(monitor_node,), daemon=True)
    thread.start()
    
    # Run TUI
    app = XPITopApp(monitor_node)
    app.run()
    
    # Cleanup
    monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
