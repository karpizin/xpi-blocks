#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Temperature
import os
import subprocess

class RTCMonitorNode(Node):
    """
    ROS2 Node for monitoring Real-Time Clock modules (DS3231).
    It publishes the internal temperature of the DS3231 and system/hardware clock status.
    """
    def __init__(self):
        super().__init__('rtc_monitor_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('polling_rate', 0.1) # Once every 10 seconds
        self.declare_parameter('frame_id', 'rtc_link')

        self.bus_id = self.get_parameter('i2c_bus').value
        self.rate = self.get_parameter('polling_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.temp_pub = self.create_publisher(Temperature, '~/internal_temperature', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)

        # Check if RTC is visible in system (hwclock)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.get_logger().info("RTC Monitor Node started.")

    def _get_rtc_temp(self):
        """
        Reads the temperature from DS3231 internal sensor.
        Note: Requires the module to be bound to the kernel driver.
        The temperature is often exposed via sysfs if the driver is loaded.
        """
        # Path for DS3231 temperature in modern kernels
        temp_path = "/sys/class/hwmon/hwmon0/temp1_input"
        # Check if it exists, otherwise scan hwmon
        if not os.path.exists(temp_path):
            # Try to find which hwmon is the RTC
            try:
                for h in os.listdir("/sys/class/hwmon/"):
                    name_path = f"/sys/class/hwmon/{h}/name"
                    if os.path.exists(name_path):
                        with open(name_path, 'r') as f:
                            if "ds3231" in f.read():
                                temp_path = f"/sys/class/hwmon/{h}/temp1_input"
                                break
            except:
                pass

        if os.path.exists(temp_path):
            try:
                with open(temp_path, 'r') as f:
                    return float(f.read()) / 1000.0
            except:
                return None
        return None

    def timer_callback(self):
        # 1. Monitor Temperature (Specific to DS3231)
        temp = self._get_rtc_temp()
        if temp is not None:
            t_msg = Temperature()
            t_msg.header.stamp = self.get_clock().now().to_msg()
            t_msg.header.frame_id = self.frame_id
            t_msg.temperature = temp
            self.temp_pub.publish(t_msg)

        # 2. Monitor Clock Sync
        try:
            # Check hwclock vs system clock
            res = subprocess.check_output(["hwclock", "--show"], stderr=subprocess.STDOUT).decode()
            self.status_pub.publish(String(data=f"RTC OK: {res.strip()}"))
        except Exception as e:
            self.status_pub.publish(String(data=f"RTC Error: {str(e).strip()}"))

def main(args=None):
    rclpy.init(args=args)
    node = RTCMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
