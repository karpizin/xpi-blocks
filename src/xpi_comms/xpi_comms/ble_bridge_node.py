#!/usr/bin/env python3
import asyncio
import json
import threading
import logging

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions
)

# --- CONFIGURATION (UUIDs) ---
# Generated UUIDs for XPI-Robot
SERVICE_UUID = "A07498CA-AD5B-474E-940D-16F1FBE7E8CD"
# Characteristic for sending commands TO the robot (Write)
CHAR_COMMAND_UUID = "51FF12BB-3ED8-46E5-B4F9-D64E2FEC021B"
# Characteristic for receiving data FROM the robot (Notify/Read)
CHAR_TELEMETRY_UUID = "51FF12BB-3ED8-46E5-B4F9-D64E2FEC021C"

class BLEBridgeNode(Node):
    """
    ROS2 Node that acts as a BLE Peripheral (Server).
    - Accepts commands via 'Command' Characteristic -> /cmd_vel
    - Sends telemetry via 'Telemetry' Characteristic <- /battery, /status
    """
    def __init__(self):
        super().__init__('ble_bridge_node')
        
        self.declare_parameter('device_name', 'XPI-Robot')
        self.device_name = self.get_parameter('device_name').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, '~/incoming_command', 10)

        # Subscribers (Telemetry sources)
        self.create_subscription(Float32, '/battery/voltage', self._battery_callback, 10)
        
        # State
        self.server = None
        self.telemetry_data = {
            "bat": 0.0,
            "status": "ok"
        }
        self.loop = asyncio.new_event_loop()
        
        # Start BLE in a separate thread because it needs its own asyncio loop
        self.ble_thread = threading.Thread(target=self._run_ble_loop)
        self.ble_thread.start()
        
        # Timer to push telemetry updates to BLE clients
        self.create_timer(1.0, self._push_telemetry_timer)
        
        self.get_logger().info(f"BLE Bridge Initialized. Device Name: {self.device_name}")
        self.get_logger().info(f"Service UUID: {SERVICE_UUID}")

    def _battery_callback(self, msg):
        self.telemetry_data["bat"] = round(msg.data, 2)

    def _push_telemetry_timer(self):
        """Periodically update the Telemetry Characteristic"""
        if self.server and self.server.connected:
            try:
                payload = json.dumps(self.telemetry_data)
                # We need to schedule the update in the BLE asyncio loop
                future = asyncio.run_coroutine_threadsafe(
                    self.server.write_request(
                        CHAR_TELEMETRY_UUID,
                        payload.encode('utf-8')
                    ),
                    self.loop
                )
            except Exception as e:
                self.get_logger().debug(f"Failed to push telemetry: {e}")

    def _run_ble_loop(self):
        """Entry point for the background BLE thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._init_ble_server())
        self.loop.run_forever()

    async def _init_ble_server(self):
        self.get_logger().info("Starting BLE Server...")
        
        # Initialize Bless Server
        self.server = BlessServer(name=self.device_name, loop=self.loop)
        
        # --- Add Service ---
        await self.server.add_new_service(SERVICE_UUID)
        
        # --- Add Command Characteristic (Write) ---
        # Properties: Write | WriteWithoutResponse
        # Permissions: Writeable
        await self.server.add_new_characteristic(
            SERVICE_UUID,
            CHAR_COMMAND_UUID,
            (GATTCharacteristicProperties.write | GATTCharacteristicProperties.write_without_response),
            None, # Initial value
            (GATTAttributePermissions.writeable)
        )

        # --- Add Telemetry Characteristic (Notify | Read) ---
        # Properties: Read | Notify
        # Permissions: Readable
        await self.server.add_new_characteristic(
            SERVICE_UUID,
            CHAR_TELEMETRY_UUID,
            (GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify),
            json.dumps({"status": "init"}).encode('utf-8'),
            (GATTAttributePermissions.readable)
        )

        # Register write callback
        self.server.set_write_callback(CHAR_COMMAND_UUID, self._on_ble_write)

        # Start Advertising
        try:
            await self.server.start()
            self.get_logger().info("BLE Advertising started.")
        except Exception as e:
            self.get_logger().error(f"Failed to start advertising: {e}")

    def _on_ble_write(self, characteristic_uuid: str, value: bytes):
        """Callback when phone writes to the Command Characteristic"""
        try:
            data_str = value.decode('utf-8')
            self.get_logger().info(f"BLE Received: {data_str}")
            
            # Publish raw string
            self.command_pub.publish(String(data=data_str))

            # Try to parse as JSON for cmd_vel
            # Expected format: {"lx": 0.5, "az": 1.0} (Linear X, Angular Z)
            try:
                cmd = json.loads(data_str)
                twist = Twist()
                updated = False
                
                if 'lx' in cmd:
                    twist.linear.x = float(cmd['lx'])
                    updated = True
                if 'az' in cmd:
                    twist.angular.z = float(cmd['az'])
                    updated = True
                
                if updated:
                    self.cmd_vel_pub.publish(twist)
                    
            except json.JSONDecodeError:
                pass # Not JSON, maybe just a text command

        except Exception as e:
            self.get_logger().error(f"Error handling BLE write: {e}")

    def destroy_node(self):
        if self.server:
            # Creating a task to stop server
            asyncio.run_coroutine_threadsafe(self.server.stop(), self.loop)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BLEBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
