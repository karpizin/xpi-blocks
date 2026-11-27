#!/usr/bin/env python3
import asyncio
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

from bless import (
    BlessServer,
    GATTCharacteristicProperties,
    GATTAttributePermissions
)

from xpi_comms.ble_protocol import encode_telemetry, parse_command

# --- CONFIGURATION (UUIDs) ---
# Generated UUIDs for XPI-Robot
SERVICE_UUID = "A07498CA-AD5B-474E-940D-16F1FBE7E8CD"
# Characteristic for sending commands TO the robot (Write)
CHAR_COMMAND_UUID = "51FF12BB-3ED8-46E5-B4F9-D64E2FEC021B"
# Characteristic for receiving data FROM the robot (Notify/Read)
CHAR_TELEMETRY_UUID = "51FF12BB-3ED8-46E5-B4F9-D64E2FEC021C"


class BLEBridgeNode(Node):
    """
    ROS2 Node that acts as a BLE Peripheral (Server), built on bless.
    - Accepts commands via 'Command' Characteristic -> /cmd_vel, ~/incoming_command
    - Sends telemetry via 'Telemetry' Characteristic (read + notify) <- /battery/voltage

    Safety: with require_encryption (default) the phone must pair before it can write or
    read; motion stops when the phone disconnects or stops sending for cmd_timeout seconds.

    Threads: bless runs in its own asyncio loop (self.loop, background thread). bless calls
    the read/write callbacks on that loop; ROS timers hand work to it with
    loop.call_soon_threadsafe / run_coroutine_threadsafe.
    """

    def __init__(self):
        super().__init__('ble_bridge_node')

        self.declare_parameter('device_name', 'XPI-Robot')
        # Writes/reads need an encrypted (paired) link; set False only for bench tests
        self.declare_parameter('require_encryption', True)
        # Stop if a moving robot receives no motion command for this long (0 = off)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('telemetry_period', 1.0)
        self.device_name = self.get_parameter('device_name').value
        self.require_encryption = self.get_parameter('require_encryption').value
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, '~/incoming_command', 10)

        # Subscribers (Telemetry sources)
        self.create_subscription(Float32, '/battery/voltage', self._battery_callback, 10)

        # State
        self.server = None
        self.ready = False            # GATT server built and advertising
        self.connected = False        # a central is connected (polled from bless)
        self.telemetry_data = {"bat": 0.0, "status": "ok"}
        self._state_lock = threading.Lock()
        self._moving = False
        self._last_motion_time = time.monotonic()

        # Start BLE in a separate thread because it needs its own asyncio loop
        self.loop = asyncio.new_event_loop()
        self.ble_thread = threading.Thread(target=self._run_ble_loop, daemon=True)
        self.ble_thread.start()

        # Timers: push telemetry; motion watchdog
        self.create_timer(float(self.get_parameter('telemetry_period').value), self._push_telemetry_timer)
        if self.cmd_timeout > 0:
            self.create_timer(min(0.1, self.cmd_timeout / 5.0), self._watchdog_tick)

        self.get_logger().info(f"BLE Bridge Initialized. Device Name: {self.device_name}")
        self.get_logger().info(f"Service UUID: {SERVICE_UUID}")

    # ------------------------------------------------------------------ ROS side
    def _battery_callback(self, msg):
        self.telemetry_data["bat"] = round(msg.data, 2)

    def _push_telemetry_timer(self):
        """Send the telemetry JSON to subscribed phones (runs on the BLE loop)."""
        if not self.ready:
            return
        payload = encode_telemetry(self.telemetry_data)
        self.loop.call_soon_threadsafe(self._set_telemetry, payload)

    def _publish_motion(self, lx, az):
        twist = Twist()
        twist.linear.x = lx
        twist.angular.z = az
        with self._state_lock:
            self._moving = lx != 0.0 or az != 0.0
            self._last_motion_time = time.monotonic()
        self.cmd_vel_pub.publish(twist)

    def _stop(self, reason):
        with self._state_lock:
            was_moving = self._moving
        if was_moving:
            self.get_logger().warn(f"BLE: {reason}, stopping robot.")
            self._publish_motion(0.0, 0.0)

    def _watchdog_tick(self):
        with self._state_lock:
            stale = self._moving and time.monotonic() - self._last_motion_time > self.cmd_timeout
        if stale:
            self._stop(f"no motion command for {self.cmd_timeout:.2f} s")

    # ------------------------------------------------------------------ BLE side (self.loop)
    def _run_ble_loop(self):
        """Entry point for the background BLE thread"""
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._init_ble_server())
        except Exception as e:  # no adapter, no D-Bus, no permission...
            self.get_logger().error(f"BLE server could not start: {e}")
            return
        self.loop.create_task(self._watch_connection())
        self.loop.run_forever()

    async def _init_ble_server(self):
        self.get_logger().info("Starting BLE Server...")

        self.server = BlessServer(name=self.device_name, loop=self.loop)
        # bless dispatches every GATT read/write of this server to these two callbacks
        self.server.read_request_func = self._on_ble_read
        self.server.write_request_func = self._on_ble_write

        if self.require_encryption:
            write_perm = GATTAttributePermissions.write_encryption_required
            read_perm = GATTAttributePermissions.read_encryption_required
        else:
            self.get_logger().warn("BLE: require_encryption is off, any nearby device can drive the robot.")
            write_perm = GATTAttributePermissions.writeable
            read_perm = GATTAttributePermissions.readable

        await self.server.add_new_service(SERVICE_UUID)
        # Command: Write | WriteWithoutResponse
        await self.server.add_new_characteristic(
            SERVICE_UUID,
            CHAR_COMMAND_UUID,
            (GATTCharacteristicProperties.write | GATTCharacteristicProperties.write_without_response),
            None,  # Initial value
            write_perm
        )
        # Telemetry: Read | Notify
        await self.server.add_new_characteristic(
            SERVICE_UUID,
            CHAR_TELEMETRY_UUID,
            (GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify),
            bytearray(encode_telemetry({"status": "init"})),
            read_perm
        )

        await self.server.start()
        self.ready = True
        self.get_logger().info("BLE Advertising started.")

    async def _watch_connection(self, period=0.5):
        """Poll bless for the connection state; stop the robot when the phone goes away."""
        while True:
            try:
                connected = bool(await self.server.is_connected())
            except Exception as e:
                self.get_logger().debug(f"BLE is_connected failed: {e}")
                connected = False
            if connected != self.connected:
                self.connected = connected
                self.get_logger().info(f"BLE client {'connected' if connected else 'disconnected'}.")
                if not connected:
                    self._stop("client disconnected")
            await asyncio.sleep(period)

    def _set_telemetry(self, payload):
        """Store the new value and notify subscribers (bless: set value, then update_value)."""
        try:
            char = self.server.get_characteristic(CHAR_TELEMETRY_UUID)
            if char is None:
                return
            char.value = bytearray(payload)
            self.server.update_value(SERVICE_UUID, CHAR_TELEMETRY_UUID)
        except Exception as e:
            self.get_logger().debug(f"Failed to push telemetry: {e}")

    def _on_ble_read(self, characteristic, **kwargs):
        """bless read callback: return the current value of the characteristic."""
        return characteristic.value

    def _on_ble_write(self, characteristic, value, **kwargs):
        """bless write callback: a phone wrote to one of our characteristics."""
        try:
            if str(characteristic.uuid).upper() != CHAR_COMMAND_UUID:
                return
            characteristic.value = value
            text, motion = parse_command(value)
            if text is None:
                self.get_logger().warn("BLE: ignoring a write that is not short UTF-8 text.")
                return
            self.get_logger().info(f"BLE Received: {text}")
            # Publish raw string
            self.command_pub.publish(String(data=text))
            # {"lx": 0.5, "az": 1.0} (Linear X, Angular Z) -> /cmd_vel
            if motion is not None:
                self._publish_motion(*motion)
        except Exception as e:
            self.get_logger().error(f"Error handling BLE write: {e}")

    def destroy_node(self):
        self._stop("shutting down")
        if self.server is not None and self.loop.is_running():
            try:
                asyncio.run_coroutine_threadsafe(self.server.stop(), self.loop).result(timeout=2.0)
            except Exception as e:
                self.get_logger().debug(f"BLE stop failed: {e}")
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
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
