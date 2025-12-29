#!/usr/bin/env python3
"""
ROS2 Node for Meshtastic Bridge.
Connects the XPI robot event bus with the LoRa Mesh network.
"""
import rclpy
from rclpy.node import Node
import json
import os
import sys

# Add path to our driver and consensus engine
# Using relative paths based on package structure
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../blocks/drivers/meshtastic'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../blocks/swarm/consensus'))
from driver import MeshtasticDriver
from engine import ConsensusEngine

from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix

class MeshtasticBridgeNode(Node):
    def __init__(self):
        super().__init__('meshtastic_bridge')

        # 1. Parameters
        self.declare_parameter('interface', 'serial')
        self.declare_parameter('address', '/dev/ttyUSB0')
        self.declare_parameter('node_name', 'robot_01')
        
        interface_type = self.get_parameter('interface').value
        address = self.get_parameter('address').value
        self.robot_id = self.get_parameter('node_name').value

        # 2. Driver & Consensus Initialization
        self.driver = MeshtasticDriver(interface_type, address)
        self.consensus = ConsensusEngine(self.robot_id)
        
        # Connect callbacks
        # Note: driver implementation must support appending to these lists or being assigned
        if hasattr(self.driver, 'on_telemetry_received'):
            self.driver.on_telemetry_received.append(self._on_mesh_telemetry)
        if hasattr(self.driver, 'on_command_received'):
            self.driver.on_command_received.append(self._on_mesh_command)
        
        try:
            self.driver.connect()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Meshtastic: {e}")

        # 3. ROS2 Interfaces
        self.pub_neighbors = self.create_publisher(String, '~/neighbors', 10)
        self.pub_commands = self.create_publisher(String, '~/inbound_commands', 10)
        self.pub_consensus = self.create_publisher(String, '~/consensus_reached', 10)

        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self._gps_callback, 10)
        self.sub_outbound = self.create_subscription(String, '~/outbound_broadcast', self._outbound_callback, 10)
        
        # Subscription for local consensus requests (e.g., "propose mission mode change")
        self.sub_propose = self.create_subscription(String, '~/propose_decision', self._propose_callback, 10)

        self.get_logger().info(f"Meshtastic Bridge Node started. ID: {self.robot_id}")

    def _on_mesh_command(self, sender_id, command):
        """Processes incoming commands AND votes."""
        if isinstance(command, dict) and command.get("type") == "consensus":
            # Voting logic
            total_nodes = len(self.driver.neighbors) + 1 # + ourselves
            result = self.consensus.process_incoming_vote(sender_id, command, total_nodes)
            
            if result:
                # Consensus reached!
                msg = String()
                msg.data = json.dumps({"topic": command.get("topic"), "value": result})
                self.pub_consensus.publish(msg)
                self.get_logger().info(f"CONSENSUS REACHED: {command.get('topic')} = {result}")
        else:
            # Normal command
            msg = String()
            msg.data = json.dumps({"from": sender_id, "cmd": command})
            self.pub_commands.publish(msg)

    def _on_mesh_telemetry(self, sender_id, telemetry):
        """Handles telemetry updates from other nodes."""
        # This was missing in my previous 'optimized' version
        msg = String()
        msg.data = json.dumps({"node_id": sender_id, "data": telemetry})
        self.pub_neighbors.publish(msg)

    def _propose_callback(self, msg: String):
        """Creates a new proposal to the network from our robot."""
        try:
            data = json.loads(msg.data) # {"topic": "mode", "value": "SEARCH"}
            proposal = self.consensus.create_proposal(data['topic'], data['value'])
            self.driver.broadcast_telemetry(proposal)
            self.get_logger().info(f"Broadcasted proposal: {data['topic']}={data['value']}")
        except Exception as e:
            self.get_logger().error(f"Failed to create proposal: {e}")

    def _gps_callback(self, msg: NavSatFix):
        """Sends local coordinates to the Mesh."""
        # Telemetry is sent once every N seconds to avoid clogging LoRa airtime
        # In a real implementation, a timer or change filter is needed here
        payload = {
            "id": self.robot_id,
            "type": "telemetry",
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt": msg.altitude
        }
        self.driver.broadcast_telemetry(payload)

    def _outbound_callback(self, msg: String):
        """Broadcasts arbitrary data from ROS to the Mesh."""
        try:
            data = json.loads(msg.data)
            self.driver.broadcast_telemetry(data)
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast outbound data: {e}")

    def destroy_node(self):
        if hasattr(self, 'driver'):
            self.driver.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MeshtasticBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()