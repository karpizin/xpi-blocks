#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import json
import os
import sys

# Add paths to our driver and consensus engine
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'blocks', 'drivers', 'meshtastic'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'blocks', 'swarm', 'consensus'))

from driver import MeshtasticDriver
from engine import ConsensusEngine

class MeshtasticBridgeNode(Node):
    def __init__(self):
        super().__init__('meshtastic_bridge_node')
        
        # 1. Parameters
        self.declare_parameter('interface', 'serial') # or 'tcp', 'ble'
        self.declare_parameter('address', '/dev/ttyUSB0')
        self.declare_parameter('node_name', 'robot_01')
        
        interface = self.get_parameter('interface').value
        address = self.get_parameter('address').value
        self.robot_id = self.get_parameter('node_name').value
        
        # 2. Driver & Consensus Init
        self.driver = MeshtasticDriver(node_id=self.robot_id)
        self.consensus = ConsensusEngine(node_id=self.robot_id)
        
        # 3. ROS2 Interfaces
        # Subscriptions
        self.create_subscription(NavSatFix, '/gps/fix', self._gps_callback, 10)
        self.create_subscription(String, '~/outbound_broadcast', self._broadcast_callback, 10)
        self.create_subscription(String, '~/request_consensus', self._consensus_request_callback, 10)

        # Publications
        self.neighbors_pub = self.create_publisher(String, '~/neighbors', 10)
        self.commands_pub = self.create_publisher(String, '~/inbound_commands', 10)
        self.consensus_pub = self.create_publisher(String, '~/consensus_reached', 10)
        
        # Connect to hardware
        try:
            if self.driver.connect(interface, address):
                self.get_logger().info(f'Connected to Meshtastic device at {address}')
            else:
                self.get_logger().error(f'Failed to connect to Meshtastic device!')
        except Exception as e:
            self.get_logger().error(f'Driver connection error: {e}')

        # Set driver callback
        self.driver.on_message = self._on_mesh_message
        
        self.get_logger().info(f'Meshtastic Bridge Node started. ID: {self.robot_id}')

    def _on_mesh_message(self, sender_id, data):
        """Processes incoming commands AND votes."""
        try:
            msg_dict = json.loads(data)
            
            if msg_dict.get('type') == 'consensus':
                # Voting logic
                total_nodes = len(self.driver.neighbors) + 1 # + ourselves
                result = self.consensus.process_incoming_vote(sender_id, msg_dict, total_nodes)
                if result:
                    self.get_logger().info(f'CONSENSUS ACHIEVED: {result}')
                    res_msg = String()
                    res_msg.data = json.dumps({"topic": msg_dict.get("topic"), "value": result})
                    self.consensus_pub.publish(res_msg)
            else:
                # Normal command
                self.commands_pub.publish(String(data=data))
        except Exception as e:
            self.get_logger().debug(f'Error processing mesh message: {e}')

    def _consensus_request_callback(self, msg):
        """Creates a new proposal into the network from our robot."""
        try:
            req = json.loads(msg.data)
            proposal = self.consensus.create_proposal(req['topic'], req['value'])
            self.driver.broadcast_telemetry(json.dumps(proposal))
        except: pass

    def _gps_callback(self, msg):
        """Sends local coordinates to the Mesh."""
        # We send telemetry once every N seconds to avoid clogging LoRa airtime
        # In a real implementation, a timer or change filter is needed here
        telemetry = {
            "id": self.robot_id,
            "type": "telemetry",
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt": msg.altitude
        }
        self.driver.broadcast_telemetry(json.dumps(telemetry))

    def _broadcast_callback(self, msg):
        """Broadcasts arbitrary data from ROS into Mesh."""
        self.driver.broadcast_telemetry(msg.data)

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
