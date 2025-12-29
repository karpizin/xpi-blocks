"""
Meshtastic Driver for XPI-Blocks

Handles real-time telemetry, commands, and neighbor tracking.
"""
import logging
import json
import time
from typing import Dict, Any, Callable, List
import meshtastic.serial_interface
import meshtastic.tcp_interface
from pubsub import pub # Meshtastic uses pyubsub for events

logger = logging.getLogger(__name__)

class MeshtasticDriver:
    def __init__(self, interface_type="serial", address="/dev/ttyUSB0"):
        self.interface_type = interface_type
        self.address = address
        self.interface = None
        
        # Neighbor DB state
        # ID -> { "last_seen": timestamp, "telemetry": {}, "snr": float }
        self.neighbors: Dict[str, Any] = {}
        
        # Callbacks for external systems (e.g., ROS2)
        self.on_telemetry_received: List[Callable] = []
        self.on_command_received: List[Callable] = []

    def connect(self):
        """Connects to the hardware Meshtastic device."""
        try:
            if self.interface_type == "serial":
                self.interface = meshtastic.serial_interface.SerialInterface(self.address)
            else:
                self.interface = meshtastic.tcp_interface.TCPInterface(self.address)
            
            # Subscribe to incoming packets from Meshtastic
            pub.subscribe(self._on_packet_received, "meshtastic.receive")
            logger.info(f"Successfully connected to Meshtastic @ {self.address}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Meshtastic: {e}")
            return False

    def _on_packet_received(self, packet, interface):
        """Internal handler for all packets from the Mesh."""
        try:
            sender_id = packet.get("fromId")
            decoded = packet.get("decoded", {})
            portnum = decoded.get("portnum")
            
            # 1. Handle telemetry (CUSTOM_APP port or POSITION_APP)
            if portnum == "TELEMETRY_APP" or portnum == "POSITION_APP":
                self._handle_telemetry(sender_id, decoded, packet)
            
            # 2. Handle text messages (commands)
            elif portnum == "TEXT_MESSAGE_APP":
                payload = decoded.get("text", "")
                self._handle_command(sender_id, payload)
                
        except Exception as e:
            logger.error(f"Error processing packet: {e}")

    def _handle_telemetry(self, node_id: str, data: Dict, packet: Dict):
        """Updates neighbor state and notifies subscribers."""
        telemetry = {
            "node_id": node_id,
            "timestamp": time.time(),
            "snr": packet.get("rxSnr"),
            "hop_limit": packet.get("hopLimit"),
            "data": data
        }
        self.neighbors[node_id] = telemetry
        
        for callback in self.on_telemetry_received:
            callback(node_id, data) # Match the expectation of meshtastic_bridge_node

    def _handle_command(self, sender_id: str, payload: str):
        """Validates and executes incoming command."""
        logger.info(f"Received command from {sender_id}: {payload}")
        try:
            # Try to parse JSON command
            cmd_data = json.loads(payload)
            for callback in self.on_command_received:
                callback(sender_id, cmd_data)
        except json.JSONDecodeError:
            # If not JSON, pass as raw string
            for callback in self.on_command_received:
                callback(sender_id, {"raw": payload})

    def broadcast_telemetry(self, telemetry_data: Any):
        """Broadcasts own telemetry to all swarm participants."""
        if not self.interface:
            return
        
        if isinstance(telemetry_data, dict):
            payload = json.dumps(telemetry_data)
        else:
            payload = str(telemetry_data)
            
        self.interface.sendText(payload, wantAck=False)
        logger.debug(f"Broadcasted local telemetry: {payload}")

    def send_command(self, target_node: str, command: Any):
        """Sends a targeted command to a specific node."""
        if not self.interface:
            return
        
        if isinstance(command, dict):
            payload = json.dumps(command)
        else:
            payload = str(command)
            
        self.interface.sendText(payload, destinationId=target_node, wantAck=True)
        logger.info(f"Sent command to {target_node}: {payload}")

    def get_neighbor_states(self) -> List[Dict]:
        """Returns a list of current states of all neighbors."""
        # Optional: filter by time (remove those missing for > 5 min)
        return list(self.neighbors.values())

    def close(self):
        if self.interface:
            self.interface.close()