# Technical Documentation: Meshtastic Swarm Module

## 1. Overview
The `Meshtastic Swarm` module is designed to provide decentralized communication between robots (drones) in environments without standard network infrastructure (Wi-Fi, 4G/5G). Using LoRa Mesh technology, it allows devices to be integrated into a single network with a range of up to several tens of kilometers.

---

## 2. System Architecture

The system consists of three main layers:

1.  **Hardware Layer**: LoRa radio modules flashed with Meshtastic firmware (Heltec, RAK, etc.).
2.  **Driver Layer (`blocks/drivers/meshtastic`)**: A Python wrapper around the `meshtastic` library, providing an abstraction from the low-level protocol.
3.  **Communication Layer (`src/xpi_comms`)**: ROS2 nodes integrating the radio channel into the robot's common data bus.

---

## 3. Module Components

### 3.1 MeshtasticDriver (`driver.py`)
The primary interface for hardware interaction.
*   **Neighbor DB**: Stores the current state of all visible nodes in the network (ID, SNR, last telemetry).
*   **Event System**: Uses callbacks to notify the system of incoming data.
*   **Methods**:
    *   `broadcast_telemetry(data)`: Sends a packet to all participants.
    *   `send_command(target, cmd)`: Sends a targeted command to a specific node.

### 3.2 Meshtastic Bridge Node (`meshtastic_bridge_node.py`)
The bridge between the Mesh network and ROS2.
*   **Subscriptions**: `/gps/fix` (NavSatFix), `~/outbound_broadcast` (String).
*   **Publications**: `~/neighbors` (JSON String), `~/inbound_commands` (JSON String).
*   **Logic**: Every local GPS fix update is automatically broadcast to the Mesh to inform neighbors.

### 3.3 Swarm Controller (`swarm_controller_node.py`)
An example of swarm control logic.
*   Listens to neighbor coordinates via the Bridge.
*   Calculates distances between robots.
*   **Collision Avoidance**: Generates a detour command to the `/cmd_vel` topic if robots come dangerously close (< 10m by default).

---

## 4. Installation and Setup

### Dependencies
The module requires the following Python packages:
*   `meshtastic`
*   `PyPubSub`

System Requirements:
*   Installed `FFMPEG` (for future audio sample processing).
*   Access to a serial port (typically `/dev/ttyUSB0` or `/dev/ttyACM0`).

### Permissions Setup (Linux)
To work with a USB modem, add the user to the `dialout` group:
```bash
sudo usermod -a -G dialout $USER
```

---

## 5. Usage

### Running via Launch File
To start the entire system (Bridge + Controller):
```bash
ros2 launch xpi_comms meshtastic_swarm.launch.py
```

### Launch File Parameters
*   `address`: Path to the device (e.g., `/dev/ttyUSB0` or an IP address for TCP).
*   `node_name`: Unique ID of your robot in the swarm (e.g., `drone_leader`).
*   `safe_distance`: Threshold for the collision avoidance system.

---

## 6. Data Format

### Telemetry (JSON)
Transmitted in the Mesh in the following format:
```json
{
  "id": "robot_01",
  "type": "telemetry",
  "lat": 55.7558,
  "lon": 37.6173,
  "alt": 150.5
}
```

### Commands (JSON)
Transmitted to the `~/inbound_commands` topic:
```json
{
  "from": "base_station",
  "cmd": {
    "action": "return_to_home",
    "params": {}
  }
}
```

---

## 7. Development Roadmap
1.  **Multi-Channel**: Support for separate channels for telemetry (fast) and commands (reliable).
2.  **Adaptive Sampling**: Dynamic adjustment of telemetry transmission frequency based on the robot's speed.
3.  **Encrypted Swarm**: Configuring AES encryption for private groups of robots.