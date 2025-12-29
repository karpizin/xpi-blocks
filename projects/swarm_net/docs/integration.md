# Meshtastic Integration in XPI-Blocks

## 🎯 Goal
Provide decentralized, autonomous communication for robot groups (drone swarms) and IoT devices in environments without infrastructure (Wi-Fi/GSM), using LoRa Mesh technology.

---

## 📡 Technology: Meshtastic
**Meshtastic** is an open-source project using LoRa radio modules to create a mesh network.
*   **Principle:** Every device acts as a repeater.
*   **Range:** 1-5 km (urban), up to 50+ km (LOS, air-to-ground).
*   **Topology:** Mesh. No single point of failure.
*   **Data:** Telemetry, control commands, short messages.

---

## 🛠 Block Architecture: `meshtastic_driver`

We are creating a new driver in `blocks/drivers/meshtastic` that serves as a bridge between the hardware (LoRa modem) and the XPI event bus.

### Functionality
1.  **Transport Layer:** Support for connection via USB (Serial), TCP/IP (Wi-Fi), and BLE.
2.  **Messaging:**
    *   `broadcast(data)`: Send data to all swarm participants.
    *   `send_direct(node_id, data)`: Target a specific drone.
3.  **Telemetry:** Automatic publishing of local coordinates and status into the Mesh.
4.  **Discovery:** List available nodes with signal quality (SNR).

### Interface (API)

The block provides the following methods and events:

**Methods:**
*   `connect(interface: str, address: str)`
*   `send_text(text: str, channel: int = 0)`
*   `send_data(payload: bytes, port_num: int)`

**Events (Output):**
*   `meshtastic.message`: Message received (text/data).
*   `meshtastic.node_update`: Node status updated (coordinates, battery).
*   `meshtastic.connection_lost`: Lost connection with the radio module.

---

## ⚙️ Reference Hardware

The following modules are recommended for drone swarm implementation:

1.  **Heltec V3 (ESP32 + SX1262):** Excellent for ground stations and debugging (includes OLED screen).
2.  **RAK Wireless (WisBlock RAK4631):** Industrial standard. Energy-efficient (nRF52840), modular. Ideal for drone mounting.
3.  **LilyGO T-Beam:** "All-in-one" option with GPS and 18650 battery holder (heavier for small drones).

**Frequencies:**
*   868 MHz (EU/RU)
*   915 MHz (US)
*   433 MHz (Old standard)

---

## 📦 Development Plan

1.  **Prototype:** Python script to connect to the device and read logs.
2.  **XPI Driver:** Wrapper for `meshtastic-python` into a `Block` class.
3.  **Swarm Protocol:** Development of a data exchange protocol for the swarm (packet format: ID, Lat, Lon, Alt, Status).
4.  **Simulation:** Logic testing without hardware (Mock Hardware).
