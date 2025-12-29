# XPI SwarmNet: Decentralized Robotics OS

## 1. Vision
Creating the world's first fully open, decentralized protocol for swarm robotics that does not depend on external infrastructure (GPS, Cloud, GSM).

## 2. Core Components (The Pillars)

### A. SwarmNet Core (The Protocol)
*   Wrapper for Meshtastic LoRa.
*   Custom application layer (XPI-Link) for robotic teams.
*   Node discovery and topology management system.

### B. Consensus Engine (The Mind)
*   Decentralized decision-making.
*   Distributed mission planning.
*   Dynamic Role Allocation (Scout, Relay, Performer).

### C. Hardware Abstraction (The Body)
*   Standardized schematics for onboard modules (WisBlock, Heltec).
*   Specifications for dropped and stationary repeaters.
*   Autonomous power systems (Solar/BMS).

### D. SwarmStation (The Interface)
*   Web-interface for real-time monitoring.
*   CLI toolkit for field engineers.
*   Integration with ROS2 / PX4 / ArduPilot.

## 3. Development Philosophy
1.  **Offline-First:** Any function must work without internet access.
2.  **Hardware Agnostic:** Software must run on any LoRa-capable board.
3.  **Safety by Design:** Built-in algorithms for collision avoidance and link-loss prevention.

---

## 4. Project Milestones

### Phase 1: Connectivity (Done)
*   Drivers, basic ROS2 bridge, simple telemetry exchange.

### Phase 2: Coordination (Current)
*   Formation keeping algorithms, group voting, power management.

### Phase 3: Intelligence (Next)
*   Semantic data compression, Onboard Edge AI, autonomous missions.