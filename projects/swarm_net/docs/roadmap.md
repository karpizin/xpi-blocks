# Roadmap for Meshtastic and Swarm Intelligence Integration

This document describes the development stages of network interaction and collective intelligence in the XPI-Blocks project.

---

## 📅 Short-term Tasks

### 1. Connectivity Improvements
- [ ] **Multi-Channel Support:** Separating channels into broadcast (telemetry) and secured (commands).
- [ ] **Adaptive Sampling:** Dynamic adjustment of telemetry transmission frequency (rarely when stationary, frequently when moving).
- [ ] **Link Quality Metrics:** Deeper analysis of SNR and Hop Limit to select optimal repeaters.

### 2. Collective Intelligence (Consensus & Voting)
- [ ] **Leader Election:** Automatic selection of a swarm "coordinator" based on charge level and link quality.
- [ ] **Waypoint Averaging:** Consensus algorithm for an agreed rally or patrolling point.
- [ ] **Status Voting:** Agreed decision-making on mission completion or return to base.

### 3. Security
- [ ] **Message Signing:** Implementation of digital signatures to protect against false command injection into the Mesh network.
- [ ] **Encrypted Payload:** Optional data encryption within packets (in addition to standard Meshtastic encryption).

---

## 🚀 Mid-term Tasks

### 4. Swarm Behavior
- [ ] **Boids Implementation:** Full implementation of cohesion, alignment, and separation algorithms for smooth group movement.
- [ ] **Area Coverage Algorithm:** Distribution of search zones among drones to minimize overlap.
- [ ] **Dynamic Re-routing:** Group obstacle avoidance based on data from the "lead" drone.

### 5. Visualization and Monitoring
- [ ] **Swarm Dashboard:** Web interface to display all nodes on a map in real-time.
- [ ] **Consensus Progress Bar:** Visualization of the voting process and quorum reaching.
- [ ] **Network Topology View:** Graph of connections between robots (who is relaying through whom).

---

## 🔮 Long-term Tasks

### 6. Full Autonomy
- [ ] **Decentralized Task Auction:** Robots "bid" for task execution autonomously without operator participation.
- [ ] **Swarm AI:** Training the swarm to perform complex tactical maneuvers via Edge-LLM.
- [ ] **Self-Healing Network:** Automatic filling of coverage gaps upon unit loss.