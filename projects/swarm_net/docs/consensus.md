# Collective Decision Making in XPI Swarm

## 🎯 Objective
Achieve consensus among all swarm participants on critical issues (mission state changes, leader election, goal definition) in unstable communication environments.

---

## 🏎 Selected Algorithm: Weighted Voting

For Meshtastic (LoRa), we use an optimized voting algorithm considering the "weight" (battery level and link quality) of each node.

### Principles:
1.  **Proposal:** Any node can initiate a vote by sending a `type: "proposal"` packet.
2.  **Observation:** Nodes listen to proposals and compare them with their local state.
3.  **Vote:** A node sends its "opinion" or confirmation.
4.  **Consensus Threshold:** A decision is accepted if > 50% of active nodes vote for it (quorum).

---

## 🛠 Types of Decisions

### 1. Mission State
*   Example: Switching from "Search" to "Rescue" mode.
*   Logic: If 3+ drones detect the target, the swarm votes to change the global state.

### 2. Leader Election
*   Example: Appointing a temporary coordinator to relay data to the base.
*   Criteria: The node with the highest battery charge and best SNR to the base is selected.

### 3. Waypoint Agreement
*   Example: Choosing a rally point.
*   Logic: Nodes average the proposed coordinates to find the optimal center of mass for the group.

---

## 📦 Voting Packet Format (JSON)

```json
{
  "type": "consensus",
  "topic": "mission_mode",
  "proposal_id": "req_12345",
  "value": "RESCUE",
  "node_weight": 0.85,
  "signature": "..." 
}
```

---

## 🚀 Implementation in XPI

The `ConsensusEngine` module will work as an extension of `MeshtasticDriver`.
It maintains a table of current voting processes (`Voting Table`) and emits a `consensus_reached` event once the threshold is met.
