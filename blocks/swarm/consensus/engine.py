"""
XPI Swarm Consensus Engine

Manages voting and collective decision making across the mesh network.
"""
import logging
import time
from typing import Dict, Any, List, Optional

logger = logging.getLogger(__name__)

class ConsensusEngine:
    def __init__(self, node_id: str, quorum_ratio: float = 0.5):
        self.node_id = node_id
        self.quorum_ratio = quorum_ratio
        
        # Active voting processes
        # topic -> { "proposal_id": { "votes": {node_id: value}, "timestamp": float } }
        self.active_votes: Dict[str, Dict] = {}
        
        # Consensus results (agreed values)
        self.agreed_states: Dict[str, Any] = {}

    def process_incoming_vote(self, sender_id: str, vote_data: Dict[str, Any], total_nodes: int):
        """
        Processes an incoming vote from the network.
        """
        topic = vote_data.get("topic")
        proposal_id = vote_data.get("proposal_id")
        value = vote_data.get("value")
        
        if not topic or not proposal_id:
            return None

        if topic not in self.active_votes:
            self.active_votes[topic] = {}
            
        if proposal_id not in self.active_votes[topic]:
            self.active_votes[topic][proposal_id] = {
                "votes": {},
                "start_time": time.time()
            }
            
        # Record the vote
        self.active_votes[topic][proposal_id]["votes"][sender_id] = value
        
        # Check if quorum is reached
        return self._check_consensus(topic, proposal_id, total_nodes)

    def _check_consensus(self, topic: str, proposal_id: str, total_nodes: int):
        """
        Checks if consensus is reached for a specific proposal.
        """
        vote_info = self.active_votes[topic][proposal_id]
        votes = vote_info["votes"]
        
        # Count votes for each value
        counts = {}
        for val in votes.values():
            counts[val] = counts.get(val, 0) + 1
            
        for val, count in counts.items():
            if count / total_nodes > self.quorum_ratio:
                logger.info(f"CONSENSUS REACHED on topic '{topic}': {val}")
                self.agreed_states[topic] = val
                # Clear old votes for this topic
                del self.active_votes[topic]
                return val
                
        return None

    def create_proposal(self, topic: str, value: Any):
        """
        Creates a new proposal to be broadcasted to the network.
        """
        proposal_id = f"prop_{int(time.time())}_{self.node_id}"
        proposal = {
            "type": "consensus",
            "topic": topic,
            "proposal_id": proposal_id,
            "value": value,
            "timestamp": time.time()
        }
        
        # Vote for our own proposal
        if topic not in self.active_votes:
            self.active_votes[topic] = {}
        self.active_votes[topic][proposal_id] = {
            "votes": {self.node_id: value},
            "start_time": time.time()
        }
        
        return proposal

    def get_state(self, topic: str):
        return self.agreed_states.get(topic)