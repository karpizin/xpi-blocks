import unittest
import sys
import os
import math
import time

# Add scripts path to import modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from gait_engine import GaitEngine

class TestGaitEngine(unittest.TestCase):
    def setUp(self):
        self.gait = GaitEngine(step_height=0.03, step_length=0.05, gait_type='tripod')

    def test_phase_increment(self):
        """Test if phase increments with velocity."""
        initial_phase = self.gait.phase
        # Move forward at 0.1 m/s for 0.1s
        self.gait.calculate_offsets(velocity=[0.1, 0.0], omega=0.0, dt=0.1)
        self.assertTrue(self.gait.phase > initial_phase)

    def test_tripod_alternation(self):
        """Test that in Tripod gait, opposite legs have opposite phases."""
        # RF and LF should be 0.5 phase apart
        offsets = self.gait.calculate_offsets(velocity=[0.1, 0.0], omega=0.0, dt=0.01)
        # Check Z lift
        # In tripod at phase ~0.1, one group is in swing (Z > 0), other in stance (Z <= 0)
        rf_z = offsets['rf']['z']
        lf_z = offsets['lf']['z']
        self.assertNotEqual(rf_z > 0, lf_z > 0)

    def test_ground_search(self):
        """Test Ground Search logic."""
        # Force a contact for RF leg
        self.gait.register_contact('rf', -0.01) # Hit ground at -1cm
        
        # Calculate offsets
        offsets = self.gait.calculate_offsets(velocity=[0.1, 0.0], omega=0.0, dt=0.01)
        
        # RF should now stay at -0.01 if it's in stance/locked swing
        # Note: Depending on phase, it might be stance.
        # Let's check if the contact height is preserved.
        self.assertEqual(self.gait.contact_heights['rf'], -0.01)

if __name__ == '__main__':
    unittest.main()
