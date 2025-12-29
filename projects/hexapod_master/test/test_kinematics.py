import unittest
import sys
import os
import math

# Add scripts path to import modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from leg_kinematics import LegKinematics
from body_kinematics import BodyKinematics
from interpolator import Interpolator

class TestHexapodMath(unittest.TestCase):
    def test_leg_ik_neutral(self):
        """Test Leg IK in a known neutral position."""
        ik = LegKinematics(l1=0.03, l2=0.05, l3=0.08)
        # RF neutral position relative to coxa
        x, y, z = 0.08, 0.0, -0.04
        angles = ik.calculate_ik(x, y, z)
        
        self.assertAlmostEqual(angles[0], 0.0) # Coxa should be 0
        # Femur and Tibia should be within reasonable bounds
        self.assertTrue(-math.pi < angles[1] < math.pi)
        self.assertTrue(-math.pi < angles[2] < math.pi)

    def test_leg_ik_out_of_reach(self):
        """Test error handling for out of reach points."""
        ik = LegKinematics(l1=0.03, l2=0.05, l3=0.08)
        with self.assertRaises(ValueError):
            ik.calculate_ik(1.0, 1.0, 1.0) # Clearly too far

    def test_interpolator_linear(self):
        """Test smooth interpolation."""
        interp = Interpolator(start_val=0.0, speed=1.0)
        interp.set_target(1.0)
        
        # After 0.5 seconds, value should be 0.5 (assuming speed is 1.0 units/sec)
        # But our update uses internal time. Let's mock time or just check progression.
        v1 = interp.update()
        time.sleep(0.1)
        v2 = interp.update()
        self.assertTrue(v2 > v1)
        self.assertTrue(v2 <= 1.0)

    def test_body_kinematics_translation(self):
        """Test if body translation correctly affects all legs."""
        leg_configs = {
            'leg_rf': {'x': 0.1, 'y': -0.1, 'angle': 0.0}
        }
        body = BodyKinematics(leg_configs)
        
        # Translate body UP by 2cm (z=+0.02)
        # Legs should move DOWN by 2cm relative to body (z=-0.02)
        results = body.calculate_body_ik(translation=[0, 0, 0.02], rotation=[0, 0, 0])
        self.assertAlmostEqual(results['leg_rf']['z'], -0.02)

if __name__ == '__main__':
    unittest.main()
