import math
import os
import unittest

import yaml
from body_kinematics import BodyKinematics
from interpolator import Interpolator
from leg_kinematics import LegKinematics

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hexapod_config.yaml')
LINKS = {'l1': 0.03, 'l2': 0.05, 'l3': 0.08}


def load_config():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


class TestLegKinematics(unittest.TestCase):
    def test_leg_ik_neutral(self):
        """A reachable point straight ahead: coxa 0, femur/tibia finite and bounded."""
        ik = LegKinematics(LINKS)
        coxa, femur, tibia = ik.calculate_ik(0.08, 0.0, -0.04)
        self.assertAlmostEqual(coxa, 0.0)
        self.assertTrue(-math.pi < femur < math.pi)
        self.assertTrue(-math.pi < tibia < math.pi)

    def test_leg_ik_matches_forward_kinematics(self):
        """Angles from IK put the foot back where it was asked to go."""
        ik = LegKinematics(LINKS)
        for target in [(0.08, 0.0, -0.04), (0.06, 0.03, -0.05), (0.1, -0.02, -0.02)]:
            coxa, femur, tibia = ik.calculate_ik(*target)
            # planar 2-link FK in the leg plane, then rotate by coxa
            knee_r = LINKS['l2'] * math.cos(femur)
            knee_z = LINKS['l2'] * math.sin(femur)
            foot_r = knee_r + LINKS['l3'] * math.cos(femur + tibia)
            foot_z = knee_z + LINKS['l3'] * math.sin(femur + tibia)
            r = LINKS['l1'] + foot_r
            got = (r * math.cos(coxa), r * math.sin(coxa), foot_z)
            for g, t in zip(got, target):
                self.assertAlmostEqual(g, t, places=6, msg=f'target {target}, got {got}')

    def test_leg_ik_out_of_reach(self):
        ik = LegKinematics(LINKS)
        with self.assertRaises(ValueError):
            ik.calculate_ik(1.0, 1.0, 1.0)


class TestInterpolator(unittest.TestCase):
    def test_moves_towards_target_at_speed(self):
        interp = Interpolator(start_val=0.0, speed=1.0)
        interp.set_target(1.0)
        self.assertAlmostEqual(float(interp.update(0.1)), 0.1)
        self.assertAlmostEqual(float(interp.update(0.2)), 0.3)

    def test_does_not_overshoot(self):
        interp = Interpolator(start_val=[0.0, 0.0], speed=1.0)
        interp.set_target([0.3, 0.4])          # distance 0.5
        out = interp.update(10.0)
        self.assertAlmostEqual(float(out[0]), 0.3)
        self.assertAlmostEqual(float(out[1]), 0.4)


class TestBodyKinematics(unittest.TestCase):
    def test_raising_body_lowers_every_foot(self):
        """Body up by 2 cm -> each foot 2 cm lower in its leg frame, x/y unchanged."""
        body = BodyKinematics(load_config())
        base = body.calculate_body_ik(translation=[0, 0, 0], rotation=[0, 0, 0])
        up = body.calculate_body_ik(translation=[0, 0, 0.02], rotation=[0, 0, 0])
        self.assertEqual(set(base), {'leg_rf', 'leg_rm', 'leg_rb', 'leg_lf', 'leg_lm', 'leg_lb'})
        for leg in base:
            self.assertAlmostEqual(up[leg]['z'] - base[leg]['z'], -0.02, places=9)
            self.assertAlmostEqual(up[leg]['x'], base[leg]['x'], places=9)
            self.assertAlmostEqual(up[leg]['y'], base[leg]['y'], places=9)

    def test_neutral_pose_is_reachable(self):
        cfg = load_config()
        body = BodyKinematics(cfg)
        ik = LegKinematics(cfg['dimensions'])
        for leg, p in body.calculate_body_ik([0, 0, 0], [0, 0, 0]).items():
            ik.calculate_ik(p['x'], p['y'], p['z'])  # raises ValueError if out of reach


if __name__ == '__main__':
    unittest.main()
