import unittest

from gait_engine import GaitEngine


class TestGaitEngine(unittest.TestCase):
    def setUp(self):
        self.gait = GaitEngine(step_height=0.03, step_length=0.05, gait_type='tripod')

    def test_phase_increment(self):
        """The gait phase advances while the robot is commanded to move."""
        before = self.gait.phase
        self.gait.calculate_offsets(velocity=[0.1, 0.0], omega=0.0, dt=0.1)
        self.assertGreater(self.gait.phase, before)

    def test_phase_holds_when_stopped(self):
        before = self.gait.phase
        self.gait.calculate_offsets(velocity=[0.0, 0.0], omega=0.0, dt=0.1)
        self.assertEqual(self.gait.phase, before)

    def test_tripod_alternation(self):
        """In tripod, RF and LF are half a cycle apart: one swings (z > 0) while the other stands."""
        offsets = self.gait.calculate_offsets(velocity=[0.1, 0.0], omega=0.0, dt=0.01)
        self.assertNotEqual(offsets['rf']['z'] > 0, offsets['lf']['z'] > 0)

    def test_ground_contact_locks_height_while_descending(self):
        """A contact in the second half of the swing is kept as the leg's ground height."""
        self.gait.register_contact('rf', -0.01, leg_phase=0.4)   # swing 0..0.5, descending after 0.25
        self.assertTrue(self.gait.contacts['rf'])
        self.assertEqual(self.gait.contact_heights['rf'], -0.01)

    def test_contact_ignored_while_lifting(self):
        self.gait.register_contact('rf', -0.01, leg_phase=0.1)   # first half of swing: still lifting
        self.assertFalse(self.gait.contacts['rf'])


if __name__ == '__main__':
    unittest.main()
