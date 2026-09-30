"""

Regression tests for position/heading tracking and the assumptions
PiBOT.scan() makes about the motion state machine.

Control._tracking() (position/heading dead reckoning) and PiBOT.scan()
(lidar sweep) both sit "above" the motion control code covered by
test_motion_state_machine.py and test_curvature.py. Neither lives in
control.py, so a change there wouldn't otherwise be caught by those
tests. These tests exist to catch the case where a future change to
control.py accidentally breaks one of these dependents.

"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import unittest
from math import pi

from robot_test_utils import make_control, step, run_until_stopped
from commands import Move


class TestPositionAndHeadingTracking(unittest.TestCase):

    def test_heading_tracks_monotonically_during_a_left_rotation(self):
        # PiBOT.scan()'s data-collection loop assumes heading changes
        # monotonically while rotating; this would silently corrupt
        # scan() data if it stopped being true
        ctrl = make_control()
        ctrl._motion_curr = Move('rotate', protect=True,
                                 ang_speed=180*(pi/180), angle=90*(pi/180))
        step(ctrl)
        self.assertEqual(ctrl._motion_state, 'rotate')

        headings = []
        steps = 0
        while ctrl._motion_state == 'rotate' and steps < 5000:
            step(ctrl)
            headings.append(ctrl._heading)
            steps += 1

        self.assertTrue(all(headings[i] <= headings[i+1] + 1e-9
                            for i in range(len(headings) - 1)))
        self.assertAlmostEqual(ctrl._heading * (180/pi), 90, delta=1.0)

    def test_protect_clears_after_a_protected_rotation_finishes(self):
        # scan() waits in "while self._control._protect: continue" after
        # its sweep; this must actually become False or scan() would hang
        ctrl = make_control()
        ctrl._motion_curr = Move('rotate', protect=True,
                                 ang_speed=180*(pi/180), angle=90*(pi/180))
        run_until_stopped(ctrl)
        self.assertFalse(ctrl._protect)

    def test_position_updates_during_an_arc_move(self):
        # used by the .position property and .go_to_position()
        ctrl = make_control()
        ctrl._motion_curr = Move('arc', protect=False, speed=10, radius=20,
                                 arc_length=15, sense='counterclockwise')
        run_until_stopped(ctrl)
        self.assertNotEqual(ctrl._position, [0, 0])
        # arc_length / radius gives the expected heading change (rad)
        expected_heading = (15 / 20) * (180/pi)
        self.assertAlmostEqual(ctrl._heading * (180/pi), expected_heading,
                               delta=2.0)


if __name__ == '__main__':
    unittest.main()
