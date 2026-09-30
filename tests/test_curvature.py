"""

Regression tests for the (speed, curvature) motion model in
_motion_control(): arc geometry, no leftover curvature bleeding into a
following straight move, and steer changing curvature without a speed
dip.

These protect the unified-control-loop design described in
_motion_control()'s and _set_curvature()'s docstrings in control.py.

"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import unittest

from robot_test_utils import make_control, step, run_until_stopped
import constants as cnst
from commands import Move


class TestArcGeometry(unittest.TestCase):

    def setUp(self):
        self.radius = 20
        self.arc_len = 15

    def test_ccw_arc_outer_and_inner_wheel_distances(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('arc', protect=False, speed=10,
                                 radius=self.radius, arc_length=self.arc_len,
                                 sense='counterclockwise')
        run_until_stopped(ctrl)
        dist_R = ctrl._theta_R * (cnst.WHEEL_DIA/2)
        dist_L = ctrl._theta_L * (cnst.WHEEL_DIA/2)
        expected_outer = self.arc_len * (1 + (cnst.WHEEL_SPAN/2)/self.radius)
        expected_inner = self.arc_len * (1 - (cnst.WHEEL_SPAN/2)/self.radius)
        self.assertAlmostEqual(dist_R, expected_outer, delta=0.1)
        self.assertAlmostEqual(dist_L, expected_inner, delta=0.1)
        self.assertGreater(dist_R, dist_L)

    def test_cw_arc_mirrors_ccw_arc(self):
        ccw = make_control()
        ccw._motion_curr = Move('arc', protect=False, speed=10,
                                radius=self.radius, arc_length=self.arc_len,
                                sense='counterclockwise')
        run_until_stopped(ccw)

        cw = make_control()
        cw._motion_curr = Move('arc', protect=False, speed=10,
                               radius=self.radius, arc_length=self.arc_len,
                               sense='clockwise')
        run_until_stopped(cw)

        ccw_R = ccw._theta_R * (cnst.WHEEL_DIA/2)
        ccw_L = ccw._theta_L * (cnst.WHEEL_DIA/2)
        cw_R = cw._theta_R * (cnst.WHEEL_DIA/2)
        cw_L = cw._theta_L * (cnst.WHEEL_DIA/2)
        self.assertAlmostEqual(cw_R, ccw_L, delta=0.1)
        self.assertAlmostEqual(cw_L, ccw_R, delta=0.1)


class TestCurvatureDoesNotLeak(unittest.TestCase):

    def test_linear_move_after_arc_is_actually_straight(self):
        # this is the exact bug the (v, curvature) design could
        # introduce if _curvature weren't explicitly reset when
        # transitioning to a 'linear' move
        ctrl = make_control()
        ctrl._motion_curr = Move('arc', protect=False, speed=10, radius=15,
                                 arc_length=10, sense='clockwise')
        run_until_stopped(ctrl)
        self.assertNotEqual(ctrl._curvature, 0)

        L_before, R_before = ctrl._theta_L, ctrl._theta_R
        ctrl._motion_curr = Move('linear', protect=False, speed=10,
                                 distance=10)
        run_until_stopped(ctrl)
        self.assertEqual(ctrl._curvature, 0)

        delta_L = ctrl._theta_L - L_before
        delta_R = ctrl._theta_R - R_before
        self.assertAlmostEqual(delta_L, delta_R, places=6,
                               msg='both wheels should travel equally '
                                   'during a straight-line move, even '
                                   'right after a curved one')


class TestSteer(unittest.TestCase):

    def test_steer_changes_curvature_without_a_speed_dip(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('arc', protect=False, speed=10, radius=25,
                                 arc_length=0, sense='counterclockwise')
        for _ in range(150):
            step(ctrl)
        speed_before_steer = ctrl._velo_curr
        self.assertGreater(speed_before_steer, 9)

        ctrl._motion_resume = ctrl._motion_prev.copy()
        ctrl._motion_curr = Move('steer', protect=False,
                                 speed=ctrl._velo_set, radius=8,
                                 arc_length=6, sense='clockwise')
        step(ctrl)
        self.assertAlmostEqual(ctrl._velo_curr, speed_before_steer,
                               delta=0.5)
        self.assertEqual(ctrl._motion_state, 'steer')

    def test_steer_resumes_the_original_arc_when_finished(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('arc', protect=False, speed=10, radius=25,
                                 arc_length=0, sense='counterclockwise')
        for _ in range(150):
            step(ctrl)

        ctrl._motion_resume = ctrl._motion_prev.copy()
        ctrl._motion_curr = Move('steer', protect=False,
                                 speed=ctrl._velo_set, radius=8,
                                 arc_length=6, sense='clockwise')
        step(ctrl)

        for _ in range(500):
            step(ctrl)
            if ctrl._motion_state == 'arc':
                break
        self.assertEqual(ctrl._motion_state, 'arc')
        self.assertGreater(ctrl._velo_curr, 5,
                           'speed should never have been lost during '
                           'the steer-to-arc transition')


if __name__ == '__main__':
    unittest.main()
