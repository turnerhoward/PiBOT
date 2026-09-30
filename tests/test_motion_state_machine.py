"""

Regression tests for the motion state machine: pause/resume, rotation
interrupting and auto-resuming other motion, and Sequence chaining.

These protect the behavior described in the Motion and Sequence class
docstrings in motion.py. If one of these tests breaks, a student-facing
behavior documented there has likely changed too.

"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import unittest

from robot_test_utils import make_control, step, run_until_stopped
from motion import Motion
from commands import Move


class TestBasicMotion(unittest.TestCase):

    def test_forward_reaches_distance_setpoint(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('linear', protect=False, speed=10,
                                 distance=10)
        run_until_stopped(ctrl)
        self.assertAlmostEqual(ctrl._dist_curr, 10, delta=0.15)

    def test_forward_ends_stopped_and_unprotected(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('linear', protect=False, speed=10,
                                 distance=10)
        run_until_stopped(ctrl)
        self.assertEqual(ctrl._motion_state, 'stop')
        self.assertFalse(ctrl._protect)


class TestPauseAndResume(unittest.TestCase):

    def test_pause_halts_an_active_move(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('linear', protect=False, speed=10,
                                 distance=20)
        for _ in range(60):
            step(ctrl)
        ctrl._pause()
        for _ in range(500):
            step(ctrl)
            if ctrl._motion_state == 'pause':
                break
        self.assertEqual(ctrl._motion_state, 'pause')
        self.assertTrue(0 < ctrl._dist_curr < 20)

    def test_resume_completes_the_remaining_distance(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('linear', protect=False, speed=10,
                                 distance=20)
        for _ in range(60):
            step(ctrl)
        ctrl._pause()
        for _ in range(500):
            step(ctrl)
            if ctrl._motion_state == 'pause':
                break
        dist_before_resume = ctrl._dist_curr
        ctrl._resume()
        run_until_stopped(ctrl)
        # dist_curr resets to 0 at the start of the resumed segment (it
        # always tracks progress on the *current* segment only), so
        # total distance traveled is the sum of both segments
        total_traveled = dist_before_resume + ctrl._dist_curr
        self.assertAlmostEqual(total_traveled, 20, delta=0.15)

    def test_pause_works_even_on_protected_motion(self):
        # per the .pause() docstring: "Calling .pause() halts any motion
        # command, including protected motion and an active sequence."
        ctrl = make_control()
        ctrl._motion_curr = Move('linear', protect=True, speed=10,
                                 distance=20)
        for _ in range(60):
            step(ctrl)
        self.assertTrue(ctrl._protect)
        ctrl._pause()
        for _ in range(500):
            step(ctrl)
            if ctrl._motion_state == 'pause':
                break
        self.assertEqual(ctrl._motion_state, 'pause')


class TestRotateInterruptsAndResumes(unittest.TestCase):

    def test_rotate_interrupts_then_resumes_linear_motion(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('linear', protect=False, speed=10,
                                 distance=30)
        for _ in range(80):
            step(ctrl)
        dist_before_interrupt = ctrl._dist_curr
        self.assertTrue(0 < dist_before_interrupt < 30)

        # simulate what Motion.rotate_left() does when it interrupts an
        # active, unprotected move
        ctrl._motion_resume = Move('ready')
        if ctrl._motion_state not in ('stop', 'pause', 'rotate'):
            ctrl._motion_resume = ctrl._motion_prev.copy()
        if ctrl._motion_state not in ('stop', 'pause', 'steer'):
            ctrl._pause()
            for _ in range(500):
                step(ctrl)
                if ctrl._motion_state in ('stop', 'pause'):
                    break
        # capture distance *after* the pause fully completes (including
        # the extra distance covered while decelerating) -- this is the
        # same value _resume() itself uses to compute what's left, so
        # it's the correct baseline for checking the total distance
        # traveled once the interrupted move eventually finishes
        dist_at_full_pause = ctrl._dist_curr

        ctrl._motion_curr = Move('rotate', protect=False, ang_speed=3.14,
                                 angle=1.5708)
        for _ in range(500):
            step(ctrl)
            if ctrl._motion_state == 'rotate':
                break
        self.assertEqual(ctrl._motion_state, 'rotate')

        for _ in range(2000):
            step(ctrl)
            if ctrl._motion_state == 'linear':
                break
        self.assertEqual(ctrl._motion_state, 'linear',
                         'linear motion should automatically resume '
                         'after the rotation finishes')
        self.assertLess(ctrl._dist_set, 30 - dist_before_interrupt + 1,
                        'resumed move should remember its remaining '
                        'distance, not restart the full 30 cm')

        run_until_stopped(ctrl)
        total = dist_at_full_pause + ctrl._dist_curr
        self.assertAlmostEqual(total, 30, delta=0.5)


class TestPauseEveryKindOfMotion(unittest.TestCase):
    """Pausing must reach the 'pause' state from every kind of motion.

    Motion.pause() waits for that state, so a kind that never reaches it
    would hang the caller. Steering once regressed this way when the
    four separate control methods were merged into _motion_control().
    """

    def _pause_and_wait(self, ctrl):
        ctrl._pause()
        for _ in range(3000):
            step(ctrl)
            if ctrl._motion_state == 'pause':
                return True
        return False

    def test_pause_from_linear(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('linear', speed=15)
        for _ in range(100):
            step(ctrl)
        self.assertTrue(self._pause_and_wait(ctrl))

    def test_pause_from_rotate(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('rotate', ang_speed=1.5)
        for _ in range(100):
            step(ctrl)
        self.assertTrue(self._pause_and_wait(ctrl))

    def test_pause_from_arc(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('arc', speed=15, radius=20,
                                 sense='counterclockwise')
        for _ in range(100):
            step(ctrl)
        self.assertTrue(self._pause_and_wait(ctrl))

    def test_pause_from_steer(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('arc', speed=10, radius=25,
                                 sense='counterclockwise')
        for _ in range(150):
            step(ctrl)
        ctrl._motion_resume = ctrl._motion_prev.copy()
        ctrl._motion_curr = Move('steer', speed=ctrl._velo_set, radius=8,
                                 arc_length=0, sense='clockwise')
        for _ in range(20):
            step(ctrl)
        self.assertEqual(ctrl._motion_state, 'steer')
        self.assertTrue(self._pause_and_wait(ctrl),
                        'pausing during a steer must reach the pause state')

    def test_paused_steer_can_be_resumed(self):
        ctrl = make_control()
        ctrl._motion_curr = Move('arc', speed=10, radius=25,
                                 sense='counterclockwise')
        for _ in range(150):
            step(ctrl)
        ctrl._motion_resume = ctrl._motion_prev.copy()
        # a discrete steer that is still in progress when paused
        ctrl._motion_curr = Move('steer', speed=ctrl._velo_set, radius=8,
                                 arc_length=12, sense='clockwise')
        for _ in range(15):
            step(ctrl)
        self.assertEqual(ctrl._motion_state, 'steer')
        self.assertTrue(self._pause_and_wait(ctrl))
        ctrl._resume()
        for _ in range(3000):
            step(ctrl)
            if ctrl._motion_state == 'arc':
                break
        self.assertEqual(ctrl._motion_state, 'arc',
                         'after the resumed steer finishes, the original '
                         'arc should continue')


class TestSequence(unittest.TestCase):

    def test_sequence_builds_moves_and_marks_them_protected(self):
        ctrl = make_control()
        mv = Motion(ctrl)
        seq = mv.sequence().forward(10, 5).rotate_left(90, 90).forward(10, 5)
        self.assertEqual(len(seq._moves), 3)
        self.assertTrue(all(m.protect for m in seq._moves))

    def test_sequence_rejects_a_continuous_move(self):
        ctrl = make_control()
        mv = Motion(ctrl)
        seq = mv.sequence()
        seq.forward(10, 0)  # distance=0 means continuous; not allowed
        self.assertEqual(len(seq._moves), 0)
        seq.forward(10, 5)
        self.assertEqual(len(seq._moves), 1)

    def test_sequence_runs_moves_back_to_back(self):
        ctrl = make_control()
        mv = Motion(ctrl)
        seq = mv.sequence()
        seq.forward(10, 5).rotate_left(90, 90).forward(10, 5)
        moves = list(seq._moves)

        # equivalent to what Sequence.run() does, minus its busy-wait on
        # a real background thread (there isn't one in this test)
        ctrl._motion_queue.extend(moves)
        seq._moves = []
        self.assertEqual(len(ctrl._motion_queue), 3)

        for _ in range(50):
            step(ctrl)
            if ctrl._motion_state != 'stop':
                break
        self.assertEqual(ctrl._motion_state, 'linear')
        self.assertTrue(ctrl._protect)

        for _ in range(3000):
            step(ctrl)
            if ctrl._motion_state == 'rotate':
                break
        self.assertEqual(ctrl._motion_state, 'rotate')

        for _ in range(3000):
            step(ctrl)
            if ctrl._motion_state == 'linear':
                break
        self.assertEqual(ctrl._motion_state, 'linear')

        run_until_stopped(ctrl)
        self.assertEqual(ctrl._motion_queue, [])
        self.assertFalse(ctrl._protect)


class TestMoveObject(unittest.TestCase):

    def test_equal_moves_compare_equal(self):
        a = Move('linear', protect=False, speed=10, distance=5)
        b = Move('linear', protect=False, speed=10, distance=5)
        self.assertEqual(a, b)

    def test_moves_differing_in_one_field_compare_unequal(self):
        a = Move('linear', protect=False, speed=10, distance=5)
        c = Move('linear', protect=True, speed=10, distance=5)
        self.assertNotEqual(a, c)

    def test_copy_is_independent_but_equal(self):
        a = Move('linear', protect=False, speed=10, distance=5)
        b = a.copy()
        self.assertEqual(a, b)
        self.assertIsNot(a, b)


if __name__ == '__main__':
    unittest.main()
