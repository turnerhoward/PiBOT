"""

Regression tests for PiBOT's lidar analysis methods and the Scan class
they share.

These guard against bugs found and fixed in this codebase:

1. The old validation check "not isinstance(x, list) and len(x) < n"
   used "and" where "or" was required, silently accepting too-short
   lists and crashing on non-list input. Validation now happens once,
   in Scan and in a single "is a Scan, has enough points" check per
   method.
2. max_distance() compared against a bare literal 140 instead of the
   named cnst.LIDAR_OUT_OF_RANGE constant it represents.
3. _find_objects() returned positional tuples; it now returns
   _DetectedObject instances with named fields.
4. convert_point_to_xy() relies on convert_to_xy() accepting a
   single-point scan; correcting the "and"/"or" bug briefly made that
   fail, so it is tested explicitly.

"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import unittest
from math import cos, sin, radians

from robot_test_utils import make_pibot
import constants as cnst
from pibot import Scan, _DetectedObject


class TestScanClass(unittest.TestCase):

    def test_valid_scan_stores_the_lists(self):
        angle = [0.0, 1.0, 2.0]
        distance = [10.0, 11.0, 12.0]
        scan = Scan(angle, distance)
        self.assertEqual(scan.angle, angle)
        self.assertEqual(scan.distance, distance)
        self.assertEqual(len(scan), 3)

    def test_scan_iterates_as_angle_distance_pairs(self):
        scan = Scan([0.0, 1.0], [10.0, 11.0])
        self.assertEqual(list(scan), [(0.0, 10.0), (1.0, 11.0)])

    def test_mismatched_lengths_give_an_empty_scan(self):
        scan = Scan([0.0, 1.0, 2.0], [10.0, 11.0])
        self.assertEqual(len(scan), 0)

    def test_non_list_input_gives_an_empty_scan_without_crashing(self):
        self.assertEqual(len(Scan(5, [1.0])), 0)
        self.assertEqual(len(Scan([1.0], 5)), 0)
        self.assertEqual(len(Scan(None, None)), 0)

    def test_non_numeric_values_give_an_empty_scan(self):
        self.assertEqual(len(Scan([0.0, 'a'], [1.0, 2.0])), 0)
        self.assertEqual(len(Scan([0.0, 1.0], [1.0, None])), 0)

    def test_scan_does_not_copy_but_separate_scans_are_independent(self):
        # documented behavior: Scan wraps the lists it is given, so two
        # Scans built from separate lists never affect each other
        first = Scan([0.0, 1.0, 2.0], [1.0, 2.0, 3.0])
        second = Scan([5.0, 6.0, 7.0], [4.0, 5.0, 6.0])
        second.angle.clear()
        self.assertEqual(len(first), 3)


class TestArgumentValidation(unittest.TestCase):
    """Every analysis method must reject a non-Scan and a too-short Scan
    with a friendly message (returning None) instead of crashing."""

    def setUp(self):
        self.robot = make_pibot()

    def _rejects_bad_input(self, method, min_points):
        # not a Scan at all (including the old two-list style)
        self.assertIsNone(method(5))
        self.assertIsNone(method([1.0, 2.0, 3.0]))
        # a Scan that is too short
        short = Scan([float(i) for i in range(min_points - 1)],
                     [10.0]*(min_points - 1))
        self.assertIsNone(method(short))
        # a Scan that failed validation (empty)
        self.assertIsNone(method(Scan(5, 5)))

    def test_max_distance_validation(self):
        self._rejects_bad_input(self.robot.max_distance, 3)

    def test_min_distance_validation(self):
        self._rejects_bad_input(self.robot.min_distance, 3)

    def test_convert_to_xy_validation(self):
        self._rejects_bad_input(self.robot.convert_to_xy, 1)

    def test_center_point_validation(self):
        self._rejects_bad_input(self.robot.center_point, 3)

    def test_centroid_validation(self):
        self._rejects_bad_input(self.robot.centroid, 3)

    def test_find_corners_validation(self):
        self._rejects_bad_input(self.robot.find_corners, 3)

    def test_detect_objects_validation(self):
        self._rejects_bad_input(self.robot.detect_objects, 3)

    def test_valid_scan_is_still_accepted(self):
        scan = Scan([0.0, 1.0, 2.0], [10.0, 12.0, 11.0])
        self.assertEqual(self.robot.max_distance(scan), (1.0, 12.0))
        self.assertEqual(self.robot.min_distance(scan), (0.0, 10.0))


class TestConvertToXY(unittest.TestCase):

    def setUp(self):
        self.robot = make_pibot()

    def test_converts_polar_points_to_xy(self):
        scan = Scan([0.0, 90.0], [10.0, 20.0])
        x, y = self.robot.convert_to_xy(scan)
        self.assertAlmostEqual(x[0], 10.0, places=1)
        self.assertAlmostEqual(y[0], 0.0, places=1)
        self.assertAlmostEqual(x[1], 0.0, places=1)
        self.assertAlmostEqual(y[1], 20.0, places=1)

    def test_convert_point_to_xy_accepts_a_single_point(self):
        # convert_point_to_xy wraps one point in a one-point Scan, so
        # convert_to_xy must accept a scan with a single point
        x, y = self.robot.convert_point_to_xy(30.0, 10.0)
        self.assertAlmostEqual(x, round(10*cos(radians(30)), 1), places=1)
        self.assertAlmostEqual(y, round(10*sin(radians(30)), 1), places=1)


class TestLidarOutOfRangeConstant(unittest.TestCase):

    def test_constant_matches_lidar_max_plus_offset(self):
        self.assertEqual(cnst.LIDAR_OUT_OF_RANGE,
                         cnst.LIDAR_MAX + cnst.LIDAR_OFFSET)

    def test_max_distance_finds_center_of_out_of_range_block(self):
        robot = make_pibot()
        angle = [float(a) for a in range(-20, 21)]
        distance = ([50.0]*15 + [cnst.LIDAR_OUT_OF_RANGE]*11 + [50.0]*15)
        result = robot.max_distance(Scan(angle, distance))
        self.assertEqual(result[1], cnst.LIDAR_OUT_OF_RANGE)
        # center of the out-of-range block (indices 15-25) is index 20,
        # which corresponds to angle 0.0
        self.assertAlmostEqual(result[0], 0.0, delta=1.0)


class TestDetectedObject(unittest.TestCase):

    def setUp(self):
        self.robot = make_pibot()
        # flat background at 100 cm with a sharp dip to 30 cm between
        # roughly -10 and +10 degrees -- a single, clear object
        angle = [float(a) for a in range(-40, 41)]
        distance = [30.0 if -10 <= a <= 10 else 100.0 for a in angle]
        self.scan = Scan(angle, distance)

    def test_find_objects_returns_detected_object_instances(self):
        objects = self.robot._find_objects(self.scan)
        self.assertEqual(len(objects), 1)
        self.assertIsInstance(objects[0], _DetectedObject)

    def test_detected_object_fields_are_readable_by_name(self):
        obj = self.robot._find_objects(self.scan)[0]
        self.assertAlmostEqual(obj.min_distance, 30.0)
        self.assertGreater(obj.width, 0)
        self.assertLess(abs(obj.center_angle), 5)
        self.assertIsInstance(obj.leading_edge, int)
        self.assertIsInstance(obj.trailing_edge, int)

    def test_detect_objects_reads_the_same_fields_as_find_objects(self):
        objects = self.robot._find_objects(self.scan)
        obj_angle, obj_dist, obj_width = self.robot.detect_objects(self.scan)
        self.assertEqual(obj_angle, [objects[0].center_angle])
        self.assertEqual(obj_dist, [objects[0].min_distance])
        self.assertEqual(obj_width, [objects[0].width])


class TestCornersAndCentroid(unittest.TestCase):
    """Exercise the remaining Scan consumers on a simple square room."""

    def setUp(self):
        self.robot = make_pibot()
        # 360 degree scan (1 degree steps) from the center of a 100 cm
        # square room: distance to the wall along each ray
        angle, distance = [], []
        for a in range(-180, 180):
            c, s = abs(cos(radians(a))), abs(sin(radians(a)))
            distance.append(round(50 / max(c, s), 1))
            angle.append(float(a))
        self.scan = Scan(angle, distance)

    def test_center_point_and_centroid_of_a_centered_room(self):
        cx, cy = self.robot.center_point(self.scan)
        self.assertAlmostEqual(cx, 0.0, delta=1.5)
        self.assertAlmostEqual(cy, 0.0, delta=1.5)
        gx, gy = self.robot.centroid(self.scan)
        self.assertAlmostEqual(gx, 0.0, delta=1.5)
        self.assertAlmostEqual(gy, 0.0, delta=1.5)

    def test_find_corners_returns_matching_length_lists(self):
        corner_angle, corner_dist = self.robot.find_corners(self.scan)
        self.assertEqual(len(corner_angle), len(corner_dist))
        # a square room has four corners
        self.assertEqual(len(corner_angle), 4)


if __name__ == '__main__':
    unittest.main()
