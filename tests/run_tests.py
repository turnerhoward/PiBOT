"""

Runs the full PiBOT motion-logic regression suite on desktop Python.

Usage
-----
    python3 tests/run_tests.py

These tests run entirely on a desktop Python interpreter, using stub
replacements for MicroPython-only modules (see tests/stubs/) and a
"perfect tracking" stand-in for the real motors and encoders (see
robot_test_utils.make_control()). They check the motion state machine
and kinematics logic in control.py, motion.py, and commands.py; they do
not require, and cannot replace, testing on the actual robot.

"""

import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

if __name__ == '__main__':
    loader = unittest.TestLoader()
    suite = loader.discover(start_dir=_HERE, pattern='test_*.py')
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
