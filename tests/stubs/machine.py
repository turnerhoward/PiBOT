"""Minimal desktop stand-in for MicroPython's "machine" module.

Only implements what control.py, outputs.py, and sensors.py touch at
import time and during Control initialization. None of it needs to
behave like real hardware, because the test suite replaces the pieces
that actually move the robot; see robot_test_utils.make_control().

"""


class Pin:
    OUT = 0
    IN = 1
    PULL_UP = 2
    PULL_DOWN = 3

    def __init__(self, *args, **kwargs):
        pass

    def value(self, *args, **kwargs):
        return 0

    def irq(self, *args, **kwargs):
        pass

    def low(self, *args, **kwargs):
        pass

    def high(self, *args, **kwargs):
        pass

    def on(self, *args, **kwargs):
        pass

    def off(self, *args, **kwargs):
        pass


class PWM:
    def __init__(self, *args, **kwargs):
        pass

    def freq(self, *args, **kwargs):
        pass

    def duty_u16(self, *args, **kwargs):
        pass

    def duty_ns(self, *args, **kwargs):
        pass


def time_pulse_us(*args, **kwargs):
    return 0
