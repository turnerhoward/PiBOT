"""Minimal desktop stand-in for MicroPython's "utime" module."""

import time as _time


def ticks_ms():
    return int(_time.time() * 1000)


def ticks_us():
    return int(_time.time() * 1_000_000)


def ticks_diff(a, b):
    return a - b


def ticks_add(a, b):
    return a + b


def sleep_ms(x):
    pass


def sleep_us(x):
    pass
