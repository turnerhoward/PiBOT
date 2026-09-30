"""Minimal desktop stand-in for MicroPython's "rp2" module (PIO)."""


class PIO:
    SHIFT_LEFT = 0
    SHIFT_RIGHT = 1
    IN_LOW = 0
    OUT_LOW = 0


class StateMachine:
    def __init__(self, *args, **kwargs):
        pass

    def exec(self, *args, **kwargs):
        pass

    def active(self, *args, **kwargs):
        pass

    def rx_fifo(self):
        return 0

    def get(self, *args, **kwargs):
        return 0


def asm_pio(*args, **kwargs):
    def decorator(func):
        return func
    return decorator
