"""Minimal desktop stand-in for the "neopixel" module (NeoPixel LEDs)."""


class NeoPixel:
    def __init__(self, *args, **kwargs):
        n = args[1] if len(args) > 1 else kwargs.get('n', 2)
        self._data = [(0, 0, 0)] * n

    def __setitem__(self, i, value):
        self._data[i] = value

    def __getitem__(self, i):
        return self._data[i]

    def fill(self, value):
        self._data = [value] * len(self._data)

    def write(self):
        pass
