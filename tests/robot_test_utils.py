"""

Test utilities shared by every module in this "tests" package.

The PiBOT library imports MicroPython-only modules (machine, rp2,
utime, _thread, neopixel) that don't exist on a desktop Python
interpreter. This module makes the test suite runnable on a laptop in
two steps:

1. It inserts the "stubs" folder (minimal desktop-only fill-ins for
   those MicroPython modules) at the front of sys.path, so "import
   control" pulls in the stub versions instead of failing.
2. It patches _thread.start_new_thread to a no-op *before* control.py
   is imported, so creating a Control object does not spin up a real
   background thread. Instead, tests call step() to run one iteration
   of the control loop's logic directly, on demand, in the same thread
   as the test itself. This makes the tests deterministic: nothing
   races against wall-clock timing the way the real control loop does.

Important
---------
These stubs simulate "perfect" wheel tracking: make_control() replaces
_motors._read_angles() so it always reports exactly the wheel angle
the control loop just commanded, with no PID lag, no motor dynamics,
and no real encoder noise. That is intentional: this test suite checks
the *state machine and kinematics logic* in control.py (does it
compute the right thing at each step?), not the electrical/PID layer
and not real-world behavior like wheel slip. Passing these tests is a
strong signal that a change to the motion logic didn't break the
pause/resume/sequence/curvature behavior this library depends on, but
it is not a substitute for testing on the actual robot before
classroom use.

"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_STUBS = os.path.join(_HERE, 'stubs')
_PACKAGE_ROOT = os.path.dirname(_HERE)

# stub hardware modules take priority; the actual library package (one
# directory up from tests/) comes right after, so "import control",
# "import motion", "import commands", etc. resolve to the real library
# under test
for _path in (_PACKAGE_ROOT, _STUBS):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import gc
if not hasattr(gc, 'threshold'):
    # gc.threshold() is a MicroPython-only API used by Control.__init__()
    # to configure forced garbage collection; desktop Python doesn't have
    # it, so make it a harmless no-op for testing purposes
    gc.threshold = lambda *args, **kwargs: None

import _thread as _real_thread
# prevent Control.__init__() from spinning a real background thread;
# tests drive the control loop themselves with step(), below, instead
_real_thread.start_new_thread = lambda fn, args: None

import control
from commands import Move


class FakeSink:
    """Stands in for the LEDs, Buzzer, and Remote objects Control expects.

    Control._control_loop() checks a few attributes on these objects
    every iteration (for buzzer/LED pulse timing and remote polling).
    The motion-logic tests in this suite don't exercise any of that, so
    this class only needs to exist and expose harmless attributes and
    methods that satisfy those checks without doing anything.

    """

    _pulse_set = False
    _pulse_right_set = False
    _pulse_left_set = False

    class _np:
        @staticmethod
        def write():
            pass

    def _update_command(self):
        pass


def make_control():
    """Creates a Control instance suitable for testing on desktop Python.

    Returns
    -------
    Control
        A fresh Control instance whose _motors._read_angles() has been
        replaced with a "perfect tracking" stub (see module docstring)
        and whose _motors._set_output() is a no-op, so tests can drive
        it with step() without any real hardware attached.

    """

    ctrl = control.Control(leds=FakeSink(), buzzer=FakeSink(),
                           remote=FakeSink())
    ctrl._motors._read_angles = lambda: (ctrl._theta_L, ctrl._theta_R)
    ctrl._motors._set_output = lambda *args: None
    return ctrl


def make_pibot():
    """Creates a full PiBOT instance for testing on desktop Python.

    Returns
    -------
    PiBOT
        A fresh PiBOT instance, built entirely from the stub hardware
        modules in tests/stubs/. Suitable for tests that exercise
        methods not tied to real-time motion control (e.g. the lidar
        analysis methods), which don't need make_control()'s "perfect
        tracking" stub since they operate on plain angle/distance lists
        rather than driving the control loop.

    """

    from pibot import PiBOT
    return PiBOT()


def step(ctrl, n=1):
    """Runs n iterations of the control loop's state-management logic.

    Parameters
    ----------
    ctrl : Control
        A Control instance, normally created with make_control().
    n : int, default=1
        The number of 10 ms control loop iterations to simulate.

    Notes
    -----
    This replays the body of Control._control_loop() -- queue popping,
    state update, motion control, and control action -- minus the
    hardware-timing wait and the buzzer/LED/remote polling, neither of
    which is relevant to motion logic or possible without real hardware.

    """

    for _ in range(n):
        if (ctrl._motion_queue and ctrl._motion_state != 'pause'
                and (not ctrl._protect
                     or (ctrl._protect and ctrl._motion_state == 'stop'))):
            ctrl._motion_curr = ctrl._motion_queue.pop(0)
            ctrl._motion_resume = Move('ready')
            ctrl._update_state()
        if ctrl._motion_curr.kind != 'ready':
            ctrl._update_state()
        if ctrl._motion_state in ('rotate', 'linear', 'arc', 'steer'):
            ctrl._motion_control()
        ctrl._control_action()


def run_until_stopped(ctrl, max_steps=20000):
    """Steps ctrl until its motion state settles to 'stop'.

    Parameters
    ----------
    ctrl : Control
        A Control instance with an active, discrete motion command.
    max_steps : int, default=20000
        A safety limit to avoid an infinite loop if a test's motion
        command never naturally reaches 'stop' (e.g. a continuous move).

    Returns
    -------
    int
        The number of steps actually taken.

    """

    steps = 0
    step(ctrl)
    steps += 1
    while ctrl._motion_state != 'stop' and steps < max_steps:
        step(ctrl)
        steps += 1
    return steps
