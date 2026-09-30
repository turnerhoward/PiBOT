# PiBOT motion-logic regression tests

These tests check the *logic* in `control.py`, `motion.py`, and
`commands.py` — the pause/resume state machine, `Sequence` chaining,
and the (speed, curvature) kinematics model — on a regular desktop
Python interpreter, without needing the Maker Pi RP2040 board.

## Why these exist

`control.py` imports several MicroPython-only modules (`machine`,
`rp2`, `utime`, `_thread`, `neopixel`) that don't exist on a laptop, so
the library can't normally be imported or tested outside MicroPython.
This test suite works around that with two things:

- **`stubs/`** — minimal, desktop-only stand-ins for those modules,
  just enough to satisfy imports and `Control`'s initialization.
- **`robot_test_utils.py`** — replaces `Control._motors._read_angles()`
  with a "perfect tracking" stub that always reports exactly the wheel
  angle the control loop just commanded (no PID lag, no motor
  dynamics, no real encoder noise), and replaces
  `_thread.start_new_thread` so tests drive the control loop
  themselves, one step at a time, instead of racing against a real
  background thread.

This means these tests check "does the control loop *compute* the
right thing at each step," not "does the robot correctly *move* on
carpet with a low battery." **They're a fast, deterministic first check
before deploying to hardware, not a replacement for testing on the
actual robot.**

## Running the tests

From the top level of the library:

    python3 tests/run_tests.py

or, using unittest's own discovery directly:

    python3 -m unittest discover -s tests -p "test_*.py" -v

## What's covered

- **`test_motion_state_machine.py`** — pause/resume, a rotation
  interrupting and then auto-resuming another move, `Sequence`
  building/chaining/validation, and `Move` equality/copy semantics.
- **`test_curvature.py`** — arc wheel geometry against the exact
  expected values for a given radius, CW/CCW mirroring, curvature not
  leaking from one move into the next, and steer changing curvature
  without a speed dip.
- **`test_tracking_and_scan.py`** — the position/heading dead-reckoning
  math, and the specific assumptions `PiBOT.scan()` makes about the
  motion state machine (monotonic heading during a sweep, `protect`
  clearing when the sweep finishes).
- **`test_lidar_analysis.py`** — the `Scan` class (validation, iteration,
  independence), the "is a `Scan` with enough points" check shared by
  every lidar analysis method, `convert_to_xy()`/`convert_point_to_xy()`,
  the `LIDAR_OUT_OF_RANGE` constant used by `max_distance()`,
  `_find_objects()` returning `_DetectedObject` instances, and
  `center_point()`/`centroid()`/`find_corners()` on a simulated square room.

`PiBOT.scan()` itself is not in the automated suite: it busy-waits on the
control loop's state, so testing it needs a real-time-paced simulation,
which is timing-sensitive and would make the suite flaky.

## On-robot check (not part of the automated suite)

`hardware/pibot_hw_check.py` runs on the Maker Pi RP2040 itself (run it from
Thonny with the robot on blocks). It checks MicroPython compatibility of the
constructs the library uses, benchmarks tuple vs. class vs. `__slots__` vs.
`Move` memory and speed on the real interpreter, times the lidar analysis
methods on synthetic scans, and measures control-loop period, per-call time,
and heap allocation per tick under continuous forward, steer, rotate, and arc
motion, discrete moves, and a `Sequence`. It detects whether the original or
refactored library is installed, so the same script can be run against both
for a direct comparison.

## Adding a test

If you change `control.py`, `motion.py`, or `commands.py`, run this
suite before testing on hardware. If you add a new kind of motion or
change how an existing one is computed, add a test here first — most
existing tests are a good template: build a `Move`, step the control
loop with `step()` or `run_until_stopped()`, and assert on the
resulting `Control` state.
