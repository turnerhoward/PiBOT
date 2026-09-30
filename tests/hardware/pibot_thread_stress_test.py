"""

Stress test for two specific MicroPython core1 (_thread) bug patterns
found in the MicroPython issue tracker, reported against versions after
v1.21.0 and relevant to this library because Control runs its motion
loop as a permanent second-core thread for the life of the program.

Part A: repeated thread restart within one power-on session.
    PiBOT.reset() stops the control thread and starts a new one, in
    place, without a reboot. This is exactly the pattern GitHub issue
    "V1.22 creating new thread leads to hangup" describes, and it is
    also something a student's code might do many times per session
    (e.g. calling .reset() between test runs). This part runs
    entirely within one script execution -- no reset needed.

Part B: a soft reset (Ctrl+D) while a thread is running, followed
    immediately by a file operation. This reproduces GitHub issue
    #15192 ("RP2 Hanging on file write after soft reboot when a
    thread was started") as closely as a script run from Thonny can.
    Because a soft reset restarts the interpreter, this part cannot
    complete in a single run: it leaves a marker file, starts a
    thread, and calls machine.soft_reset(). After the board comes
    back, re-run this same script (or see "How to run" below) and it
    will detect the marker and finish the test.

How to run
----------
1. Copy this file to the board.
2. Run it once from Thonny (Run > Run current script). Part A runs
   immediately and prints its results. Part B then starts a thread
   and triggers a soft reset -- you'll see "MPY: soft reboot" in the
   Shell, which is expected, not a crash.
3. Run the SAME script again (Thonny keeps it open; just Run again).
   It detects the leftover marker file and completes Part B.
4. If the board becomes unresponsive at any point and needs Ctrl+D to
   recover, that itself is the answer for this MicroPython version --
   note where it happened and share this output.

If manually re-running after the reset is inconvenient (e.g. testing
unattended), an alternative is to temporarily save this file as
main.py on the board -- it then re-runs itself automatically after
the reset. Restore your normal main.py afterward if you do this.

"""

import os
import gc
from utime import ticks_ms, ticks_diff, sleep_ms

MARKER = 'pibot_stress_marker.tmp'
WRITE_TEST_FILE = 'pibot_stress_write_test.tmp'


def line(text=''):
    print(text)


# =====================================================================
# Part A: repeated thread restart within one session (PiBOT.reset())
# =====================================================================

def run_part_a(robot, n=15, move_cm=2, move_speed=10):
    line('--- Part A: repeated thread restart (robot.reset() x%d) ---' % n)
    line('  Robot will make %d short (%g cm) moves. Keep it on blocks or '
        'in clear space.' % (n, move_cm))
    for s in range(3, 0, -1):
        line('    starting in %d...' % s)
        sleep_ms(1000)

    fails = 0
    times = []
    for i in range(n):
        t0 = ticks_ms()
        try:
            robot.reset()
            # confirm the new thread is actually alive and can drive a
            # real move through to completion, not just that the
            # object was reconstructed without raising
            robot.move.forward(move_speed, move_cm, protect=True)
            wait_t0 = ticks_ms()
            while robot.busy and ticks_diff(ticks_ms(), wait_t0) < 5000:
                sleep_ms(10)
            if robot.busy:
                raise RuntimeError('robot.busy never cleared within 5 s '
                                   'of commanding a %g cm move' % move_cm)
            if robot._control._motion_state != 'stop':
                raise RuntimeError("motion_state is %r, expected 'stop'"
                                   % robot._control._motion_state)
        except Exception as e:
            fails += 1
            line('  iteration %2d: FAIL  %s: %s' % (i, type(e).__name__, e))
            continue
        dt = ticks_diff(ticks_ms(), t0)
        times.append(dt)
        line('  iteration %2d: OK    reset()+move took %d ms' % (i, dt))

    line()
    line('  Part A result: %d/%d iterations succeeded' % (n - fails, n))
    if times:
        line('  reset()+move time: mean %.0f ms, min %d ms, max %d ms' %
             (sum(times) / len(times), min(times), max(times)))
    if fails:
        line('  ** at least one iteration failed; this matches the '
            '"repeated thread restart" bug pattern from the MicroPython '
            'issue tracker (see this script\'s module docstring) **')
    line()
    try:
        robot.move.pause()
    except Exception:
        pass
    return fails == 0


# =====================================================================
# Part B: soft reset while a thread is running, then an immediate file
# operation (reproduces MicroPython issue #15192)
# =====================================================================

def _marker_exists():
    try:
        os.stat(MARKER)
        return True
    except OSError:
        return False


def run_part_b():
    line('--- Part B: file write immediately after a soft reset while a '
        'thread was running ---')

    if _marker_exists():
        # second phase: we just came back from the soft reset triggered
        # below, with a thread having been running at the moment of reset
        line('  soft reset marker found -- board just came back from the '
            'reset triggered a moment ago')
        line('  attempting a file write now, immediately, matching the '
            'reported bug pattern...')
        t0 = ticks_ms()
        try:
            with open(WRITE_TEST_FILE, 'w') as f:
                f.write('pibot stress test\n')
            with open(WRITE_TEST_FILE, 'r') as f:
                f.read()
            os.remove(WRITE_TEST_FILE)
            dt = ticks_diff(ticks_ms(), t0)
            line('  Part B result: PASS -- file write/read/remove '
                'completed in %d ms' % dt)
            passed = True
        except Exception as e:
            line('  Part B result: FAIL -- file write raised %s: %s' %
                (type(e).__name__, e))
            passed = False
        try:
            os.remove(MARKER)
        except OSError:
            pass
        line()
        return passed
    else:
        # first phase: start a thread, leave a marker so the next run
        # knows to finish the test, then soft reset while it's running
        line('  starting a background thread on core1, then triggering a '
            'soft reset in 2 seconds...')
        line('  ("MPY: soft reboot" appearing next is expected, not a '
            'crash)')
        line('  re-run this same script after the board comes back to '
            'complete Part B')
        with open(MARKER, 'w') as f:
            f.write('1')
        sleep_ms(2000)

        import _thread

        def _spin():
            while True:
                pass
        _thread.start_new_thread(_spin, ())
        sleep_ms(300)  # let the thread actually start running

        import machine
        machine.soft_reset()
        # unreachable: soft_reset() does not return
        return None


# =====================================================================
# main
# =====================================================================

def main():
    line('===== PIBOT THREAD STRESS TEST =====')
    gc.collect()
    line('heap free at start: %d bytes' % gc.mem_free())
    line()

    # Part B's second phase takes priority: if the marker is present, the
    # board just came back from a soft reset triggered by a previous run,
    # so finish that immediately before anything else touches the thread
    # subsystem or the heap.
    if _marker_exists():
        run_part_b()
        line('===== END (Part B completed; Part A was already run in a '
            'previous invocation) =====')
        return

    from pibot import PiBOT
    robot = PiBOT()
    a_ok = run_part_a(robot)
    line('===== Part A done; starting Part B (will trigger a soft reset) '
        '=====')
    line()
    run_part_b()
    # not reached in the first invocation: run_part_b() ends in
    # machine.soft_reset() when no marker exists yet


if __name__ == '__main__':
    main()
