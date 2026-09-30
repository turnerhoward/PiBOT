"""

On-robot check for the refactored PiBOT library (MicroPython, RP2040).

The desktop test suite verifies the motion LOGIC, but it cannot tell
whether the library is compatible with your MicroPython build, or
whether the changes cost anything in the 10 ms control loop. This
script measures those things on the actual board.

How to run
----------
1. Copy the library files (and this script) to the Maker Pi RP2040.
2. Put the robot ON BLOCKS so the wheels spin freely in the air. The
   motion section drives the wheels through continuous forward,
   steering, rotation, and arc moves, and short discrete moves.
3. In Thonny, open this file and choose Run > Run current script.
4. Copy EVERYTHING printed in the Shell between the two REPORT lines
   and share it.
5. Soft-reset (Ctrl+D) afterwards; the timing instrumentation stays
   attached to the control object until then.

Comparing with the original library
-----------------------------------
The control-loop timing section uses only calls that exist in both the
original and the refactored library (it detects which is installed), so
you can run this same script against each version and compare the
numbers directly. Sections that need the new classes (Move, Scan,
Sequence) are skipped automatically on the original.

Settings
--------
Turn a section off by setting its RUN_ flag to False. Set RUN_MOTION to
False to skip everything that moves the wheels (the loop-timing section
then only measures the idle loop).

"""

import gc
import sys
from utime import ticks_us, ticks_ms, ticks_diff, sleep_ms

# ---- settings -------------------------------------------------------
RUN_COMPAT = True        # MicroPython feature checks (no motion)
RUN_MICROBENCH = True    # memory/speed of tuple vs class vs slots vs Move
RUN_ANALYSIS = True      # lidar analysis timing on synthetic scans
RUN_MOTION = True        # control-loop timing while the wheels move
COUNTDOWN_S = 5          # seconds to step back before the wheels move
WARMUP_MS = 1200         # let each motion reach steady state
WINDOW_MS = 600          # measurement window per phase
# ---------------------------------------------------------------------

T_STEP_US = 10000        # nominal control loop step (matches T_STEP)


def mem_free():
    try:
        return gc.mem_free()
    except AttributeError:
        return 0


def line(text=''):
    print(text)


# =====================================================================
# Section 1: MicroPython compatibility checks
# =====================================================================

class _Plain6:
    def __init__(self, a, b, c, d, e, f):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.e = e
        self.f = f


class _Slots6:
    __slots__ = ('a', 'b', 'c', 'd', 'e', 'f')

    def __init__(self, a, b, c, d, e, f):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.e = e
        self.f = f


def check(name, fn):
    """Runs fn; PASS if it returns True/None, FAIL if False, ERR if it raises."""
    try:
        result = fn()
        tag = 'FAIL' if result is False else 'PASS'
        msg = ''
    except Exception as e:
        tag = 'ERR '
        msg = '  <- %s: %s' % (type(e).__name__, e)
    line('  %s  %s%s' % (tag, name, msg))
    return tag


def info(name, fn):
    """Runs fn and prints its value; for facts rather than pass/fail."""
    try:
        line('  INFO  %s: %s' % (name, fn()))
    except Exception as e:
        line('  INFO  %s: raised %s: %s' % (name, type(e).__name__, e))


def run_compat(robot, Move, Scan):
    line('--- 1. MicroPython compatibility ---')
    tags = []

    # __slots__ behaviour (the library uses it on Move; earlier I assumed
    # it saves memory in MicroPython, which the microbenchmarks verify)
    def slots_class_works():
        o = _Slots6(1, 2, 3, 4, 5, 6)
        return o.a == 1 and o.f == 6
    tags.append(check('class with __slots__ can be defined and used',
                      slots_class_works))

    def slots_attr_visible():
        return tuple(_Slots6.__slots__) == ('a', 'b', 'c', 'd', 'e', 'f')
    tags.append(check('__slots__ is readable as a class attribute '
                      '(Move.__eq__ iterates it)', slots_attr_visible))

    def slots_enforced():
        o = _Slots6(1, 2, 3, 4, 5, 6)
        try:
            o.not_a_slot = 1
        except AttributeError:
            return 'yes (AttributeError raised)'
        return 'NO (undeclared attribute accepted; __slots__ is ignored)'
    info('__slots__ enforced on assignment', slots_enforced)

    # locks used by tracking (with-statement form appears in pibot.py)
    def lock_with():
        lock = robot._control._tracking_lock
        with lock:
            pass
        return True
    tags.append(check('`with lock:` on _tracking_lock', lock_with))

    def lock_manual():
        lock = robot._control._tracking_lock
        lock.acquire()
        lock.release()
        return not lock.locked()
    tags.append(check('lock.acquire()/release()/locked()', lock_manual))

    def position_property():
        robot.position = [1.0, 2.0]
        p = robot.position
        robot.position = [0, 0]
        return p == [1.0, 2.0]
    tags.append(check('position getter/setter (lock + copy)',
                      position_property))

    if Move is not None:
        def move_basic():
            a = Move('linear', protect=True, speed=10, distance=5)
            b = a.copy()
            return a == b and a is not b and a.speed == 10
        tags.append(check('Move construct / copy / ==', move_basic))

        def move_ne():
            a = Move('linear', speed=10)
            b = Move('linear', speed=11)
            return (a != b) is True and (a != a.copy()) is False
        tags.append(check('Move != (falls back to __eq__?)', move_ne))

        def move_eq_other():
            return (Move('ready') == 5) is False
        tags.append(check('Move == non-Move returns False without error '
                          '(uses NotImplemented)', move_eq_other))

        def move_repr():
            return 'linear' in repr(Move('linear'))
        tags.append(check('Move.__repr__ with %r formatting', move_repr))

        def move_mutation():
            a = Move('linear', distance=5)
            b = a.copy()
            b.distance = 2
            return a.distance == 5 and b.distance == 2
        tags.append(check('Move.copy() is independent', move_mutation))

    if Scan is not None:
        def scan_valid():
            s = Scan([0.0, 1.0, 2.0], [5.0, 6.0, 7.0])
            return len(s) == 3 and list(s) == [(0.0, 5.0), (1.0, 6.0),
                                                (2.0, 7.0)]
        tags.append(check('Scan valid input, len(), zip iteration',
                          scan_valid))

        def scan_invalid():
            return (len(Scan([1.0], [1.0, 2.0])) == 0
                    and len(Scan(5, [1.0])) == 0
                    and len(Scan([1.0, 'a'], [1.0, 2.0])) == 0)
        tags.append(check('Scan rejects bad input (prints, empty Scan)',
                          scan_invalid))

        def scan_repr():
            return 'Scan(3 points)' == repr(Scan([1, 2, 3], [1, 2, 3]))
        tags.append(check('Scan.__repr__', scan_repr))

        def scan_public_api():
            s = Scan([0.0, 1.0, 2.0], [5.0, 9.0, 6.0])
            return (robot.max_distance(s) == (1.0, 9.0)
                    and robot.min_distance(s) == (0.0, 5.0))
        tags.append(check('max_distance()/min_distance() accept a Scan',
                          scan_public_api))

        def scan_rejects_old_style():
            return robot.max_distance([1.0, 2.0, 3.0]) is None
        tags.append(check('analysis methods reject a non-Scan',
                          scan_rejects_old_style))

        def xy_point():
            x, y = robot.convert_point_to_xy(0.0, 10.0)
            return abs(x - 10.0) < 0.2 and abs(y) < 0.2
        tags.append(check('convert_point_to_xy() (single-point Scan)',
                          xy_point))

    if hasattr(robot.move, 'sequence'):
        def seq_build():
            seq = (robot.move.sequence().forward(10, 5)
                   .rotate_left(90, 90).arc_forward(10, 15, arc_angle=45))
            return len(seq._moves) == 3 and all(m.protect
                                                for m in seq._moves)
        tags.append(check('Sequence builds and chains (not run)',
                          seq_build))

    line('  compat summary: %d checks, %d not PASS' %
         (len(tags), len([t for t in tags if t != 'PASS'])))
    line()
    return tags


# =====================================================================
# Section 2: object micro-benchmarks
# =====================================================================

def _bytes_per_object(make, n=200):
    """Heap bytes per object, with the holding list's own cost removed.

    Never disables gc (see the note in run_analysis()): a control loop
    is running concurrently on the other core for the lifetime of the
    PiBOT object, and disabling gc anywhere near it risks the same
    MemoryError-in-the-other-thread failure that this comment is here
    to warn against repeating.

    """

    gc.collect()
    m0 = mem_free()
    base = [0 for i in range(n)]
    gc.collect()
    m1 = mem_free()
    keep = [make(i) for i in range(n)]
    gc.collect()
    m2 = mem_free()
    list_cost = m0 - m1
    obj_cost = (m1 - m2) - list_cost
    del base, keep
    gc.collect()
    return obj_cost / n


def _create_us(make, n=300):
    t0 = ticks_us()
    for i in range(n):
        make(i)
    return ticks_diff(ticks_us(), t0) / n


def run_microbench(Move):
    line('--- 2. Object micro-benchmarks (this MicroPython build) ---')
    makers = [
        ('tuple(6)', lambda i: (i, 2, 3, 4, 5, 6)),
        ('list(6)', lambda i: [i, 2, 3, 4, 5, 6]),
        ('plain class, 6 attrs', lambda i: _Plain6(i, 2, 3, 4, 5, 6)),
        ('__slots__ class, 6 attrs', lambda i: _Slots6(i, 2, 3, 4, 5, 6)),
    ]
    if Move is not None:
        makers.append(('Move (9 fields, slots)',
                       lambda i: Move('linear', True, 10, i)))
    line('  %-26s %10s %12s' % ('type', 'bytes/obj', 'create (us)'))
    for name, make in makers:
        try:
            b = _bytes_per_object(make)
            c = _create_us(make)
            line('  %-26s %10.1f %12.1f' % (name, b, c))
        except Exception as e:
            line('  %-26s raised %s: %s' % (name, type(e).__name__, e))

    # attribute access vs index
    n = 3000
    t = (1, 2, 3, 4, 5, 6)
    lst = [1, 2, 3, 4, 5, 6]
    p = _Plain6(1, 2, 3, 4, 5, 6)
    s = _Slots6(1, 2, 3, 4, 5, 6)
    x = 0
    t0 = ticks_us()
    for i in range(n):
        pass
    base = ticks_diff(ticks_us(), t0)

    def timed(label, body):
        t0 = ticks_us()
        body()
        el = ticks_diff(ticks_us(), t0) - base
        line('  %-26s %8.2f us per 1000 reads' % (label, el * 1000 / n))

    def r_tuple():
        for i in range(n):
            x = t[3]

    def r_list():
        for i in range(n):
            x = lst[3]

    def r_plain():
        for i in range(n):
            x = p.d

    def r_slots():
        for i in range(n):
            x = s.d
    line()
    timed('read tuple[3]', r_tuple)
    timed('read list[3]', r_list)
    timed('read plain.d', r_plain)
    timed('read slots.d', r_slots)
    if Move is not None:
        m = Move('linear', True, 10, 5)

        def r_move():
            for i in range(n):
                x = m.kind
        timed('read Move.kind', r_move)

        m2 = m.copy()

        def eq_move():
            for i in range(n):
                x = m == m2
        timed('Move == Move', eq_move)

        def copy_move():
            for i in range(n // 10):
                x = m.copy()
        t0 = ticks_us()
        copy_move()
        el = ticks_diff(ticks_us(), t0)
        line('  %-26s %8.2f us per copy' % ('Move.copy()', el / (n // 10)))
    line()


# =====================================================================
# Section 3: lidar analysis timing on synthetic scans
# =====================================================================

def _box_scan():
    angle = [float(a) for a in range(-40, 41)]
    dist = [30.0 if -10 <= a <= 10 else 100.0 for a in angle]
    return angle, dist


def _room_scan():
    # matches the point count a real 360 degree scan() produces at the
    # default 2.5 degree increment, rather than a denser synthetic one
    from math import cos, sin, radians
    angle, dist = [], []
    a = -180.0
    while a < 180.0:
        c = abs(cos(radians(a)))
        s = abs(sin(radians(a)))
        dist.append(round(50 / max(c, s), 1))
        angle.append(a)
        a += 2.5
    return angle, dist


def run_analysis(robot, Scan):
    line('--- 3. Lidar analysis timing on synthetic scans ---')
    if Scan is None:
        line('  (original library: analysis methods take two lists)')
    box = _box_scan()
    room = _room_scan()

    def arg(pair):
        # the refactored library takes a Scan; the original took two lists
        return (Scan(pair[0], pair[1]),) if Scan else (pair[0], pair[1])

    jobs = [
        ('max_distance (360 pts)', robot.max_distance, room),
        ('min_distance (360 pts)', robot.min_distance, room),
        ('convert_to_xy (360 pts)', robot.convert_to_xy, room),
        ('center_point (360 pts)', robot.center_point, room),
        ('centroid (360 pts)', robot.centroid, room),
        ('find_corners (360 pts)', robot.find_corners, room),
        ('detect_objects (81 pts)', robot.detect_objects, box),
        ('detect_objects (360 pts)', robot.detect_objects, room),
    ]
    line('  %-28s %10s %12s' % ('method', 'ms', 'retained B'))
    line('  (retained = still allocated after a gc.collect() once the call')
    line('   returns; this deliberately never disables gc, since a control')
    line('   loop is running concurrently on the other core and doing so')
    line('   caused a MemoryError there during an earlier test run)')
    results = {}
    for name, fn, data in jobs:
        try:
            a = arg(data)
            gc.collect()
            m0 = mem_free()
            t0 = ticks_us()
            result = fn(*a)
            el = ticks_diff(ticks_us(), t0)
            gc.collect()
            m1 = mem_free()
            results[name] = result
            line('  %-28s %10.2f %12d' % (name, el / 1000, m0 - m1))
        except Exception as e:
            line('  %-28s raised %s: %s' % (name, type(e).__name__, e))
        # free this job's result and collect before the next one, so
        # nothing accumulates across the 8 jobs
        result = None
        gc.collect()
        if mem_free() < 60000:
            line('  low heap (%d bytes free); stopping section 3 early '
                 'to protect the control loop thread' % mem_free())
            break

    # numeric results on the board's float type vs desktop reference
    line()
    line('  results (desktop reference in brackets):')
    try:
        oa, od, ow = results['detect_objects (81 pts)']
        line('    box object: angle=%s dist=%s width=%s  [-0.5, 30.0, 8.9]'
             % (oa, od, ow))
    except Exception as e:
        line('    box object: unavailable (%s)' % e)
    try:
        cx, cy = results['centroid (360 pts)']
        line('    room centroid: (%s, %s)  [~(0, 0)]' % (cx, cy))
        ca, cd = results['find_corners (360 pts)']
        line('    room corners found: %d  [4]' % len(ca))
    except Exception as e:
        line('    room results: unavailable (%s)' % e)
    line()


# =====================================================================
# Section 4: control-loop timing and allocation under real motion
# =====================================================================

_STATS = {}                # method name -> [count, total_us, max_us]
_P = [None, 0, 0, 0, 0, 0, 0, 10**9]
# _P = [last_start, n, total, max, n>11ms, n>15ms, n>25ms, min]


def _instrument(ctrl, name):
    if not hasattr(ctrl, name):
        return False
    orig = getattr(ctrl, name)
    st = [0, 0, 0]
    _STATS[name] = st

    def wrapper():
        t0 = ticks_us()
        orig()
        d = ticks_diff(ticks_us(), t0)
        st[0] += 1
        st[1] += d
        if d > st[2]:
            st[2] = d
    setattr(ctrl, name, wrapper)
    return True


def _instrument_action(ctrl):
    """Wraps _control_action (called once per tick) to time the period."""
    orig = ctrl._control_action
    st = [0, 0, 0]
    _STATS['_control_action'] = st
    P = _P

    def wrapper():
        t0 = ticks_us()
        orig()
        d = ticks_diff(ticks_us(), t0)
        st[0] += 1
        st[1] += d
        if d > st[2]:
            st[2] = d
        if P[0] is not None:
            p = ticks_diff(t0, P[0])
            P[1] += 1
            P[2] += p
            if p > P[3]:
                P[3] = p
            if p < P[7]:
                P[7] = p
            if p > 11000:
                P[4] += 1
            if p > 15000:
                P[5] += 1
            if p > 25000:
                P[6] += 1
        P[0] = t0
    ctrl._control_action = wrapper


def _reset_stats():
    for st in _STATS.values():
        st[0] = st[1] = st[2] = 0
    _P[0] = None
    for i in range(1, 7):
        _P[i] = 0
    _P[7] = 10**9


def _calibrate():
    """Cost of the timing wrapper itself, so it can be subtracted."""
    def noop():
        pass
    st = [0, 0, 0]

    def wrapper():
        t0 = ticks_us()
        noop()
        d = ticks_diff(ticks_us(), t0)
        st[0] += 1
        st[1] += d
        if d > st[2]:
            st[2] = d
    t0 = ticks_us()
    for i in range(500):
        wrapper()
    return ticks_diff(ticks_us(), t0) / 500


def _stat_str(name):
    st = _STATS.get(name)
    if not st or st[0] == 0:
        return '-'
    return '%d calls, mean %.0f us, max %d us' % (st[0], st[1] / st[0],
                                                  st[2])


def _report_phase(label):
    line('  [%s]' % label)
    n = _P[1]
    if n:
        line('    period: mean %.2f ms, min %.2f, max %.2f | >11ms: %d, '
             '>15ms: %d, >25ms: %d (of %d)' %
             (_P[2] / n / 1000, _P[7] / 1000, _P[3] / 1000,
              _P[4], _P[5], _P[6], n))
    for name in ('_motion_control', '_forward_control', '_rotate_control',
                 '_arc_control', '_steer_control', '_update_state',
                 '_control_action'):
        if name in _STATS and _STATS[name][0]:
            line('    %-17s %s' % (name, _stat_str(name)))


def _measure(ctrl, label, window_ms):
    # deliberately never disables gc here: the control loop runs
    # concurrently on the other core for the whole window, and an
    # earlier version of this script that disabled gc across a window
    # like this caused a MemoryError in that thread (not this one),
    # which silently killed it and hung the next motion call forever.
    # A GC pause during the window would still show up as a spike in
    # the period timing below, so that risk-free signal is used instead
    # of a direct allocation count.
    gc.collect()
    _reset_stats()
    sleep_ms(window_ms)
    _report_phase(label)


def _wait_not_busy(robot, timeout_ms=15000):
    t0 = ticks_ms()
    while robot.busy and ticks_diff(ticks_ms(), t0) < timeout_ms:
        sleep_ms(10)


def _settle(robot):
    robot.move.pause()
    sleep_ms(300)


def run_motion(robot):
    line('--- 4. Control-loop timing and allocation under motion ---')
    ctrl = robot._control
    move = robot.move
    line('  library variant: %s' %
         ('refactored (_motion_control)' if hasattr(ctrl, '_motion_control')
          else 'original (separate _*_control methods)'))
    wrapper_us = _calibrate()
    line('  timing-wrapper overhead: ~%.0f us per instrumented call '
         '(already included in the figures below)' % wrapper_us)
    line('  nominal loop step: %d us' % T_STEP_US)
    for name in ('_motion_control', '_forward_control', '_rotate_control',
                 '_arc_control', '_steer_control', '_update_state'):
        _instrument(ctrl, name)
    _instrument_action(ctrl)
    line()

    line('  Wheels will move. Robot must be ON BLOCKS. Starting in %d s...'
         % COUNTDOWN_S)
    for s in range(COUNTDOWN_S, 0, -1):
        line('    %d' % s)
        sleep_ms(1000)

    try:
        _measure(ctrl, 'idle (no motion)', WINDOW_MS)

        move.forward(15)
        sleep_ms(WARMUP_MS)
        _measure(ctrl, 'linear, continuous 15 cm/s', WINDOW_MS)

        move.steer_left(15)
        sleep_ms(WARMUP_MS // 2)
        _measure(ctrl, 'steer, continuous r=15 cm', WINDOW_MS)

        ctrl._pause()
        _measure(ctrl, 'pause ramp-down (from steer)', 250)
        _settle(robot)

        move.rotate_left(ang_speed=90)
        sleep_ms(WARMUP_MS)
        _measure(ctrl, 'rotate, continuous 90 deg/s', WINDOW_MS)
        ctrl._pause()
        _measure(ctrl, 'pause ramp-down (from rotate)', 250)
        _settle(robot)

        move.arc_forward(15, 20)
        sleep_ms(WARMUP_MS)
        _measure(ctrl, 'arc, continuous 15 cm/s r=20 cm', WINDOW_MS)
        ctrl._pause()
        _measure(ctrl, 'pause ramp-down (from arc)', 250)
        _settle(robot)

        # discrete protected moves: many state transitions in a row
        gc.collect()
        _reset_stats()
        for i in range(3):
            move.forward(20, 10, protect=True)
            _wait_not_busy(robot)
            move.rotate_left(90, protect=True)
            _wait_not_busy(robot)
        _report_phase('discrete moves x6 (forward 10 cm, rotate 90)')

        if hasattr(move, 'sequence'):
            gc.collect()
            _reset_stats()
            (move.sequence().forward(15, 15).rotate_left(90, 90)
             .arc_forward(10, 15, arc_angle=90).reverse(15, 15).run())
            _wait_not_busy(robot)
            _report_phase('Sequence (forward, rotate, arc, reverse)')
    finally:
        gc.enable()
        try:
            move.pause()
        except Exception:
            pass
    line()


# =====================================================================
# main
# =====================================================================

def main(robot=None):
    line('===== PIBOT HARDWARE CHECK REPORT =====')
    line('python: %s' % (sys.version,))
    try:
        line('implementation: %s' % (sys.implementation,))
    except AttributeError:
        pass
    try:
        import machine
        line('cpu freq: %d MHz' % (machine.freq() // 1000000))
    except Exception:
        pass
    gc.collect()
    line('heap free at start: %d bytes' % mem_free())

    if robot is None:
        gc.collect()
        m0 = mem_free()
        from pibot import PiBOT
        m1 = mem_free()
        robot = PiBOT()
        m2 = mem_free()
        line('heap used by importing pibot: %d bytes' % (m0 - m1))
        line('heap used by creating PiBOT(): %d bytes' % (m1 - m2))
        gc.collect()
        line('heap free after import + PiBOT() + collect: %d bytes'
             % mem_free())
    try:
        from commands import Move
    except ImportError:
        Move = None
    try:
        from pibot import Scan
    except ImportError:
        Scan = None
    line('library: %s' % ('refactored (Move/Scan present)'
                          if Move is not None else 'original'))
    line()

    if RUN_COMPAT:
        run_compat(robot, Move, Scan)
    if RUN_MICROBENCH:
        run_microbench(Move)
    if RUN_ANALYSIS:
        run_analysis(robot, Scan)
    if RUN_MOTION:
        run_motion(robot)
    gc.collect()
    line('heap free at end: %d bytes' % mem_free())
    line('===== END REPORT =====')


if __name__ == '__main__':
    main()
