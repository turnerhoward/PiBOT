"""

This module defines the Move class, which represents a single motion
request passed between the Motion class (running on the main core) and
the Control class (running the control loop on the second core).

In earlier versions of this library, a motion request was encoded as a
plain list, e.g. ['linear', True, 10, 5] for a protected forward move
at 10 cm/s for 5 cm, or ['arc', False, 15, 20, 30, 'clockwise'] for an
unprotected arc. The meaning of each position in the list was different
for each kind of move, and every place that handled these lists (the
Motion class, the Control class, and the internal resume/pause logic)
had to know the correct position for the value it needed. That made the
code difficult to read and risky to extend.

The Move class replaces that encoding with named attributes. A move is
still created and handed off in essentially the same way, but a rotate
command now looks like:

>>> Move('rotate', protect=True, ang_speed=1.5, angle=3.14)

instead of:

>>> ['rotate', True, 1.5, 3.14]

Every attribute is present on every Move object, but only the ones that
apply to a given "kind" are meaningful; see the Parameters section
below for which attributes are used by each kind.

"""


class Move:
    """
    Represents a single motion request understood by the control loop.

    ...

    Parameters
    ----------
    kind : {'ready', 'pause', 'linear', 'rotate', 'arc', 'steer'}
        The type of motion this object describes. 'ready' means no
        command is pending, and 'pause' requests a smooth stop of
        whatever is currently active.
    protect : bool, default=False
        Prevents interruption by another motion command. Used by
        'linear', 'rotate', 'arc', and 'steer'.
    speed : int or float, default=0
        Linear speed in cm/s. Used by 'linear', 'arc', and 'steer'.
    distance : int or float, default=0
        Linear distance in cm. Used by 'linear'.
    ang_speed : int or float, default=0
        Angular speed in rad/s. Used by 'rotate'.
    angle : int or float, default=0
        Rotation angle in rad. Used by 'rotate'.
    radius : int or float, default=0
        Arc radius in cm. Used by 'arc' and 'steer'.
    arc_length : int or float, default=0
        Arc length in cm. Used by 'arc' and 'steer'.
    sense : {'counterclockwise', 'clockwise'}, optional
        Rotation sense of the arc. Used by 'arc' and 'steer'.

    Notes
    -----
    Move objects are created by the Motion class in response to a user
    call (e.g. .forward(), .rotate_left()) and are read by the Control
    class to update the active motion state. A Move is treated as a
    simple data record; the .copy() method returns an independent copy
    so that adjusting one field (for example, reducing "distance" to
    the remaining distance when resuming a paused move) never affects
    the original object.

    """

    __slots__ = ('kind', 'protect', 'speed', 'distance', 'ang_speed',
                'angle', 'radius', 'arc_length', 'sense')

    def __init__(self, kind, protect=False, speed=0, distance=0,
                ang_speed=0, angle=0, radius=0, arc_length=0, sense=None):
        """Creates a Move with the given kind and field values."""

        self.kind = kind
        self.protect = protect
        self.speed = speed
        self.distance = distance
        self.ang_speed = ang_speed
        self.angle = angle
        self.radius = radius
        self.arc_length = arc_length
        self.sense = sense

    def copy(self):
        """Returns a new, independent Move with the same field values."""

        return Move(self.kind, self.protect, self.speed, self.distance,
                    self.ang_speed, self.angle, self.radius,
                    self.arc_length, self.sense)

    def __eq__(self, other):
        """Compares two Move objects for identical field values.

        Used by the Motion class to detect and ignore repeat calls
        with the same arguments as the currently active move.

        """

        if not isinstance(other, Move):
            return NotImplemented
        return all(getattr(self, field) == getattr(other, field)
                  for field in self.__slots__)

    def __repr__(self):
        """Returns a readable representation for debugging."""

        return ('Move(kind=%r, protect=%r, speed=%r, distance=%r, '
                'ang_speed=%r, angle=%r, radius=%r, arc_length=%r, '
                'sense=%r)' % (self.kind, self.protect, self.speed,
                              self.distance, self.ang_speed, self.angle,
                              self.radius, self.arc_length, self.sense))
