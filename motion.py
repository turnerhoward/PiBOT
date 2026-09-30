"""

The Motion class defined in this module contains several methods to move
the PiBOT. The path planning and motor control needed to enable smooth
and accurate motion are fairly complex and involve calculating the
desired wheel rotation for each time step, measuring the actual rotation
with encoders on each motor, and computing the required motor effort
with a closed-loop PID algorithm. Those tasks are encapsulated in the
Control class, which does not have any user-accessible attributes or
methods and works behind the scenes on the second core of the RP2040
microcontroller to maintain a fast (approximately 10 ms) time step. The
user-accessible methods in the Motion class must have access to the
private attributes of the Control class to operate the PiBOT. Therefore,
when the PiBOT object is created, an instance of Control called
"_control" is passed into an instance of Motion called "move".

The "_control" object runs an infinite loop on the second core using the
_thread module. The loop checks for new events, manages motion, and
attempts to maintain a uniform time step of 10 ms. When a user calls one
of the methods of the "move" object from the main core, attributes are
set in the "_control" object, and the control loop running on the second
core takes action to handle the path planning and control.

As a result of this multi-threaded, event-driven approach to controlling
the PiBOT motion, it is possible to send many motion commands to the
control loop in quick succession such that earlier commands will not
finish before being replaced by later commands. For example, if a user
requests forward motion at 10 cm/s for a distance of 10 cm, that command
is received by the control loop and started immediately. The main core
is then ready to handle more commands. Meanwhile, the PiBOT has just
started accelerating and won't complete the requested move for about
another second. If the user then requests 90 degree rotation with
another motion command, the control loop immediately starts rotation.
From the user perspective, the forward motion appears to be skipped over
and all they see is rotation. In effect, the user motion calls are
non-blocking, meaning that they don't tie up and pause a program until
the motion is completed. The advantage to this approach is that while
motion is managed on the second core, the main core is free to take many
other actions like checking sensors.

Important
---------
To create a sequence of motion where later motion calls do not overwrite
the earlier ones, a means of waiting is required. A simple time delay
could be used but would require calculating the duration of each delay
and would eliminate the advantage of non-blocking motion that allows
other commands to run concurrently. Instead, the .sequence() method
returns a Sequence object used to describe a chain of discrete moves,
which run back-to-back once .run() is called. When one move finishes,
the control loop automatically starts the next one in the sequence. The
examples below in the Motion and Sequence classes demonstrate how to
build and run a sequence.

Note
----
By default, when a "move" method is called, the control loop stops any
active motion and starts the new request immediately. There is an
argument in several of the methods called "protect" that prevents active
motion from being interrupted by any subsequent motion calls. Every move
added to a Sequence is automatically protected, so that once a sequence
starts running, later queued moves cannot interrupt earlier ones before
they finish.

"""

import constants as cnst
from math import pi, degrees, sqrt, atan2
from commands import Move


# ---------------------------------------------------------------------
# Small, single-purpose argument validators shared by the Motion and
# Sequence classes. Each one checks a single argument and prints a
# message and returns False if it is invalid, or returns True if it is
# valid. Composing several small checks with "and" (as done throughout
# this module) keeps each motion method's validation short and makes it
# easy to see exactly which checks apply to which method, rather than
# routing every method through one large function that has to know
# about every argument used by every motion type.
# ---------------------------------------------------------------------

def _valid_speed(speed):
    """Checks that speed is numeric and within the allowed range."""

    if not isinstance(speed, (int, float)):
        print('Error: speed must be a numeric value')
        return False
    if speed < 0:
        print('Error: speed must be positive')
        return False
    if speed > cnst.SPD_MAX:
        print('Error: maximum speed of robot is: %.2f cm/s' %cnst.SPD_MAX)
        return False
    if 0 <= speed < cnst.SPD_MIN:
        print('Error: minimum speed of robot is: %.2f cm/s' %cnst.SPD_MIN)
        return False
    return True


def _valid_distance(distance, required=False):
    """Checks that distance is numeric and positive (or zero if allowed)."""

    if not isinstance(distance, (int, float)):
        print('Error: distance must be a numeric value')
        return False
    if distance != 0 and distance < 0:
        print('Error: distance must be positive')
        return False
    if required and distance == 0:
        print('Error: only discrete moves are allowed in a sequence, '
             + 'set a distance')
        return False
    return True


def _valid_ang_speed(ang_speed):
    """Checks that ang_speed is numeric and within the allowed range."""

    if not isinstance(ang_speed, (int, float)):
        print('Error: ang_speed must be a numeric value')
        return False
    if ang_speed < 0:
        print('Error: ang_speed must be positive')
        return False
    if ang_speed > cnst.ANG_SPD_MAX:
        print('Error: maximum ang_speed is: %.2f deg/s' %cnst.ANG_SPD_MAX)
        return False
    if ang_speed < cnst.ANG_SPD_MIN:
        print('Error: minimum ang_speed is: %.2f deg/s' %cnst.ANG_SPD_MIN)
        return False
    return True


def _valid_angle(angle, required=False):
    """Checks that angle is numeric and within the allowed range."""

    if not isinstance(angle, (int, float)):
        print('Error: angle must be a numeric value')
        return False
    if angle < 0:
        print('Error: angle must be positive')
        return False
    if angle > cnst.ANG_MAX:
        print('Error: maximum rotation angle is %d degrees' %cnst.ANG_MAX)
        return False
    if required and angle == 0:
        print('Error: only discrete moves are allowed in a sequence, '
             + 'set an angle')
        return False
    return True


def _valid_radius(radius):
    """Checks that radius is numeric and at least the minimum radius."""

    if not isinstance(radius, (int, float)):
        print('Error: radius must be a numeric value')
        return False
    if radius < cnst.MIN_RADIUS:
        print('Error: minimum radius is %.1f cm' %cnst.MIN_RADIUS)
        return False
    return True


def _valid_arc_size(arc_length, arc_angle, required=False):
    """Checks arc_length and arc_angle, which cannot both be set."""

    if not isinstance(arc_length, (int, float)):
        print('Error: arc_length must be a numeric value')
        return False
    if arc_length < 0:
        print('Error: arc_length must be positive')
        return False
    if not isinstance(arc_angle, (int, float)):
        print('Error: arc_angle must be a numeric value')
        return False
    if arc_angle < 0 or arc_angle > cnst.ANG_MAX:
        print('Error: arc_angle must be positive and <= %d degrees'
             %cnst.ANG_MAX)
        return False
    if arc_length != 0 and arc_angle != 0:
        print('Error: arc_length and arc_angle cannot both be set')
        return False
    if required and arc_length == 0 and arc_angle == 0:
        print('Error: only discrete moves are allowed in a sequence, set '
             + 'arc_length or arc_angle')
        return False
    return True


def _valid_sense(sense):
    """Checks that sense is 'clockwise' or 'counterclockwise'."""

    if sense != 'clockwise' and sense != 'counterclockwise':
        print("Error: sense must be 'clockwise' or 'counterclockwise'")
        return False
    return True


def _valid_bool(value, name):
    """Checks that value is a boolean True or False."""

    if value != True and value != False:
        print('Error: %s must be a boolean value True or False' %name)
        return False
    return True


def _valid_position(position):
    """Checks that position is a 2-element list or tuple of numbers."""

    if not isinstance(position, (list, tuple)):
        print('Error: desired position must be a tuple or list')
        return False
    if len(position) != 2:
        print('Error: position must have two elements (i.e., x and y)')
        return False
    if not all(isinstance(x, (int, float)) for x in position):
        print('Error: desired x-y coordinates must be numeric values')
        return False
    return True


def _valid_heading(heading):
    """Checks that heading is numeric and in the range -180 to 180."""

    if not isinstance(heading, (int, float)):
        print('Error: desired heading must be a numeric value')
        return False
    if not (-180 <= heading <= 180):
        print('Error: desired heading must be in the range -180 to 180')
        return False
    return True


def _valid_direction(direction):
    """Checks that direction is 'left', 'right', or None."""

    if direction is not None and direction != 'left' and direction != 'right':
        print("Error: rotation direction must be 'left', 'right', or None")
        return False
    return True


class Motion:
    """
    Generates user motion commands for PiBOT.

    ...

    Parameters
    ----------
    _control : object
        Instance of the Control class.

    Notes
    -----
    When creating an instance of the PiBOT class, an instance of the
    Control class is passed to this Motion class to gain access to the
    control attributes.

    """

    def __init__(self, _control):
        """Creates references to control attributes."""

        self._ctrl = _control

    def pause(self):
        """Smoothly pauses all motion by ramping down velocity.

        Note
        ----
        Calling .pause() halts any motion command, including protected
        motion and an active sequence.
        
        """

        self._ctrl._pause()
        # wait until the motion ramps down and the state is at pause
        while (self._ctrl._motion_state != 'pause'
               and self._ctrl._motion_curr.kind != 'ready'):
            continue

    def resume(self):
        """Smoothly resumes motion after a pause.
        
        Example
        -------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()
     
        Move 10 cm at 5 cm/s and protect the motion from interruptions
        by other motion commands. Pause after 1 second and then resume
        the previous motion after 1 more second. Notice that the
        .pause() method always halts motion, even protected moves.

        >>> robot.move.forward(5, 10, protect=True)
        >>> time.sleep(1)
        >>> robot.move.pause()
        >>> time.sleep(1)
        >>> robot.move.resume()
        
        """

        self._ctrl._resume()

    def forward(self, speed, distance=0, protect=False, reverse=False):
        """Commands a forward (or reverse) linear path with constant speed.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        distance : int or float, default=0
            The desired distance to travel in cm. The default of zero
            commands continuous motion.
        protect : bool, default=False
            Prevents interruption by another motion command.
        reverse : bool, default=False
            Reverses the direction.

        Notes
        -----
        Uses a linear velocity profile to approximate constant
        acceleration by ramping velocity up and down at the start and
        stop, which produces smoother motion with less wheel slippage.

        Examples
        --------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command continuous motion at 10 cm/s and pause after 2 seconds.

        >>> robot.move.forward(10)
        >>> time.sleep(2)
        >>> robot.move.pause()

        Move 10 cm at 15 cm/s and protect the motion from interruptions
        by other motion commands.

        >>> robot.move.forward(15, 10, protect=True)

        Build and run a sequence with the .sequence() method. Every move
        in a sequence is automatically protected so that later steps
        can't interrupt earlier ones.

        >>> robot.move.sequence().forward(5, 5).reverse(5, 5) \\
        ...     .forward(20, 10).reverse(20, 10).run()
        
        An unprotected move can be interrupted at any time with a new
        motion call. The control loop handles the smooth transitions.
        The following example calls forward motion and then reverse
        without stopping first.

        >>> robot.move.forward(10)
        >>> time.sleep(1)
        >>> robot.move.reverse(20, 10)

        """

        # check for valid arguments
        try:
            valid = (_valid_speed(speed) and _valid_distance(distance)
                    and _valid_bool(protect, 'protect')
                    and _valid_bool(reverse, 'reverse'))
        except:
            print('Error: invalid argument to .forward() method')
            return
        if not valid:
            return
        # do nothing if motion is currently protected
        if self._ctrl._protect:
            return
        # change signs for reverse direction
        if reverse:
            speed = -speed
            distance = -distance
        # assign the command to 'motion_call' to test for repeats
        motion_call = Move('linear', protect=protect, speed=speed,
                           distance=distance)
        # return without calling motion if a repeat is being requested
        if self._is_repeat_call(motion_call):
            return
        # check for running motion and ramp it down if needed
        if self._ctrl._motion_state not in ('stop', 'pause', 'steer'):
            self.pause()
        # store command in motion_curr attribute
        self._ctrl._motion_curr = motion_call
        # clear out any pending sequence when calling a new immediate move
        self._ctrl._motion_queue.clear()
        # ensure motion has started before returning
        while self._ctrl._motion_state != 'linear':
            continue

    def reverse(self, speed, distance=0, protect=False):
        """Commands a linear reverse path with constant speed.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        distance : int or float, default=0
            The desired distance to travel in cm. The default of zero
            commands continuous motion.
        protect : bool, default=False
            Prevents interruption by another motion command.

        Notes
        -----
        Reverse linear motion is generated using the .forward() method
        with the reverse parameter set to True.

        """

        self.forward(speed, distance, protect, reverse=True)

    def go_to_position(self, position, speed=30, protect=False):
        """Commands a straight-line move to the desired x-y position.

        Parameters
        ----------
        position : tuple or list with 2 elements
            The x-y coordinates of the desired position.
        speed : int or float, default=30
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        protect : bool, default=False
            Prevents interruption by another motion command.

        Notes
        -----
        Uses the current robot position and heading to calculate the
        heading and distance required to move the robot to the desired
        position. The .rotate_to_heading() method is used to orient the
        robot and the .forward() method then commands the motion to the
        final position. The robot's heading at the end of the move will
        remain along the direction of the path taken.

        Note
        ----
        This method cannot be added to a sequence because the
        calculation is based on the position and heading at the moment
        it is called, and likely not at the correct step in the
        sequence. Use the robot.moving or robot.busy properties to wait
        for other motion to finish and call this method at the time it
        is needed.
        
        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Get the current robot position.

        >>> robot.position
        [10.225, -25.7433]
        
        Move to the position (0, 0).
        
        >>> robot.move.go_to_position((0, 0))
        
        Wait for motion to stop and recheck the position.
        
        >>> robot.position
        [-0.023275, 0.061157]
        
        """

        # check for valid arguments
        try:
            valid = (_valid_position(position) and _valid_speed(speed)
                    and _valid_bool(protect, 'protect'))
        except:
            print('Error: invalid argument to .go_to_position() method')
            return
        if not valid:
            return
        # wait to avoid getting heading while _tracking() method is active
        with self._ctrl._tracking_lock:
            start_heading = self._ctrl._heading * (180/pi)
            start_position = self._ctrl._position.copy()
        # calculate distance to desired position
        distance = sqrt((position[0]-start_position[0])**2
                               + (position[1]-start_position[1])**2)
        # calculate angle of position vector
        angle = atan2((position[1]-start_position[1]),
                             (position[0]-start_position[0]))
        # convert angle to degrees
        angle = degrees(angle)
        # rotate toward position
        self.rotate_to_heading(angle)
        # wait for rotation to finish
        while self._ctrl._motion_state not in ('stop', 'pause'):
            continue
        # move forward to home position
        self.forward(speed, distance, protect=protect)

    def rotate_left(self, angle=0, ang_speed=cnst.ANG_SPD_MAX, protect=False,
                    right=False):
        """Commands counterclockwise rotation in place.

        Parameters
        ----------
        angle : int or float, default=0 for continuous, maximum 720
            The desired rotation angle in degrees.
        ang_speed : int or float, default=180 deg/s
            The desired angular speed. Range from 30 to 180 deg/s.
        protect : bool, default=False
            Prevents interruption by another motion command.
        right : bool, default=False
            Commands right (clockwise) rotation instead of left.

        Notes
        -----
        Uses a ramped angular velocity profile to approximate constant
        acceleration by ramping velocity up and down at the start and
        stop, which produces smoother motion with less wheel slippage.
        The maximum angular speed and acceleration are set by the
        constants ANG_SPD_MAX and ANG_ACC and should not be changed.

        Examples
        --------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Rotate right and then left back to starting position at default
        speed of 180 deg/s, which is also the maximum.

        >>> robot.move.rotate_left(45)
        >>> time.sleep(1)
        >>> robot.move.rotate_right(45)

        Move 10 cm at 15 cm/s, turn around at 30 deg/s, return to
        starting point, and turn around at maximum speed of 180 deg/s.
        This motion uses .sequence() to build and run a sequence.

        >>> robot.move.sequence() \\
        ...     .forward(15, 10) \\
        ...     .rotate_left(180, 30) \\
        ...     .forward(15, 10) \\
        ...     .rotate_right(180, 180) \\
        ...     .run()
        
        If discrete rotation is called while another unprotected move is
        in progress, the rotation temporarily interrupts the current
        motion and then resumes it after completing the rotation.

        >>> robot.move.forward(5, 20)
        >>> time.sleep(2)
        >>> robot.move.rotate_left(180)
        
        If continuous rotation interrupts another move, that move is
        canceled and cannot be resumed.
        
        >>> robot.move.forward(5)
        >>> time.sleep(2)
        >>> robot.move.rotate_left(ang_speed=90)
        >>> time.sleep(2)
        >>> robot.move.pause()
        >>> time.sleep(2)
        >>> robot.move.resume()

        """

        # check for valid arguments
        try:
            valid = (_valid_angle(angle) and _valid_ang_speed(ang_speed)
                    and _valid_bool(protect, 'protect')
                    and _valid_bool(right, 'right'))
        except:
            print('Error: invalid argument to .rotate_left() method')
            return
        if not valid:
            return
        # do nothing if motion is currently protected
        if self._ctrl._protect:
            return
        # calculate desired rotation angle (radians)
        angle = angle*(pi/180)
        # calculate desired angular speed (radians/second)
        ang_speed = ang_speed*(pi/180)
        # adjust angle and speed for right rotation (i.e., clockwise)
        if right:
            angle = -angle
            ang_speed = -ang_speed
        # assign the command to 'motion_call' to test for repeats
        motion_call = Move('rotate', protect=protect, ang_speed=ang_speed,
                           angle=angle)
        # return without calling motion if a repeat is being requested
        if self._is_repeat_call(motion_call):
            return
        # if interrupting, store current motion to resume after rotation
        self._ctrl._motion_resume = Move('ready')
        if self._ctrl._motion_state not in ('stop', 'pause', 'rotate'):
            self._ctrl._motion_resume = self._ctrl._motion_prev.copy()
        # check for running motion and ramp it down if needed
        if self._ctrl._motion_state not in ('stop', 'pause', 'steer'):
            self.pause()
        # store command in motion_curr attribute
        self._ctrl._motion_curr = motion_call
        # clear out any pending sequence when calling a new immediate move
        self._ctrl._motion_queue.clear()
        # ensure motion has started before returning
        while self._ctrl._motion_state != 'rotate':
            continue

    def rotate_right(self, angle=0, ang_speed=cnst.ANG_SPD_MAX,
                     protect=False):
        """Commands clockwise rotation in place.

        Parameters
        ----------
        angle : int or float, default=0 for continuous, maximum 720
            The desired rotation angle in degrees.
        ang_speed : int or float, default=180 deg/s
            The desired angular speed. Range from 30 to 180 deg/s.
        protect : bool, default=False
            Prevents interruption by another motion command.

        Notes
        -----
        Right rotation is generated using the .rotate_left() method with
        the "right" parameter set to True.

        """

        self.rotate_left(angle, ang_speed, protect, right=True)

    def rotate_to_heading(self, desired_heading, ang_speed=cnst.ANG_SPD_MAX,
                          direction=None):
        """Calculates and commands rotation to the desired heading.

        Parameters
        ----------
        desired_heading : int or float, maximum +/- 180
            The desired heading angle in degrees.
        ang_speed : int or float, default=180 deg/s
            The desired angular speed. Range from 30 to 180 deg/s.
        direction : {'left', 'right'}, default is None for shortest angle
            The rotation direction for robot to turn to heading.

        Notes
        -----
        Calculates the angle required to rotate the robot to the desired
        heading, and then uses the .rotate_left() or .rotate_right()
        methods to command rotation. The rotation direction can be set
        to 'left' (i.e., counterclockwise) or 'right' (i.e., clockwise).
        If no direction is specified, the rotation direction is
        determined automatically for the shortest rotation angle.
        
        The rotation is automatically set for protected motion to
        ensure the move is completed without interruption.

        Note
        ----
        This method cannot be added to a sequence because the
        calculation is based on the heading at the moment it is called,
        and likely not at the correct step in the sequence. Use the
        robot.moving or robot.busy properties to wait for other motion
        to finish and call this method at the time it is needed.
        
        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Rotate right 90 degrees.

        >>> robot.move.rotate_right(90)
        
        Wait for motion to stop and check heading.
        
        >>> robot.heading
        -89.8435
        
        Rotate to a heading of 180 degrees.
        
        >>> robot.move.rotate_to_heading(180)
        
        Wait for motion to stop and check heading.
        
        >>> robot.heading
        -179.4782
        
        Notice in this case, the robot took the shortest path using
        the .rotate_right() method (i.e., clockwise rotation). The
        returned heading angle is negative, but it is very close to the
        crossover point of +/-180 degrees. If the robot had over rotated
        slightly instead of under rotating, the heading value would have
        been positive. Using the direction parameter forces the desired
        rotation direction as shown below where the robot slowly rotates
        back to its starting position (i.e., heading of 0 degrees) with
        the left direction (i.e., counterclockwise):
        
        >>> robot.move.rotate_to_heading(0, ang_speed=30, direction='left')
        
        Wait for motion to stop and check heading.
        
        >>> robot.heading
        -0.5215316
        
        """

        # check for valid arguments
        try:
            valid = (_valid_heading(desired_heading)
                    and _valid_ang_speed(ang_speed)
                    and _valid_direction(direction))
        except:
            print('Error: invalid argument to .rotate_to_heading() method')
            return
        if not valid:
            return
        # wait to avoid getting heading while _tracking() method is active
        with self._ctrl._tracking_lock:
            current_heading = self._ctrl._heading * (180/pi)
        # calculate rotation angle to arrive at heading (final - intial)
        rotation_angle = desired_heading - current_heading
        # if rotation direction is set to left, ensure a positive angle
        if direction == 'left':
            if rotation_angle < 0:
                rotation_angle += 360
        # if rotation direction is set to right, ensure a negative angle
        elif direction == 'right':
            if rotation_angle > 0:
                rotation_angle -= 360
        # otherwise find the shortest angular path direction
        elif rotation_angle > 180:
            rotation_angle -= 360
        elif rotation_angle < -180:
            rotation_angle += 360
        # command left rotation
        if rotation_angle > 0:
            self.rotate_left(rotation_angle, ang_speed, protect=True)
        # command right rotation
        elif rotation_angle < 0:
            self.rotate_right(-rotation_angle, ang_speed, protect=True)
        
    def arc_forward(self, speed, radius, arc_length=0, arc_angle=0,
                    sense='counterclockwise', protect=False, reverse=False):
        """Commands a forward (or reverse) arc path with constant speed.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        radius : int or float
            The desired arc radius in cm. Must be positive. Minimum
            radius is 0.5 cm.
        arc_length : int or float, default=0
            The desired arc length to travel in cm. The default of zero
            commands a continuous arc or allows arc angle to be defined.
            Must be positive for discrete arc motion.
        arc_angle : int or float, default=0, maximum 720
            The desired arc angle to travel in degrees. The default of
            zero commands a continuous arc or allows arc length to be
            defined. Must be positive for discrete arc motion.
        sense : {'counterclockwise', 'clockwise'}
            The rotation sense for robot to rotate.
        protect : bool, default=False
            Prevents interruption by another motion command.
        reverse : bool, default=False
            Reverses the direction

        Notes
        -----
        Uses a linear velocity profile to approximate constant
        acceleration by ramping velocity up and down at the start and
        stop, which produces smoother motion with less wheel slippage.

        Examples
        --------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command continuous arc with a 15 cm radius at a speed of 10 cm/s
        and pause after 1 second.

        >>> robot.move.arc_forward(10, 15)
        >>> time.sleep(1)
        >>> robot.move.pause()

        Move an arc length of 10 cm with a 20 cm radius at a speed of
        5 cm/s with a clockwise sense.

        >>> robot.move.arc_forward(5, 20, arc_length=10, sense='clockwise')

        Build and run a sequence with the .sequence() method. Every move
        in a sequence is automatically protected so that later steps
        can't interrupt earlier ones.

        >>> robot.move.sequence() \\
        ...     .arc_forward(10, 5, arc_angle=45) \\
        ...     .arc_reverse(10, 5, arc_angle=45) \\
        ...     .arc_forward(15, 25, arc_angle=30, sense='clockwise') \\
        ...     .arc_reverse(15, 25, arc_angle=30, sense='clockwise') \\
        ...     .run()

        An unprotected move can be interrupted at any time with a new
        motion call. The control loop handles the smooth transitions.
        The following example calls arc motion and then reverse motion
        without stopping first.

        >>> robot.move.arc_forward(10, 15, sense='clockwise')
        >>> time.sleep(1)
        >>> robot.move.reverse(20, 10)

        """

        # check for valid arguments
        try:
            valid = (_valid_speed(speed) and _valid_radius(radius)
                    and _valid_arc_size(arc_length, arc_angle)
                    and _valid_sense(sense)
                    and _valid_bool(protect, 'protect')
                    and _valid_bool(reverse, 'reverse'))
        except:
            print('Error: invalid argument to .arc_forward() method')
            return
        if not valid:
            return
        # do nothing if motion is currently protected
        if self._ctrl._protect:
            return
        # limit velocity as needed to avoid exceeding SPD_MAX on outer wheel
        if (1 + ((cnst.WHEEL_SPAN/2)/radius)) * speed > cnst.SPD_MAX:
            speed = cnst.SPD_MAX / (1 + ((cnst.WHEEL_SPAN/2)/radius))
            if speed/radius > cnst.ANG_SPD_MAX:
                speed = radius * cnst.ANG_SPD_MAX
        # calculate arc length from desired arc angle
        if arc_angle != 0:
            arc_length = radius * arc_angle*(pi/180)
        # change signs for reverse direction
        if reverse:
            speed = -speed
            arc_length = -arc_length
        # assign the command to 'motion_call' to test for repeats
        motion_call = Move('arc', protect=protect, speed=speed,
                           radius=radius, arc_length=arc_length, sense=sense)
        # return without calling motion if a repeat is being requested
        if self._is_repeat_call(motion_call):
            return
        # check for running motion and ramp it down if needed
        if self._ctrl._motion_state not in ('stop', 'pause', 'steer'):
            self.pause()
        # store command in motion_curr attribute
        self._ctrl._motion_curr = motion_call
        # clear out any pending sequence when calling a new immediate move
        self._ctrl._motion_queue.clear()
        # ensure motion has started before returning
        while self._ctrl._motion_state != 'arc':
            continue

    def arc_reverse(self, speed, radius, arc_length=0, arc_angle=0,
                    sense='counterclockwise', protect=False):
        """Commands a reverse arc path with constant speed.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        radius : int or float
            The desired arc radius in cm. Must be positive. Minimum
            radius is 0.5 cm.
        arc_length : int or float, default=0
            The desired arc length to travel in cm. The default of zero
            commands a continuous arc or allows arc angle to be defined.
            Must be positive for discrete arc motion.
        arc_angle : int or float, default=0, maximum 720
            The desired arc angle to travel in degrees. The default of
            zero commands a continuous arc or allows arc length to be
            defined. Must be positive for discrete arc motion.
        sense : {'counterclockwise', 'clockwise'}
            The rotation sense for robot to rotate.
        protect : bool, default=False
            Prevents interruption by another motion command.

        Notes
        -----
        A reverse arc path is generated using the .arc_forward() method
        with the reverse parameter set to True.

        """

        self.arc_forward(speed, radius, arc_length, arc_angle, sense,
                         protect, reverse=True)

    def steer_left(self, radius, arc_angle=0, protect=False, right=False):
        """Steers left in an arc path at a discrete angle if already in motion.

        Parameters
        ----------
        radius : int or float
            The desired arc radius in cm. Must be positive. Minimum
            radius is 0.5 cm.
        arc_angle : int or float, default=0, maximum 360
            The desired arc angle in degrees. The default of zero
            commands a continuous steering arc. Must be positive for
            discrete steering motion.
        protect : bool, default=False
            Prevents interruption by another motion command.
        right : bool, default=False
            Commands right steering instead of left (changes sense).

        Notes
        -----
        Uses the .arc_forward() method to create a an arc path for
        smooth steering without stopping. This method only works when
        the robot is already moving in either a linear or arc path.
        After turning through the specified angle, the robot resumes its
        previous motion path. If no arc_angle is given (i.e., set to
        default of zero), the steering motion is a continuous arc and
        the previous motion cannot be resumed.

        Note
        ----
        Steering only makes sense as an interruption of motion that is
        already underway, so unlike .forward(), .rotate_left(), and
        .arc_forward(), this method cannot be added to a sequence.

        Example
        -------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Start by commanding forward motion at 5 cm/s. Then steer left
        30 degrees and back to the right 210 degrees to loop around.

        >>> robot.move.forward(5)
        >>> time.sleep(1)
        >>> robot.move.steer_left(25, 30)
        >>> time.sleep(4)
        >>> robot.move.steer_right(5, 210)
        >>> time.sleep(6)
        >>> robot.move.pause()
        
        Start by commanding arc_forward motion at 5 cm/s with a radius
        of 25 cm. Then steer right continuously with a 5 cm radius. You
        must pause or command new motion to end continuous steering.

        >>> robot.move.arc_forward(5, 25)
        >>> time.sleep(2)
        >>> robot.move.steer_right(5)
        >>> time.sleep(2)
        >>> robot.move.pause()

        """

        # check for valid arguments
        try:
            valid = (_valid_radius(radius) and _valid_angle(arc_angle)
                    and _valid_bool(protect, 'protect')
                    and _valid_bool(right, 'right'))
        except:
            print('Error: invalid argument to .steer_left() method')
            return
        if not valid:
            return
        # do nothing if robot is not in linear or arc motion or is ramping down
        if (self._ctrl._protect
                or self._ctrl._motion_state in ('stop', 'rotate', 'steer')
                or self._ctrl._velo_set == 0):
            return
        # store previous motion to resume after steering
        self._ctrl._motion_resume = self._ctrl._motion_prev.copy()
        # calculate desired robot steering angle (rad)
        if self._ctrl._velo_set > 0:
            arc_length = radius * arc_angle*(pi/180)
        else:
            arc_length = -radius * arc_angle*(pi/180)
        # set the arc radius and rotation sense for steering
        if right:
            sense = 'clockwise'
        else:
            sense = 'counterclockwise'
        # assign the command to 'motion_call' to test for repeats
        motion_call = Move('steer', protect=protect,
                           speed=self._ctrl._velo_set, radius=radius,
                           arc_length=arc_length, sense=sense)
        # return without calling motion if a repeat is being requested
        if self._is_repeat_call(motion_call):
            return
        # store command in motion_curr attribute
        self._ctrl._motion_curr = motion_call
        # ensure motion has started before returning
        while self._ctrl._motion_state != 'steer':
            continue

    def steer_right(self, radius, arc_angle=0, protect=False):
        """Steers right in an arc path at a discrete angle if already in motion.

        Parameters
        ----------
        radius : int or float
            The desired arc radius in cm. Must be positive. Minimum
            radius is 0.5 cm.
        arc_angle : int or float, default=0, maximum 720
            The desired arc angle in degrees. The default of zero
            commands a continuous steering arc. Must be positive for
            discrete steering motion.
        protect : bool, default=False
            Prevents interruption by another motion command.

        Notes
        -----
        Right steering motion is generated using the .steer_left()
        method with the right parameter set to True.

        """

        self.steer_left(radius, arc_angle, protect, right=True)

    def change_speed(self, speed):
        """Adjusts the speed of the active motion command.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.

        Notes
        -----
        Changing the speed is only possible during an active linear,
        arc, or steering command. During normal operation, a new motion
        call pauses all existing motion before starting (except for the
        steer command, which maintains the current speed). This command
        allows the speed to be altered without first coming to a halt.

        Example
        -------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Start by commanding forward motion at 5 cm/s. Then increase the
        speed to 20 cm/s after 2 seconds.

        >>> robot.move.forward(5)
        >>> time.sleep(2)
        >>> robot.move.change_speed(20)
        >>> time.sleep(2)
        >>> robot.move.pause()

        """

        # check for valid speed input
        try:
            valid = _valid_speed(speed)
        except:
            print('Error: invalid argument to .change_speed() method')
            return
        if not valid:
            return
        # check for compatible motion in progress
        if (self._ctrl._protect
                or self._ctrl._motion_state in ('stop', 'pause', 'rotate')
                or self._ctrl._velo_set == 0):
            return     
        # determine correct sign for velocity
        if self._ctrl._velo_set < 0:
            speed = -speed
        # set new velocity
        self._ctrl._velo_set = speed

    def sequence(self):
        """Starts building a sequence of discrete, protected moves.

        Returns
        -------
        Sequence
            A new, empty Sequence object tied to this robot's motion
            system.

        Notes
        -----
        A Sequence is a chain of discrete moves that run back-to-back:
        once one move finishes, the control loop immediately starts the
        next one. Each of the Sequence class's methods (.forward(),
        .reverse(), .rotate_left(), .rotate_right(), .arc_forward(), and
        .arc_reverse()) appends one move to the sequence and returns the
        Sequence itself, so calls can be chained together. Nothing is
        sent to the robot until .run() is called.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Move 5 cm at 5 cm/s, reverse 5 cm at 5 cm/s, move 10 cm at
        20 cm/s, then reverse 10 cm at 20 cm/s.

        >>> robot.move.sequence() \\
        ...     .forward(5, 5) \\
        ...     .reverse(5, 5) \\
        ...     .forward(20, 10) \\
        ...     .reverse(20, 10) \\
        ...     .run()

        A Sequence can also be built up over several statements, which
        is useful for building a sequence in a loop or from a list of
        waypoints.

        >>> path = robot.move.sequence()
        >>> for distance in (5, 10, 15):
        ...     path.forward(10, distance)
        ...     path.rotate_left(90, 90)
        >>> path.run()

        """

        return Sequence(self)

    def _is_repeat_call(self, motion_call):
        """Checks for repeat user calls with the same arguments.

        Used to prevent repeat motion calls from sending commands to the
        control loop where they will needlessly trigger a state update.

        """

        # wait to avoid testing during update of control loop state
        while self._ctrl._motion_curr.kind != 'ready':
            continue
        # once update is complete, check for repeat motion call
        if (self._ctrl._motion_state not in ('stop', 'pause')
                and motion_call == self._ctrl._motion_prev):
            return True
        return False


class Sequence:
    """
    Builds and runs an ordered chain of discrete, protected moves.

    ...

    Parameters
    ----------
    motion : Motion
        The Motion instance (i.e., "robot.move") this sequence belongs
        to. Normally a Sequence is created with the .sequence() method
        rather than directly.

    Notes
    -----
    Each move-adding method (.forward(), .reverse(), .rotate_left(),
    .rotate_right(), .arc_forward(), and .arc_reverse()) validates its
    arguments, appends one move to the sequence, and returns the
    Sequence itself so calls can be chained. Only discrete moves (a
    nonzero distance, angle, arc_length, or arc_angle) are allowed,
    since a continuous move would never finish and the rest of the
    sequence would never run.

    No move is sent to the robot until .run() is called. This keeps
    "describing a plan" (building the Sequence) separate from
    "carrying it out" (calling .run()), so a sequence built across
    several statements, a loop, or a function is only ever handed to
    the control loop once, as a complete, ordered chain.

    Examples
    --------

    Create an instance of PiBOT.

    >>> from pibot import PiBOT
    >>> robot = PiBOT()

    Trace out a square with 20 cm sides.

    >>> square = robot.move.sequence()
    >>> for side in range(4):
    ...     square.forward(15, 20)
    ...     square.rotate_left(90, 90)
    >>> square.run()

    """

    def __init__(self, motion):
        """Creates an empty sequence tied to the given Motion instance."""

        self._motion = motion
        self._moves = []

    def forward(self, speed, distance, reverse=False):
        """Adds a discrete forward (or reverse) move to the sequence.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        distance : int or float
            The desired distance to travel in cm. Must be nonzero.
        reverse : bool, default=False
            Reverses the direction.

        Returns
        -------
        Sequence
            This same Sequence, so calls can be chained together.

        """

        try:
            valid = (_valid_speed(speed)
                    and _valid_distance(distance, required=True)
                    and _valid_bool(reverse, 'reverse'))
        except:
            print('Error: invalid argument to .forward() sequence step')
            return self
        if not valid:
            return self
        if reverse:
            speed = -speed
            distance = -distance
        self._moves.append(Move('linear', protect=True, speed=speed,
                                distance=distance))
        return self

    def reverse(self, speed, distance):
        """Adds a discrete reverse move to the sequence.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        distance : int or float
            The desired distance to travel in cm. Must be nonzero.

        Returns
        -------
        Sequence
            This same Sequence, so calls can be chained together.

        Notes
        -----
        A reverse move is generated using the .forward() method with
        the reverse parameter set to True.

        """

        return self.forward(speed, distance, reverse=True)

    def rotate_left(self, angle, ang_speed=cnst.ANG_SPD_MAX, right=False):
        """Adds a discrete counterclockwise rotation to the sequence.

        Parameters
        ----------
        angle : int or float, maximum 720
            The desired rotation angle in degrees. Must be nonzero.
        ang_speed : int or float, default=180 deg/s
            The desired angular speed. Range from 30 to 180 deg/s.
        right : bool, default=False
            Commands right (clockwise) rotation instead of left.

        Returns
        -------
        Sequence
            This same Sequence, so calls can be chained together.

        """

        try:
            valid = (_valid_angle(angle, required=True)
                    and _valid_ang_speed(ang_speed)
                    and _valid_bool(right, 'right'))
        except:
            print('Error: invalid argument to .rotate_left() sequence step')
            return self
        if not valid:
            return self
        angle = angle*(pi/180)
        ang_speed = ang_speed*(pi/180)
        if right:
            angle = -angle
            ang_speed = -ang_speed
        self._moves.append(Move('rotate', protect=True, ang_speed=ang_speed,
                                angle=angle))
        return self

    def rotate_right(self, angle, ang_speed=cnst.ANG_SPD_MAX):
        """Adds a discrete clockwise rotation to the sequence.

        Parameters
        ----------
        angle : int or float, maximum 720
            The desired rotation angle in degrees. Must be nonzero.
        ang_speed : int or float, default=180 deg/s
            The desired angular speed. Range from 30 to 180 deg/s.

        Returns
        -------
        Sequence
            This same Sequence, so calls can be chained together.

        Notes
        -----
        Right rotation is generated using the .rotate_left() method
        with the "right" parameter set to True.

        """

        return self.rotate_left(angle, ang_speed, right=True)

    def arc_forward(self, speed, radius, arc_length=0, arc_angle=0,
                    sense='counterclockwise', reverse=False):
        """Adds a discrete forward (or reverse) arc move to the sequence.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        radius : int or float
            The desired arc radius in cm. Must be positive. Minimum
            radius is 0.5 cm.
        arc_length : int or float, default=0
            The desired arc length to travel in cm. Either arc_length
            or arc_angle must be set (but not both).
        arc_angle : int or float, default=0, maximum 720
            The desired arc angle to travel in degrees. Either
            arc_length or arc_angle must be set (but not both).
        sense : {'counterclockwise', 'clockwise'}
            The rotation sense for robot to rotate.
        reverse : bool, default=False
            Reverses the direction.

        Returns
        -------
        Sequence
            This same Sequence, so calls can be chained together.

        """

        try:
            valid = (_valid_speed(speed) and _valid_radius(radius)
                    and _valid_arc_size(arc_length, arc_angle, required=True)
                    and _valid_sense(sense)
                    and _valid_bool(reverse, 'reverse'))
        except:
            print('Error: invalid argument to .arc_forward() sequence step')
            return self
        if not valid:
            return self
        # limit velocity as needed to avoid exceeding SPD_MAX on outer wheel
        if (1 + ((cnst.WHEEL_SPAN/2)/radius)) * speed > cnst.SPD_MAX:
            speed = cnst.SPD_MAX / (1 + ((cnst.WHEEL_SPAN/2)/radius))
            if speed/radius > cnst.ANG_SPD_MAX:
                speed = radius * cnst.ANG_SPD_MAX
        # calculate arc length from desired arc angle
        if arc_angle != 0:
            arc_length = radius * arc_angle*(pi/180)
        if reverse:
            speed = -speed
            arc_length = -arc_length
        self._moves.append(Move('arc', protect=True, speed=speed,
                                radius=radius, arc_length=arc_length,
                                sense=sense))
        return self

    def arc_reverse(self, speed, radius, arc_length=0, arc_angle=0,
                    sense='counterclockwise'):
        """Adds a discrete reverse arc move to the sequence.

        Parameters
        ----------
        speed : int or float
            The desired speed in cm/s. Range from 5 to 30 cm/s.
        radius : int or float
            The desired arc radius in cm. Must be positive. Minimum
            radius is 0.5 cm.
        arc_length : int or float, default=0
            The desired arc length to travel in cm. Either arc_length
            or arc_angle must be set (but not both).
        arc_angle : int or float, default=0, maximum 720
            The desired arc angle to travel in degrees. Either
            arc_length or arc_angle must be set (but not both).
        sense : {'counterclockwise', 'clockwise'}
            The rotation sense for robot to rotate.

        Returns
        -------
        Sequence
            This same Sequence, so calls can be chained together.

        Notes
        -----
        A reverse arc move is generated using the .arc_forward() method
        with the reverse parameter set to True.

        """

        return self.arc_forward(speed, radius, arc_length, arc_angle, sense,
                                reverse=True)

    def run(self):
        """Sends the sequence to the control loop and starts it running.

        Notes
        -----
        If another unprotected move is currently active, it is
        interrupted immediately and the sequence starts in its place,
        the same way any new unprotected motion call would interrupt
        it. If another protected move or sequence is currently active,
        this sequence is appended after it and will start automatically
        once the current one finishes.

        After .run() returns, the moves already added to this Sequence
        have been sent to the robot; building can continue by adding
        more moves and calling .run() again, which will append them
        after any of this sequence's own moves that are still pending.

        """

        if not self._moves:
            return
        ctrl = self._motion._ctrl
        ctrl._motion_queue.extend(self._moves)
        self._moves = []
        # ensure the first move of the sequence has started before returning
        while not ctrl._protect:
            continue
