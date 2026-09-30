"""

This module contains the PiBOT class that incorporates the control,
motion, sensors, and outputs modules to create a fully functioning
robot. The PiBOT class has attributes to track the robot position
and heading, determine its current state, perform lidar scans of the
surroundings, and work with the lidar data. The examples in the
following section demonstrate how to use the PiBOT attributes. The
motion, sensors, and outputs modules also contain many examples of how
to use their features from within an instance of the PiBOT class.

"""

import constants as cnst
from math import pi, sin, cos, tan, radians
from utime import sleep_ms
from control import Control
from motion import Motion
from sensors import Whiskers, IrSensors, LidarSensor, IrRemote
from outputs import Buzzer, LEDs


class _DetectedObject:
    """
    Represents one candidate object found in a lidar scan.

    ...

    Parameters
    ----------
    leading_edge : int
        Index into the scan data of the object's leading edge (the
        steepest negative slope found between the first local maximum
        and the local minimum).
    min_index : int
        Index into the scan data of the object's closest point (a
        local minimum in the distance data).
    trailing_edge : int
        Index into the scan data of the object's trailing edge (the
        steepest positive slope found between the local minimum and
        the second local maximum).
    center_angle : int or float
        The estimated heading angle in degrees to the center of the
        object.
    min_distance : int or float
        The estimated minimum distance in cm to the object.
    width : int or float
        The estimated width in cm of the object.

    Notes
    -----
    _find_objects() builds one of these for each object it detects, and
    detect_objects() reads center_angle, min_distance, and width back
    out of each one. This replaces the plain 6-element tuples used in
    an earlier version of this library (e.g.
    (12, 15, 19, 42.7, 35.8, 12.6)), where each position had a
    different, undocumented meaning and every caller had to know the
    right index to read. Reading obj.center_angle instead of obj[3] is
    self-documenting, and safer to extend if a future version needs to
    record more about a detected object.

    """

    __slots__ = ('leading_edge', 'min_index', 'trailing_edge',
                'center_angle', 'min_distance', 'width')

    def __init__(self, leading_edge, min_index, trailing_edge,
                center_angle, min_distance, width):
        """Creates a _DetectedObject with the given field values."""

        self.leading_edge = leading_edge
        self.min_index = min_index
        self.trailing_edge = trailing_edge
        self.center_angle = center_angle
        self.min_distance = min_distance
        self.width = width

    def __repr__(self):
        """Returns a readable representation for debugging."""

        return ('_DetectedObject(center_angle=%r, min_distance=%r, '
                'width=%r, leading_edge=%r, min_index=%r, '
                'trailing_edge=%r)' %(self.center_angle, self.min_distance,
                                     self.width, self.leading_edge,
                                     self.min_index, self.trailing_edge))


class Scan:
    """
    Represents lidar scan data as paired angle and distance lists.

    ...

    Parameters
    ----------
    angle : list of int or float
        The heading angles in degrees for each scan point.
    distance : list of int or float
        The corresponding lidar distances in cm for each scan point,
        measured from the robot's center of rotation.

    Notes
    -----
    A Scan bundles the angle and distance lists that were previously
    passed as two separate arguments to every lidar analysis method
    (.max_distance(), .min_distance(), .convert_to_xy(),
    .center_point(), .centroid(), .find_corners(), and
    .detect_objects()). Because both lists always need to describe the
    same set of points, in the same order, keeping them in one object
    with validation in one place removes the risk of the two lists
    drifting out of sync, or being validated inconsistently, across
    different methods.

    A Scan does not copy the lists it is given; it stores the same
    list objects it receives, the same way a plain list or tuple
    constructor would. PiBOT.scan() always builds a fresh pair of
    lists for each new scan, so .lidar_scan from an earlier scan is
    never affected by a later one. If a Scan is built directly from
    lists a program intends to keep changing, pass copies (e.g.
    Scan(my_angle.copy(), my_distance.copy())) to keep them independent.

    If angle and distance are not both lists of the same length
    containing only numeric values, a Scan is still created, but with
    empty angle and distance lists, and an error is printed. This
    matches how every other method in this library reports a bad
    argument, and it means a Scan built from bad data is simply
    rejected by any method's own length check further downstream, with
    a clear error message, rather than crashing.

    """

    def __init__(self, angle, distance):
        """Creates a Scan, validating that angle and distance match."""

        valid = (isinstance(angle, list) and isinstance(distance, list)
                and len(angle) == len(distance)
                and all(isinstance(a, (int, float)) for a in angle)
                and all(isinstance(d, (int, float)) for d in distance))
        if not valid:
            print('Error: angle and distance must be lists of numeric '
                 + 'values with the same length')
            angle, distance = [], []
        self.angle = angle
        self.distance = distance

    def __len__(self):
        """Returns the number of points in the scan."""

        return len(self.angle)

    def __iter__(self):
        """Iterates over the scan as (angle, distance) point pairs."""

        return zip(self.angle, self.distance)

    def __repr__(self):
        """Returns a readable representation for debugging."""

        return 'Scan(%d points)' %len(self.angle)


class PiBOT:
    """
    Creates a top-level robot object to operate the PiBOT.

    ...

    Attributes
    ----------
    whiskers : Whiskers object
        Checks for left or right whisker contact.
    ir : IrSensors object
        Checks for left or right IR sensor detection.
    lidar : LidarSensor object
        Gets the current distance reading from the lidar sensor.
    remote : IrRemote object
        Gets commands from the IR remote control.
    leds : LEDs object
        Controls the left and right neopixel LEDs.
    buzzer : Buzzer object
        Controls the buzzer sound output.
    move : Motion object
        Controls the robot motion.
    lidar_scan : Scan object
        The angle and distance data stored from the most recent lidar
        scan, as a Scan object with .angle and .distance lists.
    """

    def __init__(self):
        """Creates robot control attributes."""

        # OBJECTS NEEDED TO CREATE ROBOT
        self.whiskers = Whiskers()
        self.ir = IrSensors()
        self.lidar = LidarSensor()
        self.remote = IrRemote()
        self.leds = LEDs()
        self.buzzer = Buzzer()
        # pass objects into the _control object to allow access to attributes
        self._control = Control(leds=self.leds, buzzer=self.buzzer,
                                remote=self.remote)
        # pass _control object into move for access to contol attributes
        self.move = Motion(self._control)
        # ATTRIBUTE FOR LIDAR SCAN
        self.lidar_scan = Scan([], [])

    @property
    def position(self):
        """Gets or sets the robot x-y position.

        Notes
        -----
        The global coordinate system is based on the position of the
        centerpoint between the robot's wheels when an instance of the
        PiBOT class is first created or after resetting. The x axis
        points forward along the path, and the y axis points toward the
        left wheel.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Get the position.

        >>> robot.position
        [0, 0]

        Set the position.

        >>> robot.position = [7.07, -7.07]

        """

        # wait to avoid getting new value while _tracking() method is active
        with self._control._tracking_lock:
            # return a static copy instead of the list object itself
            return self._control._position.copy()

    @position.setter
    def position(self, value):
        # check for valid argument
        if not isinstance(value, list):
            return print('Error: position must be a list in the form [x, y]')
        if len(value) != 2:
            return print('Error: position must be a list of [x, y] values')
        if not all(isinstance(i, (int, float)) for i in value):
            return print('Error: x and y must be numeric values')
        # wait to avoid setting new value while _tracking() method is active
        with self._control._tracking_lock:
            # store a copy so later changes to the caller's list can't
            # silently corrupt the robot's internal position tracking
            self._control._position = value.copy()

    @property
    def heading(self):
        """Gets or sets the robot heading angle.

        Notes
        -----
        The heading angle is measured in degrees from the x axis with
        positive values going counterclockwise.

        Important
        ---------
        The heading value ranges from -180 to 180 degrees and changes
        sign if it crosses over the min/max value.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Get the heading.

        >>> robot.heading
        89.54

        Set the heading.

        >>> robot.heading = 180

        """

        # wait to avoid getting new value while _tracking() method is active
        with self._control._tracking_lock:
            return self._control._heading * (180/pi)

    @heading.setter
    def heading(self, value):
        # check for valid argument
        if not isinstance(value, (int, float)):
            return print('Error: heading must be a numeric value in degrees')
        elif value < -180 or value > 180:
            return print('Error: heading must be in the range +/-180 degrees')
        # wait to avoid setting new value while _tracking() method is active
        with self._control._tracking_lock:
            self._control._heading = value * (pi/180)

    @property
    def current_time(self):
        """Gets the current time in seconds as a read-only value.

        This is the time since an instance of the PiBOT class was
        created. The clock is reset when the .reset() method is called.

        """

        return self._control._t_current

    @property
    def motion_state(self):
        """Gets the current motion state of the robot.

        The motion states are: 'stop', 'pause', 'linear', 'rotate',
        'arc', and 'steer'.

        """

        return self._control._motion_state

    @property
    def moving(self):
        """Determines if the robot is currently moving.

        Checks for motion states: 'stop' or 'pause'.

        """

        if self.motion_state in ('stop', 'pause'):
            return False
        else:
            return True

    @property
    def busy(self):
        """Determines if the robot has protected motion or a running sequence.

        The robot is considered busy if protected motion is being
        commanded or if a sequence has moves still waiting to run.

        """

        if self._control._protect or self._control._motion_queue:
            return True
        else:
            return False

    def reset(self):
        """Stops motion, exits thread, and reinitializes attributes."""

        self.move.pause()
        sleep_ms(15)
        self._control._thread_running = False
        # wait for thread to exit before reinitializing and starting new thread
        sleep_ms(25)
        self._control.__init__(leds=self.leds, buzzer=self.buzzer,
                               remote=self.remote)
        # pause to allow new thread to take effect before handling new commands
        sleep_ms(25)

    def scan(self, angle, increment=2.5, ang_speed=cnst.ANG_SPD_MAX,
             filename=None):
        """Sweeps the robot through an angle and stores lidar data.

        Parameters
        ----------
        angle : int, float, maximum +/- 720
            A value in degrees to rotate the robot for a scan sweep. A
            positive angle will rotate left (counterclockwise), while a
            negative value will rotate right (clockwise).
        increment : int, float, default=2.5
            The interval in degrees at which to capture lidar data.
            Minimum is 1 degree.
        ang_speed : int, float, default=cnst.ANG_SPD_MAX (180 deg/s)
            The desired angular speed. Range from 30 to 180 deg/s.
        filename : str, optional
            Name of the file to save the lidar data to a CSV.

        Returns
        -------
        Scan
            The angle and distance data collected during the scan, as
            a Scan object with .angle and .distance lists. This same
            object is also stored in .lidar_scan.

        Notes
        -----
        The default angle increment of the lidar scan is set to 2.5
        degrees, which represents a good compromise between speed and
        resolution when scanning at the default maximum angular speed.
        The actual angle stored varies because the data is collected
        dynamically while the robot rotates. The lidar detects a field
        of view of about 15 degrees, meaning the data spaced at 2.5
        degrees represents a higher resolution than the lidar can
        really discern and results in overlapping ranges of detection.
        In practice, the lidar sensor is fairly accurate at measuring
        distances to objects that are large and flat, but for small
        objects that are near the end of its measuring range, the
        sensor will report larger values than expected, making the
        object appear further away than it really is. In some cases, an
        object can be missed entirely because the IR light from the
        sensor's 15 degree beam width reflects from the background and
        overwhelms the light reflected from the small object in the
        foreground.

        The heading for each data point is recorded right after the
        lidar reading is taken, rather than before or as an average of
        the two. The lidar reading takes about 10 ms, during which the
        robot keeps rotating; at the default angular speed of 180 deg/s
        this amounts to roughly 1.8 degrees of additional rotation
        while a single reading is taken. This is small compared to the
        15 degree beam spread discussed above, which remains the
        dominant source of angular uncertainty in a scan, so the timing
        of the heading reading was not tuned further.

        The optional filename is used to save a CSV file to the RP2040's
        memory, which can be accessed with Thonny and downloaded to the
        computer for analysis.

        Each call to .scan() builds a new Scan from freshly collected
        data, so a Scan saved from an earlier call (e.g. old_scan =
        robot.scan(90)) is never changed by a later scan.

        Important
        ---------
        Remember that the lidar readings have an offset added so that
        the distances recorded are from the center of the robot's
        wheel base. Therefore, the distance values in a scan represent
        the radius from the robot's center of rotation.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 180 degree clockwise scan. The returned Scan is
        stored in a local variable as shown; however, it is not
        necessary to use the returned value because it can also be
        accessed directly with the .lidar_scan attribute.

        >>> scan_data = robot.scan(-180)

        Command a 360 degree counterclockwise scan and store the data
        to a file called 'test_data'. The data is not saved to a local
        variable. In the output, it is clear that the default angle
        increment of 2.5 degrees cannot be followed exactly but tracks
        fairly close to the desired step size.

        >>> robot.scan(360, filename='test_data')
        Scan(144 points)
        >>> robot.lidar_scan.angle
        [0.0, 2.713043, 6.052173, 8.869564, 10.01739, 12.52174,...

        """

        # check for valid argument
        if not isinstance(angle, (int, float)) or abs(angle) > cnst.ANG_MAX:
            return print('Error: maximum magnitude of angle is '
                         +'+/-%d degrees' %cnst.ANG_MAX)
        if angle == 0:
            return print('Error: angle cannot be zero')
        if not isinstance(increment, (int, float)):
            return print('Error: increment must be a numeric value')
        if increment < 1:
            return print('Error: minimum increment is 1 degree')
        if increment > abs(angle):
            return print('Error: increment must be less than scan angle')
        if not isinstance(ang_speed, (int, float)):
            return print('Error: ang_speed must be a numeric value')
        if ang_speed < 0:
            return print('Error: ang_speed must be positive')
        if ang_speed > cnst.ANG_SPD_MAX:
            return print('Error: maximum ang_speed is: '
                         +'%.2f deg/s' %cnst.ANG_SPD_MAX)
        if ang_speed < cnst.ANG_SPD_MIN:
            return print('Error: minimum ang_speed is: '
                         +'%.2f deg/s' %cnst.ANG_SPD_MIN)
        if filename and not isinstance(filename, str):
            return print('Error: filename must be a string')
        # create an angle increment counter
        i = 0
        # get the starting heading angle
        start_angle = self.heading
        # ensure the starting angle is positive
        if start_angle < 0:
            start_angle += 360
        # collect into fresh lists so an earlier .lidar_scan (or any Scan
        # a program saved from a previous call) is never changed by this
        # scan; the two lists are wrapped into a single new Scan at the end
        angle_data = []
        dist_data = []
        # start protected rotation at the specified angle and direction
        if angle > 0:
            self.move.rotate_left(angle, ang_speed, protect=True)
        elif angle < 0:
            self.move.rotate_right(-angle, ang_speed, protect=True)
        # wait for rotation to start before collecting data
        while self._control._motion_state != 'rotate':
            continue
        # while rotation is active, collect data at the approximate increment
        while (self._control._motion_state == 'rotate'
               and abs(increment*i) <= abs(angle)):
            # adjust the desired angle for the next increment
            desired_heading = start_angle + increment*i
            # get the current heading angle
            current_heading = self.heading
            # ensure the current heading angle is positive
            if current_heading < 0:
                current_heading += 360
            # add the necessary wraps to the current heading angle
            while current_heading > desired_heading + 180:
                current_heading -= 360
            while current_heading < desired_heading - 180:
                current_heading += 360
            # store data when the current heading reaches the desired heading
            # (heading is read right after the lidar call; see Notes above)
            if angle > 0 and current_heading > desired_heading:
                dist_data.append(self.lidar.read())
                angle_data.append(self.heading)
                i += 1
            elif angle < 0 and current_heading < desired_heading:
                dist_data.append(self.lidar.read())
                angle_data.append(self.heading)
                i -= 1
        # add one more data point at the end of rotation after settling
        dist_data.append(self.lidar.read())
        angle_data.append(self.heading)
        # create a CSV (comma-separated values) file and write lidar data
        if filename:
            file = open(f'{filename}.csv', 'w')
            # add a file header
            file.write('angle (degrees), distance (cm)\n')
            # write each line of data
            for i in range(len(angle_data)):
                file.write(f'{angle_data[i]}, {dist_data[i]}\n')
            file.close()
        # wait for protected status to end so other motion won't be blocked
        while self._control._protect:
            continue
        # wrap the collected data as a new Scan and store/return it
        self.lidar_scan = Scan(angle_data, dist_data)
        return self.lidar_scan

    def max_distance(self, scan):
        """Finds the maximum value in the lidar scan data.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.

        Returns
        -------
        2-tuple
            A pair comprising the heading angle and maximum distance.

        Notes
        -----

        Finds the maximum value and corresponsing angle in the scan
        data. If there is more than one identical maximum value, the
        first one in the list is returned. If there are values out of
        range (i.e., cnst.LIDAR_OUT_OF_RANGE, 140 cm), the method finds
        the block of out-of-range values in the distance list that is
        longest and returns the angle near the center of the block. If
        there is only one out-of-range value, that will be returned as
        the maximum. If there is more than one non-adjacent single
        out-of-range value, the first one in the list will be returned
        as the maximum.
                
        Important
        ---------
        This method should only be used when the robot is stationary
        immediately after a lidar scan.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 180 degree clockwise scan and use the returned Scan
        to find the maximum.

        >>> scan_data = robot.scan(-180)
        >>> robot.max_distance(scan_data)
        (128.4522, 140.0)

        The argument can also be the lidar data taken directly from the
        .lidar_scan attribute instead of a local copy of the Scan used
        in the previous example.

        >>> robot.scan(-180)
        Scan(72 points)
        >>> robot.max_distance(robot.lidar_scan)
        (-80.34782, 91.0)

        """

        # check for valid argument
        if not isinstance(scan, Scan):
            return print('Error: scan must be a Scan object')
        if len(scan) < 3:
            return print('Error: scan must have at least 3 points')
        angle = scan.angle
        distance = scan.distance
        # create count variables for out-of-range values
        count = 0
        max_count = 0
        out_of_range = []
        # create list of all out-of-range values in lidar data
        oor = cnst.LIDAR_OUT_OF_RANGE
        for i in range(distance.count(oor)):
            if i == 0:
                out_of_range.append(distance.index(oor))
            else:
                out_of_range.append(distance.index(oor, out_of_range[i-1] + 1))
        # find center of largest patch of out-of-range values
        if len(out_of_range) > 0:
            for i in range(len(out_of_range)-1):
                if out_of_range[i] + 1 == out_of_range[i+1]:
                    count += 1
                    if count > max_count:
                        max_count = count
                        center = out_of_range[(i+1)-int(count/2)]
                else:
                    count = 0
            if max_count == 0:
                center = out_of_range[0]
            return angle[center], distance[center]
        # if there are no values out of range, return the maximum of the list
        else:
            max_index = distance.index(max(distance))
            return angle[max_index], distance[max_index]

    def min_distance(self, scan):
        """Finds the minimum value in the lidar scan data.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.

        Returns
        -------
        2-tuple
            A pair comprising the heading angle and minimum distance.

        Notes
        -----
        Finds the minimum value and corresponsing angle in the scan
        data. If there is more than one identical minimum value, the
        first one in the list is returned.
        
        Important
        ---------
        This method should only be used when the robot is stationary
        immediately after a lidar scan.
        
        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 180 degree clockwise scan and use the returned Scan
        to find the minimum.

        >>> scan_data = robot.scan(-180)
        >>> robot.min_distance(scan_data)
        (72.974, 18.2)

        The argument can also be the lidar data taken directly from the
        .lidar_scan attribute instead of a local copy of the Scan used
        in the previous example.

        >>> robot.scan(-180)
        Scan(72 points)
        >>> robot.min_distance(robot.lidar_scan)
        (-120.0, 13.8)

        """

        # check for valid argument
        if not isinstance(scan, Scan):
            return print('Error: scan must be a Scan object')
        if len(scan) < 3:
            return print('Error: scan must have at least 3 points')
        angle = scan.angle
        distance = scan.distance
        # find index of minimum value in distance list
        min_index = distance.index(min(distance))
        return angle[min_index], distance[min_index]
    
    def convert_to_xy(self, scan):
        """Converts the lidar scan data to x-y coordinates.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.

        Returns
        -------
        two-tuple of lists
            A pair of lists containing the x and y coordinates .
            
        Important
        ---------
        The coordinates returned represent the x-y positions of all the
        points detected during a lidar scan and are based on the current
        coordinate system and position of the robot. If the robot has
        moved or rotated since the previous scan, the x-y coordinates
        will not be properly located and oriented to the previously
        scanned environment.

        """

        # check for valid argument
        if not isinstance(scan, Scan):
            return print('Error: scan must be a Scan object')
        if len(scan) < 1:
            return print('Error: scan must have at least 1 point')
        angle = scan.angle
        distance = scan.distance
        # get the current x and y positions
        x_position = self.position[0]
        y_position = self.position[1]
        x = []
        y = []
        # calculate the x and y coordinates and offset by the current position
        for i in range(len(distance)):
            x.append(x_position + round(distance[i]*cos(radians(angle[i])), 1))
            y.append(y_position + round(distance[i]*sin(radians(angle[i])), 1))
        return x, y
    
    def convert_point_to_xy(self, angle, distance):
        """Converts a single lidar scan point to x-y coordinates.

        Parameters
        ----------
        angle : int, float
            A single lidar angle
        distance : int, float
            A single lidar distance

        Returns
        -------
        2-tuple
            A pair comprising the x-y coordinates.
            
        Notes
        -----
        This method uses the .convert_to_xy method and simply reformats
        the returned values for more convenient use.
        
        Important
        ---------
        The x-y position returned corresponds to a single point from
        a lidar scan and is based on the current coordinate system and
        position of the robot. If the robot has moved or rotated since
        the previous scan, the returned x-y position will not be
        properly located and oriented to the previous scan.

        """

        # check for valid arguments
        if not isinstance(angle, (int, float)):
            return print('Error: angle must be a numeric value')
        if not isinstance(distance, (int, float)):
            return print('Error: distance must be a numeric value')
        # get the current x and y position using .convert_to_xy method
        x, y = self.convert_to_xy(Scan([angle], [distance]))
        return x[0], y[0]

    def center_point(self, scan):
        """Finds the approximate geometric center of lidar scan points.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.

        Returns
        -------
        list
            A two-element x-y position list for the approximate center.

        Notes
        -----
        Finds the approximate center with a simple average of the max
        and min values of x and y coordinates of the lidar scan data.
        The angle and distance are first converted to x-y positions.
        
        Important
        ---------
        This method should only be used when the robot is stationary
        immediately after a lidar scan.
        
        This approach works best for regular shapes like rectangular
        enclosures using a 360-degree scan of the entire perimeter. For
        surroundings with complex shapes or obstacles present, this
        method will not estimate the geometric center of the surrondings
        as well.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 360 degree counterclockwise scan and use the returned
        Scan to find the approximate center point.

        >>> scan_data = robot.scan(360)
        >>> robot.center_point(scan_data)
        (-15.4, 29.6)

        """

        # check for valid argument
        if not isinstance(scan, Scan):
            return print('Error: scan must be a Scan object')
        if len(scan) < 3:
            return print('Error: scan must have at least 3 points')
        # convert lidar scan data to x-y coordinates
        x, y = self.convert_to_xy(scan)
        # calculate averages of x and y min-max values
        x_center = round(((max(x) + min(x)) / 2), 1)
        y_center = round(((max(y) + min(y)) / 2), 1)
        return x_center, y_center
    
    def centroid(self, scan):
        """Finds the centroid using a weighted average of the scan data.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.

        Returns
        -------
        list
            A two-element x-y position list for the centroid.

        Notes
        -----
        Finds the centroid with a summation of all the x and y center
        positions of the triangular wedges of the scan multiplied by
        their areas as a weighting factor. The sum of those products is
        then divided by the total area of the scan to arrive at the x
        and y coordinates of the centroid.
        
        Important
        ---------
        This method should only be used when the robot is stationary
        immediately after a lidar scan.
        
        This approach works best for a 360-degree scan of the entire
        perimeter. For surroundings with complex shapes and obstacles
        this method will accurately represent the geometric center for
        the area that is visible to the lidar sensor. Any areas of an
        enclosure blocked by an obstacle will not be included in the
        scan geometry and cannot be accounted for in the centroid
        calculation. Similarly, any areas that exceed the range of the
        lidar sensor (i.e., > 140 cm) will not be included.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 360 degree counterclockwise scan and use the returned
        Scan to find the centroid.

        >>> scan_data = robot.scan(360)
        >>> robot.centroid(scan_data)
        (28.3, -3.5)

        """

        # check for valid argument
        if not isinstance(scan, Scan):
            return print('Error: scan must be a Scan object')
        if len(scan) < 3:
            return print('Error: scan must have at least 3 points')
        # get the current x and y positions
        x_position = self.position[0]
        y_position = self.position[1]
        # calculate the wedge areas and total area
        wedge_areas = self._wedge_areas(scan)
        area = sum(wedge_areas)
        # convert lidar scan data to x-y coordinates
        x, y = self.convert_to_xy(scan)
        # calculate the center of each wedge using three vertices
        x_centers = []
        y_centers = []
        for i in range(len(x)-1):
            x_centers.append((x[i] + x[i+1] + x_position) / 3)
            y_centers.append((y[i] + y[i+1] + y_position) / 3)
        # calculate the centroid using wedge areas as weighting factor
        x_sum = 0
        y_sum = 0
        for i in range(len(wedge_areas)):
            x_sum += x_centers[i] * wedge_areas[i]
            y_sum += y_centers[i] * wedge_areas[i]
        x_centroid = round((x_sum / area), 1)
        y_centroid = round((y_sum / area), 1)
        return [x_centroid, y_centroid]

    def find_corners(self, scan):
        """Finds possible inside corners using filtered distance maxima.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.

        Returns
        -------
        2-tuple of lists
            A pair of lists comprising the corner angles and distances.
            
        Notes
        -----
        Finds the inside corners based on the maxima of the distance
        data. The maxima only correspond with the locations of inside
        corners when the scanned environment has a shape with distinct
        corners and relatively straight walls. Irregular or organic
        shapes will not produce useful results, nor will scans with
        segments that are out of range (i.e., distances > 140 cm).
                
        Important
        ---------
        This tool is designed to work with data from a 360-degree scan
        and should be used immediatedly after the scan when the robot is
        stationary. Passing data from scans of more or less than 360
        degrees will product unpredictable results.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        A 360 degree scan is commanded and the returned Scan is sent
        directly to the .find_corners() method, which returns the result
        as two lists with equal lengths.

        >>> corner_angle, corner_dist = robot.find_corners(robot.scan(360))
        >>> corner_angle
        [128.4522, -173.644, -54.1342, 6.8564]
        >>> corner_dist
        [88.4, 79.5, 80.2, 90.7]

        """

        # check for valid argument
        if not isinstance(scan, Scan):
            return print('Error: scan must be a Scan object')
        if len(scan) < 3:
            return print('Error: scan must have at least 3 points')
        angle = scan.angle
        distance = scan.distance
        # find the filtered distance maxima
        extrema = self._extrema(distance)
        extrema_filt = self._extrema_filt(distance, extrema)
        maxima_filt = extrema_filt[1]
        # split angle and distance lists in half and reorder to merge start/end
        num_points = len(distance)
        halfway = int(num_points/2)
        angle_reorder = angle[halfway+1:] + angle[0:halfway+1]
        dist_reorder = distance[halfway+1:] + distance[0:halfway+1]
        # find the filtered maxima again for the reordered distance 
        extrema = self._extrema(dist_reorder)
        extrema_filt = self._extrema_filt(dist_reorder, extrema)
        maxima_filt_reorder = extrema_filt[1]
        # keep only maxima present in both original and reordered distance list
        corners = []
        for x in maxima_filt:
            index_reorder = x - (halfway+1)
            if index_reorder < 0:
                index_reorder += num_points
            if index_reorder in maxima_filt_reorder:
                corners.append(x)
        # get the angle and distance data for only the corners
        corner_angle = [angle[x] for x in corners]
        corner_dist = [distance[x] for x in corners]
        return corner_angle, corner_dist

    def detect_objects(self, scan, filename=None):
        """Detects objects in the foreground of scan data.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.
        filename : str, optional
            Name of the file to save the object data to a CSV.

        Returns
        -------
        3-tuple of lists
            A triplet of lists comprising the approximate heading angles
            in degrees to the center of the objects, the minimum
            distances in cm to the objects, and their approximate widths
            in cm.

        Notes
        -----
        Attempts to detect sharp changes in lidar distance readings that
        represent possible leading and trailing edges of an object. If
        both a sharp decrease in distance (i.e., a step from far to near
        representing a leading edge) and then a corresponding sharp
        increase (i.e., a step from near to far representing a trailing
        edge) are detected, an object is recorded.

        Important
        ---------
        This method should only be used when the robot is stationary
        immediately after a lidar scan.
        
        If an object is too far away or too narrow, it may not be
        detected. It's also possible that this method will return data
        for an object that doesn't really exist. The returned values are
        less accurate when the object is farther from the robot because
        of the 15 degree spread of the lidar beam. For better accuracy,
        move closer to the object and rescan. The estimated distance and
        width assumes the object has a flat face oriented perpendicular
        to the axis of the robot. For irregular objects or objects
        scanned at a glancing angle, the width and distance data will be
        less accurate. The accuracy of the center angle depends on the
        edge-finding algorithm, the distance of the object, and the
        angle increment (i.e., step size) in the lidar data.
        
        Note
        ----
        For fast scans, the lidar readings lag the rotation, which leads
        to a slight angular offset in the recorded center angle of the
        detected object. For narrow objects (e.g., less than 20 cm), the
        recorded object distance will be larger than the actual distance
        for distances over 50 cm because the lidar beam width smooths
        the edges of the object.

        Example
        -------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 120 degree counterclockwise scan and use the returned
        Scan to detect objects. In this case the Scan returned from the
        scan is passed directly into the .detect_objects() method. The
        returned object lists are stored as local variables as shown.
        In this example two objects are detected.

        >>> obj_angle, obj_dist, obj_width = robot.detect_objects(robot.scan(120))
        >>> obj_angle
        [42.69345, 83.32172]
        >>> obj_dist
        [35.8, 44.3]
        >>> obj_width
        [12.6, 31.8]

        """

        # check for valid arguments
        if not isinstance(scan, Scan):
            return print('Error: scan must be a Scan object')
        if len(scan) < 3:
            return print('Error: scan must have at least 3 points')
        if filename and not isinstance(filename, str):
            return print('Error: filename must be a string')
        # initialize variables
        object_angle = []
        object_distance = []
        object_width = []
        # find objects in scan data
        objects = self._find_objects(scan)
        for obj in objects:
            object_angle.append(obj.center_angle)
            object_distance.append(obj.min_distance)
            object_width.append(obj.width)
        # create a CSV (comma-separated values) file and write object data
        if filename:
            file = open(f'{filename}.csv', 'w')
            # add a file header
            file.write('angle (degrees), distance (cm), width (cm)\n')
            # write each line of data
            for i in range(len(object_angle)):
                file.write(f'{object_angle[i]}, {object_distance[i]},'
                           +f' {object_width[i]}\n')
            file.close()
        return object_angle, object_distance, object_width

    def _find_objects(self, scan, max_angle=120, max_width=50,
                      max_skew=0.5):
        """Finds objects that meet angle, width, and skew criteria.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.
        max_angle : int, float, default=120
            The maximum angle in degrees between the edges of possible
            objects for selection as detected objects.
        max_width : int, float, default=50
            The maximum width in cm of possible objects for selection as
            detected objects.
        max_skew : int, float, default=0.5
            The maximum percentage difference in distance between
            adjacent edges of possible objects for selection as detected
            objects.

        Returns
        -------
        list of _DetectedObject
            A list of _DetectedObject instances representing the
            detected objects. Each one records the indices of the
            leading edge, local minimum, and trailing edge used to find
            the object, plus the resulting center angle, distance, and
            width (see the _DetectedObject class for details).

        """

        # initialize object variables
        angle = scan.angle
        distance = scan.distance
        beam_width = cnst.BEAM_ANGLE / 2
        poss_objects = []
        objects = []
        # get a non-wrapping continuous angle list
        ang_no_wrap = self._ang_no_wrap(angle)
        # find all of the extrema and filter out the small and repeat values
        extrema = self._extrema(distance)
        extrema_filt = self._extrema_filt(distance, extrema)
        minima = extrema[0]
        minima_filt = extrema_filt[0]
        maxima = extrema[1]
        maxima_filt = extrema_filt[1]
        # find possible objects that follow a max/min/max pattern
        for i in range(len(minima_filt)):
            for j in range(len(maxima_filt)-1):
                if maxima_filt[j] < minima_filt[i] < maxima_filt[j+1]:
                    poss_objects.append((maxima_filt[j], minima_filt[i],
                                              maxima_filt[j+1]))
        # calculate the derivative with respect to the non-wrapping angle
        der = self._lidar_derivative(ang_no_wrap, distance)
        # filter the possible objects
        for i in range(len(poss_objects)):
            # find minimum negative slope between first max and min
            min_slope = min(der[poss_objects[i][0]:poss_objects[i][1]])
            # find maximum positive slope between min and second max
            max_slope = max(der[poss_objects[i][1]:poss_objects[i][2]])
            # check for steep slopes at or exceeding threshold  
            if (min_slope <= -cnst.SLOPE_THRESHOLD
                    and max_slope >= cnst.SLOPE_THRESHOLD):
                # get indices of the steep slopes
                max_neg_slope = der.index(min_slope, poss_objects[i][0],
                                          poss_objects[i][1]) + 1
                max_pos_slope = der.index(max_slope, poss_objects[i][1],
                                          poss_objects[i][2]) - 1
                # find center angle as average angle of the steepest slopes
                center_angle = round((ang_no_wrap[max_neg_slope]
                                      + ang_no_wrap[max_pos_slope])/2, 1)
                # adjust the center angle value to a +/- 180 range
                while abs(center_angle) > 180:
                    if center_angle < 0:
                        center_angle += 360
                    else:
                        center_angle -= 360
                # find the included angle between max negative slope and min
                inc_ang_neg = abs(ang_no_wrap[poss_objects[i][1]]
                                  - ang_no_wrap[max_neg_slope])
                # adjust for lidar beam width
                inc_ang_neg -= beam_width
                if inc_ang_neg < 0:
                    inc_ang_neg = 0
                # find the included angle between max positive slope and min
                inc_ang_pos = abs(ang_no_wrap[max_pos_slope]
                                  - ang_no_wrap[poss_objects[i][1]])
                # adjust for lidar beam width
                inc_ang_pos -= beam_width
                if inc_ang_pos < 0:
                    inc_ang_pos = 0
                # find minimum distance to object using local minimum index
                min_distance = distance[poss_objects[i][1]]
                # calculate object width based on included angles and distance
                width = round(min_distance * (tan(radians(inc_ang_neg))
                                              + tan(radians(inc_ang_pos))), 1)
                # check skew of edges to see they are at about same distance
                skew = (abs(distance[max_pos_slope]-distance[max_neg_slope])
                        /((distance[max_pos_slope]+distance[max_neg_slope])/2))
                # final checks before storing object
                if (inc_ang_neg + inc_ang_pos <= max_angle
                        and width <= max_width
                        and skew <= max_skew):
                    objects.append(_DetectedObject(max_neg_slope,
                                                   poss_objects[i][1],
                                                   max_pos_slope,
                                                   center_angle,
                                                   min_distance, width))
        return objects

    @staticmethod
    def _ang_no_wrap(angle):
        """Converts the lidar angle data to a non-wrapping list.

        Parameters
        ----------
        angle : list
            The lidar angle data from a scan.

        Returns
        -------
        list
            A list of non-wrapping lidar angle values with no
            discontinuity at +/- 180 degrees.

        """

        wraps = 0
        ang_no_wrap = angle.copy()
        for i in range(len(angle) - 1):
            # check for +/- 180 crossing and set number and sign of wrap
            if abs(angle[i+1]-angle[i]) > 180:
                if angle[i+1] < 0:
                    wraps += 1
                elif angle[i+1] > 0:
                    wraps -= 1
            # adjust angle to create non-wrapping values
            if wraps != 0:
                ang_no_wrap[i+1] += 360 * wraps
        return ang_no_wrap

    @staticmethod
    def _extrema(distance):
        """Finds all local minima and maxima in the lidar scan distance.

        Parameters
        ----------
        distance : list
            The lidar distance data.

        Returns
        -------
        2-tuple of lists
            A pair of lists comprising the indices of all minima and
            maxima in the distance list.

        """

        # initialize variables to store extrema
        min_index = 0
        max_index = 0
        minima = []
        maxima = []
        # compare all adjacent distance values
        for i in range(len(distance)-1):
            # look for and store local minima
            if distance[i+1] < distance[i]:
                min_index = i+1
            elif min_index != 0 and min_index == i:
                minima.append(min_index)
            # store first point as maxima if flat or descending
            if i == 0 and distance[i+1] <= distance[i]:
                maxima.append(max_index)
            # look for and store local maxima
            if distance[i+1] > distance[i]:
                max_index = i+1
            elif max_index != 0 and max_index == i:
                maxima.append(max_index)
        # store last point as maxima if flat or ascending
        if distance[-1] >= distance[-2]:
            maxima.append(len(distance)-1)
        return minima, maxima

    @staticmethod
    def _extrema_filt(distance, extrema, threshold=5):
        """Filters local minima and maxima in the lidar scan distance.

        Parameters
        ----------
        distance : list
            The lidar distance data.
        extrema : 2-tuple of lists
            The indices of the minima and maxima of the distance values.
        threshold : int, float, default=5
            The minimum size in cm of local minima or maxima to retain.

        Returns
        -------
        2-tuple of lists
            A pair of lists comprising the filtered minima and maxima
            indices after removing the small spikes from noise.

        """

        # initialize variables to filter extrema
        minima = extrema[0]
        maxima = extrema[1]
        minima_filt = []
        maxima_filt = []
        backward = False
        forward = False
        # test all local minima for size greater than or equal to threshold
        for i in range(len(minima)):
            # work backwards from each minima
            index = minima[i] - 1
            while index >= 0 and distance[index] >= distance[minima[i]]:
                if distance[index] >= distance[minima[i]] + threshold:
                    backward = True
                    break
                index -= 1
            # work forwards from each minima
            index = minima[i] + 1
            while (index <= len(distance)-1
                   and distance[index] >= distance[minima[i]]):
                if distance[index] >= distance[minima[i]] + threshold:
                    forward = True
                    break
                index += 1
            # store any minima that meet threshold requirement
            if backward and forward:
                minima_filt.append(minima[i])
            backward = False
            forward = False
        # test all local maxima for size greater than or equal to threshold
        for i in range(len(maxima)):
            # work backwards from each maxima
            index_back = maxima[i] - 1
            while (index_back >= 0
                   and distance[index_back] <= distance[maxima[i]]):
                if distance[index_back] <= distance[maxima[i]] - threshold:
                    backward = True
                    break
                index_back -= 1
            # work forwards from each maxima
            index_for = maxima[i] + 1
            while (index_for <= len(distance)-1
                   and distance[index_for] <= distance[maxima[i]]):
                if distance[index_for] <= distance[maxima[i]] - threshold:
                    forward = True
                    break
                index_for += 1
            # store any maxima that meet threshold requirement
            first_instance = None
            if backward and forward:
                maxima_filt.append(maxima[i])
            # store maxima near start of list
            elif not backward and forward and index_back == -1:
                if len(maxima_filt) == 0:
                    maxima_filt.append(maxima[i])
                elif distance[maxima_filt[0]] < distance[maxima[i]]:
                    maxima_filt.pop()
                    maxima_filt.append(maxima[i])
            # store maxima near end of list
            elif backward and not forward and index_for == len(distance):
                if first_instance is None:
                    first_instance = maxima[i]
                    maxima_filt.append(maxima[i])
                elif (distance[maxima_filt[-1]] < distance[maxima[i]]
                      and maxima[i] > first_instance):
                    maxima_filt.pop()
                    maxima_filt.append(maxima[i])
            backward = False
            forward = False
        # eliminate duplicate adjacent minima without maxima in between
        i = 0
        while i < len(minima_filt) - 1:
            duplicate = True
            if distance[minima_filt[i]] == distance[minima_filt[i+1]]:
                for j in range(len(maxima_filt)):
                    if minima_filt[i] < maxima_filt[j] < minima_filt[i+1]:
                        duplicate = False
                        break
                if duplicate:
                    minima_filt.pop(i+1)
                    i -= 1
            i += 1
        # eliminate duplicate adjacent maxima without minima in between
        i = 0
        while i < len(maxima_filt) - 1:
            duplicate = True
            if distance[maxima_filt[i]] == distance[maxima_filt[i+1]]:
                for j in range(len(minima_filt)):
                    if maxima_filt[i] < minima_filt[j] < maxima_filt[i+1]:
                        duplicate = False
                        break
                if duplicate:
                    maxima_filt.pop(i+1)
                    i -= 1
            i += 1
        return minima_filt, maxima_filt

    @staticmethod
    def _lidar_derivative(angle, distance):
        """Finds derivative of lidar distance with respect to angle.

        Parameters
        ----------
        angle : list
            The non-wrapping lidar angle data
        distance : list
            The lidar distance data

        Returns
        -------
        list
            A list of derivative values.

        Notes
        -----
        The first value uses the forward difference calculation,
        intermediate values use the central difference, and the last
        value uses the backward difference. If the difference in angle
        is zero, the derivative is calculated as the average of the two
        adjacent derivative values. If the difference in angle is zero
        at the start/end of the list, then the derivative is set equal
        to the adjacent value.

        Because the angle steps are non-uniform based on how the robot
        scan is performed, it is possible to have angle steps that are
        very small compared to the desired step, causing large
        uncertainty in the derivative calculation. In cases where the
        angle difference is less than 10% of the desired step, the
        same technique for dealing with a zero angle difference is
        employed to prevent large spikes in the derivative values.

        """

        lidar_derivative = []
        # calculate the average angle step from angle start and end values
        angle_step = abs((angle[-1]-angle[0])/(len(angle)-1))
        # for first point use forward difference calculation
        if angle[1]-angle[0] == 0 or abs(angle[1]-angle[0]) < 0.1 * angle_step:
            lidar_derivative.append(None)
        else:
            lidar_derivative.append((distance[1]-distance[0])
                                    /abs(angle[1]-angle[0]))
        # for intermediate points use central difference calculation
        for i in range(1, len(distance)-1):
            if (angle[i+1]-angle[i-1] == 0
                    or abs(angle[i+1]-angle[i-1]) < 0.1 * angle_step):
                lidar_derivative.append(None)
            else:
                lidar_derivative.append((distance[i+1]-distance[i-1])
                                        /abs(angle[i+1]-angle[i-1]))
        # for last point use backward difference calculation
        if (angle[-1]-angle[-2] == 0
                or abs(angle[-1]-angle[-2]) < 0.1 * angle_step):
            lidar_derivative.append(None)
        else:
            lidar_derivative.append((distance[-1]-distance[-2])
                                    /abs(angle[-1]-angle[-2]))
        # fill in all values that could not be calculated
        for i in range(len(lidar_derivative)):
            if lidar_derivative[i] is None:
                # if it's the first value, set it to nearest adjacent
                if i == 0:
                    j = 1
                    while lidar_derivative[j] is None:
                        j += 1
                        if j == len(lidar_derivative)-1:
                            break
                    lidar_derivative[i] = lidar_derivative[j]
                # if it's an intermediate value, use an adjacent or average
                elif 0 < i < len(lidar_derivative)-2:
                    if lidar_derivative[i+1] is None:
                        lidar_derivative[i] = lidar_derivative[i-1]
                    else:
                        lidar_derivative[i] = (lidar_derivative[i-1]
                                               + lidar_derivative[i+1])/2
                # if it's the last value, use the nearest adjacent
                else:
                    lidar_derivative[i] = lidar_derivative[i-1]
        return lidar_derivative
    
    def _wedge_areas(self, scan):
        """Calculates the area of each wedge in the scan data.

        Parameters
        ----------
        scan : Scan
            The scan data, as returned by .scan() or stored in
            .lidar_scan.

        Returns
        -------
        list
            The area of each wedge defined by two adjacent lidar scan
            distances and the angle between them.

        """

        wedge_areas = []
        distance = scan.distance
        # get the non-wrapping angle values to use in finding angle increments
        ang_no_wrap = self._ang_no_wrap(scan.angle)
        # use are of triangle formula to find area of each wedge of lidar data
        for i in range(len(distance)-1):
            wedge_areas.append(0.5 * distance[i] * distance[i+1]
                               * sin(radians(abs(ang_no_wrap[i+1]
                                                 -ang_no_wrap[i]))))
        return wedge_areas