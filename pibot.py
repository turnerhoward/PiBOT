"""

This module contains the PiBOT class that incorporates the control,
motion, sensors, and outputs modules to create a fully functioning
robot. The PiBOT class contains attributes to track the robot position
and heading, determine its current state, perform lidar scans of the
surroundings, and work with the lidar data. The examples in the
following section demonstrate how to use the PiBOT attributes. The
motion, sensors, and outputs modules also contain many examples of how
to use their features from within an instance of the PiBOT class.

"""

import constants as cnst
from math import pi, sin, radians
from utime import sleep_ms
from control import Control
from motion import Motion
from sensors import Whiskers, IrSensors, LidarSensor, IrRemote
from outputs import Buzzer, LEDs


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
    lidar_dist : list
        The lidar distance values stored when scanning.
    lidar_angle : list
        The headings (angle values) when scanning with lidar.
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
        # ATTRIBUTES FOR LIDAR SCAN
        self.lidar_dist = []
        self.lidar_angle = []

    @property
    def position(self):
        """Gets or sets the current position as a list of [x, y]
        coordinates.

        Notes
        -----
        The global coordinate system is based on the position of the
        centerpoint between the robot's wheels when motion is first
        started or after resetting. The x axis points forward along the
        path, and the y axis points to the left wheel.

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
        while self._control._tracking_lock:
            continue
        # return a static copy of the list instead of the list object itself
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
        while self._control._tracking_lock:
            continue
        self._control._position = value

    @property
    def heading(self):
        """Gets or sets the current heading angle.

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
        while self._control._tracking_lock:
            continue
        return self._control._heading * (180/pi)

    @heading.setter
    def heading(self, value):
        # check for valid argument
        if not isinstance(value, (int, float)):
            return print('Error: heading must be a numeric value in degrees')
        elif value < -180 or value > 180:
            return print('Error: heading must be in the range +/-180 degrees')
        # wait to avoid setting new value while _tracking() method is active
        while self._control._tracking_lock:
            continue
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
        """Determines is the robot is currently moving.

        Checks for motion states: 'stop' or 'pause'.

        """

        if self.motion_state in ('stop', 'pause'):
            return False
        else:
            return True

    @property
    def busy(self):
        """Determines if the robot has protected or queued commands.

        The robot is considered busy if protected motion is being
        commanded or if commands are waiting in the queue.

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
        ang_speed : int or float, default=cnst.ANG_SPD_MAX (180 deg/s)
            The desired angular speed. Range from 30 to 180 deg/s.
        filename : str, optional
            Name of the file to save the lidar data to a CSV.

        Returns
        -------
        2-tuple of lists
            The corresponding angle and distance values, which can
            also be accessed through the .lidar_angle and .lidar_dist
            attributes.

        Notes
        -----
        The default angle increment of the lidar scan is set to 2.5
        degrees, which represents a good compromise between speed and
        resolution when scanning at the default maximum angular speed.
        The actual angle stored in .lidar_angle varies because the data
        is collected dynamically while the robot rotates. The lidar
        detects a field of view of about 15 degrees, meaning the data
        spaced at 2.5 degrees represents a higher resolution than the
        lidar can really discern and results in overlapping ranges of
        detection. In practice, the lidar sensor is fairly accurate at
        measuring distances to objects that are large and flat, but for
        small objects that are near the end of its measuring range, the
        sensor will report larger values than expected, making the
        object appear further away than it really is. In some cases, an
        object can be missed entirely because the IR light from the
        sensor's 15 degree beam width reflects from the background and
        overwhelms the light reflected from the small object in the
        foreground. The best approach for scanning small objects is to
        move close to them.

        The optional filename is used to save a CSV file to the RP2040's
        memory, which can be accessed with Thonny and downloaded to the
        computer for analysis.

        Important
        ---------
        Remember that the lidar readings have an offset added so that
        the distances recorded are from the center of the robot's
        wheel base. Therefore, the data in the lidar distance list of a
        scan represents the radius from the robot's center of rotation.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 180 degree clockwise scan. The returned lists are
        stored in local variables as shown; however, it is not
        necessary to use the returned values because they can also
        be accessed directly with the .lidar_dist and .lidar_angle
        attributes.

        >>> angle, distance = robot.scan(-180)

        Command a 180 degree counterclockwise scan and store the data
        to a file called 'test_data'. The data is not saved to local
        variables. In the output, it is clear that the default angle
        increment of 2.5 degrees cannot be followed exactly but tracks
        fairly close to the desired step size.

        >>> robot.scan(180, filename='test_data')
        ([0.0, 2.713043, 6.052173, 8.869564, 10.01739, 12.52174,...

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
        # clear lidar data before a new scan
        self.lidar_dist.clear()
        self.lidar_angle.clear()
        # start protected rotation at the specified angle and direction
        if angle > 0:
            if start_angle < 0:
                start_angle += 360
            self.move.rotate_left(angle, ang_speed, protect=True)
        else:
            if start_angle > 0:
                start_angle -= 360
            self.move.rotate_right(-angle, ang_speed, protect=True)
        # wait for rotation to start before collecting data
        while self._control._motion_state != 'rotate':
            continue
        # while rotation is active, collect data at the approximate increment
        while self._control._motion_state == 'rotate':
            # for positive rotation, adjust for +/-180 crossover
            if angle > 0:
                # ensure current heading is always positive
                if self.heading < 0:
                    current_heading = self.heading + 360
                else:
                    current_heading = self.heading
                # increment desired angle and ensure it doesn't exceed 360
                desired_heading = start_angle + increment*i
                if desired_heading > 360:
                    desired_heading -= 360
                # store data when the current heading reaches the desired
                if current_heading >= desired_heading:
                    self.lidar_dist.append(self.lidar.read())
                    self.lidar_angle.append(self.heading)
                    i += 1
            # for negative rotation, adjust for +/-180 crossover
            elif angle < 0:
                # ensure current heading is always negative
                if self.heading > 0:
                    current_heading = self.heading - 360
                else:
                    current_heading = self.heading
                # decrement desired angle and ensure it doesn't exceed -360
                desired_heading = start_angle - increment*i
                if desired_heading < -360:
                    desired_heading += 360
                # store data when the current heading reaches the desired
                if current_heading <= desired_heading:
                    self.lidar_dist.append(self.lidar.read())
                    self.lidar_angle.append(self.heading)
                    i += 1
        # add one more data point at the end of rotation for settling time
        self.lidar_dist.append(self.lidar.read())
        self.lidar_angle.append(self.heading)
        # create a CSV (comma-separated values) file and write lidar data
        if filename:
            file = open(f'{filename}.csv', 'w')
            # add a file header
            file.write('angle (degrees), distance (cm)\n')
            # write each line of data
            for i in range(len(self.lidar_angle)):
                file.write(f'{self.lidar_angle[i]}, {self.lidar_dist[i]}\n')
            file.close()
        return self.lidar_angle, self.lidar_dist

    def max_distance(self, angle, distance):
        """Finds the maximum value in the lidar scan data.

        Parameters
        ----------
        angle : list
            The lidar angle data
        distance : list
            The lidar distance data

        Returns
        -------
        2-tuple
            A pair comprising the heading angle and maximum distance.

        Notes
        -----

        Finds the maximum value and corresponsing angle in scan data. If
        there is more than one identical maximum value, the first one in
        the list will be returned. If there are values that are out of
        range (i.e., 140 cm), the method finds the block of 140 cm
        values in the distance list that is longest and returns the
        angle near the center of the block. If there is only one 140 cm
        value, that will be returned as the maximum. If there is more
        than one non-adjacent single 140 cm value, the first one in the
        list will be returned as the maximum.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 180 degree clockwise scan and use the returned values
        to find the maximum.

        >>> angle, distance = robot.scan(-180)
        >>> robot.max_distance(angle, distance)
        (128.4522, 140.0)

        The arguments can also be the lidar data taken directly from the
        PiBOT attributes instead of local copies used in the previous
        example.

        >>> robot.scan(-180)
        ([0.0, -2.504347, -5.321739, -7.61739, -10.53913, -12.93913,...
        >>> robot.max_distance(robot.lidar_angle, robot.lidar_dist)
        (-80.34782, 91.0)

        The scan can also return its values directly to the max_distance
        method using the unpacking operator (*) as shown.

        >>> robot.max_distance(*robot.scan(-180))
        (-78.78259, 103.5)

        """

        # check for valid arguments
        if not isinstance(angle, list) and len(angle) < 3:
            return print('Error: angle must be a list'
                         + ' with at least 3 elements')
        if not all(isinstance(x, (int, float)) for x in angle):
            return print('Error: angle values must be numeric')
        if not isinstance(distance, list) and len(angle) < 3:
            return print('Error: distance must be a list'
                         + ' with at least 3 elements')
        if not all(isinstance(x, (int, float)) for x in distance):
            return print('Error: distance values must be numeric')
        # create count variables for out-of-range values
        count = 0
        max_count = 0
        out_of_range = []
        # create list of all out-of-range values in lidar data (i.e., 140 cm)
        for i in range(distance.count(140)):
            if i == 0:
                out_of_range.append(distance.index(140))
            else:
                out_of_range.append(distance.index(140, out_of_range[i-1] + 1))
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

    def min_distance(self, angle, distance):
        """Finds the minimum value in the lidar scan data.

        Parameters
        ----------
        angle : list
            The lidar angle data
        distance : list
            The lidar distance data

        Returns
        -------
        2-tuple
            A pair comprising the heading angle and minimum distance.

        Notes
        -----
        Finds the minimum value and corresponsing angle in scan data. If
        there is more than one identical minimum value, the first one in
        the list will be returned.

        Examples
        --------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 180 degree clockwise scan and use the returned values
        to find the minimum.

        >>> angle, distance = robot.scan(-180)
        >>> robot.min_distance(angle, distance)
        (72.974, 18.2)

        The arguments can also be the lidar data taken directly from the
        PiBOT attributes instead of local copies used in the previous
        example.

        >>> robot.scan(-180)
        ([0.0, -2.713043, -5.634782, -7.721738, -10.22609, -12.83478,...
        >>> robot.min_distance(robot.lidar_angle, robot.lidar_dist)
        (-120.0, 13.8)

        The scan can also return its values directly to the min_distance
        method using the unpacking operator (*) as shown.

        >>> robot.min_distance(*robot.scan(-180))
        (-179.9652, 19.6)

        """

        # check for valid arguments
        if not isinstance(angle, list) and len(angle) < 3:
            return print('Error: angle must be a list'
                         + ' with at least 3 elements')
        if not all(isinstance(x, (int, float)) for x in angle):
            return print('Error: angle values must be numeric')
        if not isinstance(distance, list) and len(angle) < 3:
            return print('Error: distance must be a list'
                         + ' with at least 3 elements')
        if not all(isinstance(x, (int, float)) for x in distance):
            return print('Error: distance values must be numeric')
        # find index of minimum value in distance list
        min_index = distance.index(min(distance))
        return angle[min_index], distance[min_index]

    def detect_objects(self, angle, distance, max_size=50):
        """Attempts to detect one or more objects in the foreground.

        Parameters
        ----------
        angle : list
            The lidar angle data
        distance : list
            The lidar distance data
        max_size : int, float, default=50
            Maximum width of an object in cm to detect.

        Returns
        -------
        3-tuple of lists
            A triplet of lists comprising the approximate heading angles
            in degrees to the center of the objects, the minimum
            distances in cm to the objects, and their approximate widths
            in cm.

        Notes
        -----
        Attempts to find objects in the foreground of lidar scan data by
        detecting sharp changes in distance readings that represent
        possible leading and trailing edges of an object. If both a
        sharp decrease in distance (i.e., a step from far to near
        representing a leading edge) and then a corresponding sharp
        increase (i.e., a step from near to far representing a trailing
        edge) are detected, an object is recorded.

        The algorithm first calculates the derivative of the distance
        with respect to angle to find points where the distance is
        changing most rapidly. The negative and positive peaks of the
        derivative data are recorded as possible leading and trailing
        edges. Then the peaks are filtered to ensure they meet a minimum
        threshold and that the derivative of the three points preceeding
        or following the peak is fairly flat in comparision. The intent
        of this simple filtering strategy is to find only the peaks
        where a rapid change is occurring. Finally, the potential
        leading and trailing edges are tested in sequence to determine
        if there are pairs of first a leading and then a trailing edge.
        From all pairs found, the edges are used to calculate the center
        angle, minimum distance, and approximate width of each object,
        which are stored in the corresponsing list. If no objects are
        found, empty lists are returned.

        Important
        ---------
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

        Example
        -------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Command a 120 degree counterclockwise scan and use the returned
        values to detect objects. In this case the unpacking operator is
        used to pass the angle and distance lists returned from the scan
        directly into the .detect_objects() method. The returned object
        lists are stored as local variables as shown. In this example
        two objects are detected.

        >>> obj_angle, obj_dist, obj_width = robot.detect_objects(*robot.scan(120, ang_speed=120))
        >>> obj_angle
        [42.69345, 83.32172]
        >>> obj_dist
        [35.8, 44.3]
        >>> obj_width
        [12.6, 31.8]

        """

        # check for valid arguments
        if not isinstance(angle, list) and len(angle) < 3:
            return print('Error: angle must be a list'
                         + ' with at least 3 elements')
        if not all(isinstance(x, (int, float)) for x in angle):
            return print('Error: angle values must be numeric')
        if not isinstance(distance, list) and len(angle) < 3:
            return print('Error: distance must be a list'
                         + ' with at least 3 elements')
        if not all(isinstance(x, (int, float)) for x in distance):
            return print('Error: distance values must be numeric')
        if not isinstance(max_size, (int, float)):
            return print('Error: max_size must be a numeric value')
        if max_size < 0:
            return print('Error: max_size cannot be less than zero')
        # initialize variables
        BEAM_ANGLE = 5 # approximate width of lidar beam in degrees
        object_angle = []
        object_distance = []
        object_width = []
        # create a non-wrapping angle list (i.e., no +/- 180 wrap-around)
        ang_no_wrap = self._ang_no_wrap(angle)
        # calculate the derivative with respect to the non-wrapping angle
        derivative = self._lidar_derivative(ang_no_wrap, distance)
        # the peaks and possible leading and trailing edges
        negative_peaks = self._negative_peaks(derivative)
        positive_peaks = self._positive_peaks(derivative)
        leading_edges = self._leading_edges(negative_peaks, derivative)
        trailing_edges = self._trailing_edges(positive_peaks, derivative)
        # find adjacent pairs of leading and trailing edges
        while len(leading_edges) > 0 and len(trailing_edges) > 0:
            # pop the first leading edge value out of the list
            leading_edge = leading_edges.pop(0)
            # check for corresponding trailing edges
            while len(trailing_edges) > 0:
                # trailing edge must be after leading and before next leading
                if (trailing_edges[0] >= leading_edge
                        and (len(leading_edges) == 0
                             or trailing_edges[0] < leading_edges[0])):
                    trailing_edge = trailing_edges.pop(0)
                    # adjacent pair indicates object was found
                    leading_edge_angle = ang_no_wrap[leading_edge]
                    trailing_edge_angle = ang_no_wrap[trailing_edge]
                    center_angle = (leading_edge_angle + trailing_edge_angle)/2
                    # adjust for +/- 180 wrapped values
                    if abs(center_angle) > 180:
                        if center_angle < 0:
                            center_angle += 360
                        else:
                            center_angle -= 360
                    included_angle = abs(trailing_edge_angle
                                         - leading_edge_angle)
                    # adjust for beam width
                    if included_angle > BEAM_ANGLE:
                        included_angle = included_angle - BEAM_ANGLE
                    else:
                        included_angle = 5
                    # find the minimum object distance
                    min_index = distance.index(min(distance[leading_edge
                                                            :trailing_edge]))
                    min_distance = distance[min_index]
                    # calculate approximate object width
                    width = min_distance * sin(radians(included_angle))
                    # ensure object width doesn't exceed max_width and append
                    if width <= max_size:
                        # add object data to the corresponding lists
                        object_angle.append(center_angle)
                        object_distance.append(min_distance)
                        object_width.append(round(width, 1))
                    break
                # if trailing edge is past next leading edge, go back
                elif (trailing_edges[0] > leading_edge
                          and len(leading_edges) != 0
                          and trailing_edges[0] > leading_edges[0]):
                    break
                # otherwise toss out the trailing edge and keep checking
                else:
                    trailing_edge = trailing_edges.pop(0)
        return object_angle, object_distance, object_width

    @staticmethod
    def _ang_no_wrap(angle):
        """Converts the lidar angle data to a non-wrapping list.

        Parameters
        ----------
        angle : list
            The lidar angle data from a scan

        Returns
        -------
        list
            A list of non-wrapping lidar angle values.

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
            # adjust angle for non-wrapping values
            if wraps != 0:
                ang_no_wrap[i+1] += 360 * wraps
        return ang_no_wrap

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

    @staticmethod
    def _negative_peaks(lidar_derivative):
        """Finds all negative peaks in the lidar derivative list.

        Parameters
        ----------
        lidar_derivative : list
            The data calculated with the _lidar_derivative function

        Returns
        -------
        list
            A list of indices representing all negative peak values

        """

        # initialize variables to store peaks
        peak_index = 0
        negative_peaks = []
        # look for local minima that are negative and below threshold
        for i in range(len(lidar_derivative)-1):
            if (lidar_derivative[i+1] < lidar_derivative[i]
                    and lidar_derivative[i+1] < -cnst.MIN_THRESHOLD):
                peak_index = i+1
            # store the negative peak
            elif peak_index != 0 and peak_index == i:
                negative_peaks.append(peak_index)
        # store last point if descending and below threshold
        if (lidar_derivative[-1] < lidar_derivative[-2]
                and lidar_derivative[-1] < -cnst.MIN_THRESHOLD):
            negative_peaks.append(len(lidar_derivative)-1)
        return negative_peaks

    @staticmethod
    def _positive_peaks(lidar_derivative):
        """Finds all positive peaks in the lidar derivative list.

        Parameters
        ----------
        lidar_derivative : list
            The data calculated with the _lidar_derivative function

        Returns
        -------
        list
            A list of indices representing all positive peak values

        """

        # initialize variables to store peaks
        peak_index = 0
        positive_peaks = []
        # look for local maxima that are positive and above threshold
        for i in range(len(lidar_derivative)-1):
            if (lidar_derivative[i+1] > lidar_derivative[i]
                    and lidar_derivative[i+1] > cnst.MIN_THRESHOLD):
                peak_index = i+1
            # store the positive peak
            elif peak_index != 0 and peak_index == i:
                positive_peaks.append(peak_index)
        # store last point if ascending and above threshold
        if (lidar_derivative[-1] > lidar_derivative[-2]
                and lidar_derivative[-1] > cnst.MIN_THRESHOLD):
            positive_peaks.append(len(lidar_derivative)-1)
        return positive_peaks

    @staticmethod
    def _leading_edges(negative_peaks, derivative):
        """Finds leading edges of possible objects in foreground.

        Parameters
        ----------
        negative_peaks : list
            An index list of negative peaks in the lidar derivative list
        derivative : list
            The derivative of the lidar distance with respect to angle

        Returns
        -------
        list
            A list of indices in the lidar data for points with steep
            slopes compared to the surrounding points that represent
            possible leading edges.

        """

        # initialize variables to test for edges and store their indices
        start = False
        end = False
        leading_edges = []
        # get the start and stop indices of each peak
        for i in range(len(negative_peaks)):
            start_index = negative_peaks[i]
            end_index = negative_peaks[i]
            # work backwards to include points in peak above threshold
            index = negative_peaks[i] - 1
            while (index >= 0
                   and (derivative[index]
                        /derivative[negative_peaks[i]]) >= cnst.PEAK_THRESHOLD
                   and derivative[index] > derivative[negative_peaks[i]]):
                start_index = index
                index -= 1
            # work forwards to include points in peak above threshold
            index = negative_peaks[i] + 1
            while (index <= len(derivative)-1
                   and (derivative[index]
                        /derivative[negative_peaks[i]]) >= cnst.PEAK_THRESHOLD
                   and derivative[index] > derivative[negative_peaks[i]]):
                end_index = index
                index += 1
            # test for peak above threshold of average points to left
            index = start_index - 1
            average = 0
            j = 0
            while index >= 0 and j < cnst.NUM_ADJACENT:
                average += derivative[index]
                index -= 1
                j += 1
            if j != 0:
                average = average / j
                if average/derivative[negative_peaks[i]] < cnst.ADJ_THRESHOLD:
                    start = True
            else:
                start = True
            # test for peak above threshold of average points to right
            index = end_index + 1
            average = 0
            j = 0
            while index <= len(derivative)-1 and j < cnst.NUM_ADJACENT:
                average += derivative[index]
                index += 1
                j += 1
            if j != 0:
                average = average / j
                if average/derivative[negative_peaks[i]] < cnst.ADJ_THRESHOLD:
                    end = True
            else:
                end = True
            # check for threshold passed at start and end of potential edge
            if start and end:
                leading_edges.append(negative_peaks[i])
                start = False
                end = False
        return leading_edges

    @staticmethod
    def _trailing_edges(positive_peaks, derivative):
        """Finds trailing edges of possible objects in foreground.

        Parameters
        ----------
        positive_peaks : list
            An index list of positive peaks in the lidar derivative list
        derivative : list
            The derivative of the lidar distance with respect to angle

        Returns
        -------
        list
            A list of indices in the lidar data for points with steep
            slopes compared to the surrounding points that represent
            possible trailing edges.

        """

        # initialize variables to test for edges and store their indices
        start = False
        end = False
        trailing_edges = []
        # get the start and stop indices of each peak
        for i in range(len(positive_peaks)):
            start_index = positive_peaks[i]
            end_index = positive_peaks[i]
            # work backwards to include points in peak above threshold
            index = positive_peaks[i] - 1
            while (index >= 0
                   and (derivative[index]
                        /derivative[positive_peaks[i]]) >= cnst.PEAK_THRESHOLD
                   and derivative[index] < derivative[positive_peaks[i]]):
                start_index = index
                index -= 1
            # work forwards to include points in peak above threshold
            index = positive_peaks[i] + 1
            while (index <= len(derivative)-1
                   and (derivative[index]
                        /derivative[positive_peaks[i]]) >= cnst.PEAK_THRESHOLD
                   and derivative[index] < derivative[positive_peaks[i]]):
                end_index = index
                index += 1
            # test for peak above threshold of average points to left
            index = start_index - 1
            average = 0
            j = 0
            while index >= 0 and j < cnst.NUM_ADJACENT:
                average += derivative[index]
                index -= 1
                j += 1
            if j != 0:
                average = average / j
                if average/derivative[positive_peaks[i]] < cnst.ADJ_THRESHOLD:
                    start = True
            else:
                start = True
            # test for peak above threshold of average points to right
            index = end_index + 1
            average = 0
            j = 0
            while index <= len(derivative)-1 and j < cnst.NUM_ADJACENT:
                average += derivative[index]
                index += 1
                j += 1
            if j != 0:
                average = average / j
                if average/derivative[positive_peaks[i]] < cnst.ADJ_THRESHOLD:
                    end = True
            else:
                end = True
            # check for threshold passed at start and end of potential edge
            if start and end:
                trailing_edges.append(positive_peaks[i])
                start = False
                end = False
        return trailing_edges