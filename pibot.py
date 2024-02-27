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
from math import pi, tan, radians
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
        # ensure the starting angle is positive
        if start_angle < 0:
            start_angle += 360
        # clear lidar data before a new scan
        self.lidar_dist.clear()
        self.lidar_angle.clear()
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
            if angle > 0 and current_heading > desired_heading:
                self.lidar_dist.append(self.lidar.read())
                self.lidar_angle.append(self.heading)
                i += 1
            elif angle < 0 and current_heading < desired_heading:
                self.lidar_dist.append(self.lidar.read())
                self.lidar_angle.append(self.heading)
                i -= 1
        # add one more data point at the end of rotation after settling
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

    def detect_objects(self, angle, distance, filename=None):
        """Attempts to detect one or more objects in the foreground.

        Parameters
        ----------
        angle : list
            The lidar angle data
        distance : list
            The lidar distance data
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
        Attempts to find objects in the foreground of lidar scan data by
        detecting sharp changes in distance readings that represent
        possible leading and trailing edges of an object. If both a
        sharp decrease in distance (i.e., a step from far to near
        representing a leading edge) and then a corresponding sharp
        increase (i.e., a step from near to far representing a trailing
        edge) are detected, an object is recorded.

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
        
        Note
        ----
        For fast scans, the lidar readings lag the rotation, which leads
        to a slight angular offset in the recorded center angle of the
        detected object. Slower scans provided must better angle
        accuracy. For narrow objects (e.g., below 20 cm wide), the
        recorded object distance will be larger than the actual distance
        for distances over 50 cm because the lidar beam width smooths
        the edges of the object.

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
        if filename and not isinstance(filename, str):
            return print('Error: filename must be a string')
        # initialize variables
        object_angle = []
        object_distance = []
        object_width = []
        # find objects in scan data
        objects = self._find_objects(angle, distance)
        for i in range(len(objects)):
            object_angle.append(objects[i][3])
            object_distance.append(objects[i][4])
            object_width.append(objects[i][5])
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

    def _find_objects(self, angle, distance, max_angle=120, max_width=50,
                      max_skew=0.5):
        """Finds objects that meet angle, width, and skew criteria.

        Parameters
        ----------
        angle : list
            The lidar angle data
        distance : list
            The lidar distance data
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
        list of tuples
            A list of tuples representing the detected objects. Each
            tuple consists of 6 elements. The first three elements are
            the indices of the leading edge, local minima, and trailing
            edge of the objects. The last three values are the center
            angle, distance, and width of the object.

        """

        # initialize object variables
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
                    objects.append((max_neg_slope, poss_objects[i][1],
                                    max_pos_slope, center_angle, min_distance,
                                    width))
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
        # store last point if needed
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
            A pair of lists comprising the filter minima and maxima
            indices after removing the small spikes from noise.

        """

        # initialize variables to filter extrema
        minima = extrema[0]
        maxima = extrema[1]
        minima_filt = []
        maxima_filt = []
        index = 0
        left = False
        right = False
        # test all local minima for size greater than or equal to threshold
        for i in range(len(minima)):
            # work backwards from each minima
            index = minima[i] - 1
            while index >= 0 and distance[index] >= distance[minima[i]]:
                if distance[index] >= distance[minima[i]] + threshold:
                    left = True
                    break
                index -= 1
            # work forwards from each minima
            index = minima[i] + 1
            while (index <= len(distance)-1
                   and distance[index] >= distance[minima[i]]):
                if distance[index] >= distance[minima[i]] + threshold:
                    right = True
                    break
                index += 1
            # store any minima that meet threshold requirement
            if left and right:
                minima_filt.append(minima[i])
            left = False
            right = False
        # test all local maxima for size greater than or equal to threshold
        for i in range(len(maxima)):
            # work backwards from each maxima
            left_index = maxima[i] - 1
            while (left_index >= 0
                   and distance[left_index] <= distance[maxima[i]]):
                if distance[left_index] <= distance[maxima[i]] - threshold:
                    left = True
                    break
                left_index -= 1
            # work forwards from each maxima
            right_index = maxima[i] + 1
            while (right_index <= len(distance)-1
                   and distance[right_index] <= distance[maxima[i]]):
                if distance[right_index] <= distance[maxima[i]] - threshold:
                    right = True
                    break
                right_index += 1
            # store any maxima that meet threshold requirement
            first_instance = None
            if left and right:
                maxima_filt.append(maxima[i])
            # store maxima near start of list
            elif not left and right and left_index == -1:
                if len(maxima_filt) == 0:
                    maxima_filt.append(maxima[i])
                elif distance[maxima_filt[0]] < distance[maxima[i]]:
                    maxima_filt.pop()
                    maxima_filt.append(maxima[i])
            # store maxima near end of list
            elif left and not right and right_index == len(distance):
                if first_instance is None:
                    first_instance = maxima[i]
                    maxima_filt.append(maxima[i])
                elif (distance[maxima_filt[-1]] < distance[maxima[i]]
                      and maxima[i] > first_instance):
                    maxima_filt.pop()
                    maxima_filt.append(maxima[i])
            left = False
            right = False
        # eliminate duplicate adjacent minima without maxima in between
        i = 0
        while i < len(minima_filt) - 1:
            if distance[minima_filt[i]] == distance[minima_filt[i+1]]:
                j = 0
                while j < len(maxima_filt):
                    if minima_filt[i] < maxima_filt[j] < minima_filt[i+1]:
                        i += 1
                        break
                    elif j == len(maxima_filt) - 1:
                        minima_filt.pop(i+1)
                        break
                    else:
                        j += 1
            else:
                i += 1
        # eliminate duplicate adjacent maxima without minima in between
        i = 0
        while i < len(maxima_filt) - 1:
            if distance[maxima_filt[i]] == distance[maxima_filt[i+1]]:
                j = 0
                while j < len(minima_filt):
                    if maxima_filt[i] < minima_filt[j] < maxima_filt[i+1]:
                        i += 1
                        break
                    elif j == len(minima_filt) - 1:
                        maxima_filt.pop(i+1)
                        break
                    else:
                        j += 1
            else:
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