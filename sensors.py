import constants as cnst
from utime import ticks_us, ticks_diff, ticks_add, sleep_us
from machine import Pin, time_pulse_us
from rp2 import StateMachine, asm_pio, PIO


class Whiskers:
    """
    Configures whisker sensors.

    ...

    Notes
    -----
    The wire whiskers are contact sensors that must be pressed with
    sufficient force to detect an obstacle. The fixed end of each
    whisker connects to ground. When the free end of a whisker is
    deflected toward the robot, it makes electrical contact with a metal
    screw that is connected to a GPIO pin that is normally held high
    using the built-in pull-up resistor of the GPIO. When the grounded
    whisker wire touches the screw, the pin is pulled low.

    """

    def __init__(self):
        """ Initializes the whiskers with appropriate GPIO pins."""

        self._whisker_left = Pin(cnst.WHISKER_LEFT, Pin.IN, Pin.PULL_UP)
        self._whisker_right = Pin(cnst.WHISKER_RIGHT, Pin.IN, Pin.PULL_UP)

    @property
    def left(self):
        """Determines if left whisker is making contact.

        Return
        ------
        bool
            Indicates if the whisker is making contact (i.e., pin is low).

        Example
        -------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Test for the left whisker making contact and turn on the left LED.

        >>> while True:
        >>>     if robot.whiskers.left:
        >>>         robot.leds.on(robot.leds.RED, side='left')
        >>>     else:
        >>>         robot.leds.off(side='left')

        """

        if self._whisker_left.value():
            return False
        else:
            return True

    @property
    def right(self):
        """Determines if right whisker is making contact.

        Return
        ------
        bool
            Indicates if the whisker is making contact (i.e., pin is low).

        Example
        -------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Test for the right whisker making contact and turn on the right LED.

        >>> while True:
        >>>     if robot.whiskers.right:
        >>>         robot.leds.on(robot.leds.RED, side='right')
        >>>     else:
        >>>         robot.leds.off(side='right')

        """

        if self._whisker_right.value():
            return False
        else:
            return True


class IrSensors:
    """
    Configures infrared sensors.

    ...

    Notes
    -----
    Review documentation for Pololu 38 kHz IR Proximity Sensor, Fixed
    Gain, Low Brightness (Pololu item #: 2579):
    https://www.pololu.com/product/2579. The output is normally high
    until the sensor detects sufficient reflected IR to bring the pin
    low. The enable pin turns off the IR emitter when pulled low (it is
    normally high). The enable feature is used to prevent interference
    between the left and right IR sensors, ensuring that only one is
    active when a reading is taken. The lidar sensor and IR remote
    control can produce false triggering of the IR sensors because all
    of these devices emit pulses of infrared light at a modulated
    frequency of 38 kHz. To prevent this interference from the lidar
    sensor and remote control, the IR sensors are sampled over 100 ms
    and an average reading is recorded. This strategy effectively
    ignores all short pulses from the lidar and remote control and only
    triggers a reading with a sustained input of reflected IR light from
    the IR sensor's own emitter.

    """

    def __init__(self):
        """ Initializes the IR sensors with appropriate GPIO."""

        self._IR_left = Pin(cnst.IR_LEFT, Pin.IN)
        self._IR_right = Pin(cnst.IR_RIGHT, Pin.IN)
        self._IR_left_enable = Pin(cnst.IR_LEFT_ENABLE, Pin.OUT)
        self._IR_right_enable = Pin(cnst.IR_RIGHT_ENABLE, Pin.OUT)
        # disable IR sensors at startup
        self._IR_left_enable.low()
        self._IR_right_enable.low()
        # create attributes to store the state of IR sensors
        self._left = False
        self._right = False
        # create variables to measure the time average of the sensor readings
        self._reading_left = 0
        self._reading_right = 0
        self._count_left = 0
        self._count_right = 0
        self._stop_time_left = None
        self._stop_time_right = None

    @property
    def left(self):
        """Determines the current state of the left IR sensor.

        Return
        ------
        bool
            Indicates if the IR sensor is detecting an object.

        Note
        ----
        Because the read method for the IR sensors uses a time average
        to determine if the sensor is detecting and to reject spurious
        pulses of IR from other sources, the value must be checked
        frequently in a polling loop as shown in the following example.

        Example
        -------

        Create an instance of PiBOT.

        >>> from robot import PiBOT
        >>> robot = PiBOT()

        If the left IR sensor is detecting, turn on the left LED.

        >>> while True:
        >>>     if robot.ir.left:
        >>>         robot.leds.on(robot.leds.BLUE, side='left')
        >>>     else:
        >>>         robot.leds.off(side='left')

        """

        self._read('left')
        if self._left:
            return True
        else:
            return False

    @property
    def right(self):
        """Determines the current state of the right IR sensor.

        Return
        ------
        bool
            Indicates if the IR sensor is detecting an object.

        Note
        ----
        Because the read method for the IR sensors uses a time average
        to determine if the sensor is detecting and to reject spurious
        pulses of IR from other sources, the value must be checked
        frequently in a polling loop as shown in the following example.

        Example
        -------

        Create an instance of PiBOT.

        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        If the right IR sensor is detecting, turn on the right LED.

        >>> while True:
        >>>     if robot.ir.right:
        >>>         robot.leds.on(robot.leds.BLUE, side='right')
        >>>     else:
        >>>         robot.leds.off(side='right')

        """

        self._read('right')
        if self._right:
            return True
        else:
            return False

    def _read(self, side):
        """Reads the current state of the left and right IR sensors.

        Parameters
        ----------
        side : {'left', 'right'}
            The side to read.

        Notes
        -----
        The read routine uses a time average strategy to avoid detecting
        short pulses from the remote control or lidar sensor that emit
        at the same frequency of 38 kHz. The detection has a slight
        delay (about 100 ms) to complete the average, but creates a more
        robust detection with no false readings. Readings must be taken
        frequently in a polling loop (10-20 ms cycle time or faster) for
        the averaging strategy to work effectively.

        """

        # set fraction of samples that must be high for detection
        THRESHOLD = 0.75
        # set 100 ms clock for sampling the sensors
        if side == 'left' and self._stop_time_left is None:
            self._stop_time_left = ticks_add(ticks_us(), 100_000)
        elif side == 'right' and self._stop_time_right is None:
            self._stop_time_right = ticks_add(ticks_us(), 100_000)
        # enable the correct IR emitter
        if side == 'left':
            self._IR_left_enable.high()
        elif side == 'right':
            self._IR_right_enable.high()
        # dwell to allow IR emitter time to power on
        sleep_us(250)
        # get the state of the correct IR sensor, increment count, and disable
        if side == 'left':
            self._reading_left += not self._IR_left.value()
            self._count_left += 1
            self._IR_left_enable.low()
        elif side == 'right':
            self._reading_right += not self._IR_right.value()
            self._count_right += 1
            self._IR_right_enable.low()
        # check to see if the testing period for averaging has ended
        if side == 'left' and ticks_diff(ticks_us(), self._stop_time_left) > 0:
            average = self._reading_left / self._count_left
            # reset all values to restart sampling
            self._reading_left = 0
            self._count_left = 0
            self._stop_time_left = None
            # return state of sensor
            if average > THRESHOLD:
                self._left = True
            else:
                self._left = False
        elif side == 'right' and ticks_diff(ticks_us(),
                                            self._stop_time_right) > 0:
            average = self._reading_right / self._count_right
            # reset all values to restart sampling
            self._reading_right = 0
            self._count_right = 0
            self._stop_time_right = None
            # return state of sensor
            if average > THRESHOLD:
                self._right = True
            else:
                self._right = False


class LidarSensor:
    """
    Configures lidar sensor.

    ...

    Notes
    -----
    Review documentation for Pololu Distance Sensor with Pulse Width
    Output, 130cm Max (Pololu item #: 4071):
    https://www.pololu.com/product/4071. The sensor produces a
    continuous pulsed output with the pulse width in microseconds
    corresponding to the distance by the relationship:
    d = (2 mm / 1 microsecond) * (pulse_width - 1000 microseconds).
    A pulse width of 2 ms indicates no objects are in range.

    """

    def __init__(self):
        """Initializes the lidar sensor with appropriate GPIO pin."""

        self._lidar = Pin(cnst.LIDAR, Pin.IN)

    def read(self):
        """Samples the pulse width of lidar sensor to measure distance.

        Return
        ------
        float
            The distance in cm from the center of the robot's wheelbase
            (i.e., center of rotation) to an object. The maximum value
            is determined by the sum of the constants LIDAR_MAX and
            LIDAR_OFFSET (130.5 + 9.5 = 140 cm), which is also the
            returned value if no object is in range.

        Important
        ---------
        Note that the reading is returned as the distance from the
        robot's center of rotation to the object! This is done so that
        the reading corresponds to the radial distance when doing lidar
        scans while rotating the robot.
        
        To avoid crashing into objects, the returned reading must be
        reduced by at least the length from the center of the wheels to
        the front of the robot, which is about 14.5 cm.

        Notes
        -----
        Averaging is possible by setting the constant SAMPLES_LIDAR to
        an integer greater than 1. The typical time required for a lidar
        reading is 10 ms or less. Averaging improves the precision of
        the reading at the cost of added measurement time.

        Example
        -------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Print the lidar reading every 0.1 seconds.

        >>> while True:
        >>>     print(robot.lidar.read())
        >>>     time.sleep(.1)

        """

        # initialize pulse_width measurement variable for the loop
        pulse_width = 0
        pulse_width_temp = 0
        # loop for averaging (set SAMPLES_LIDAR to 1 for no averaging)
        for _ in range(cnst.SAMPLES_LIDAR):
            # wait until a high pulse width of greater than 1 ms is captured
            while pulse_width_temp < 1000:
                # wait in the while loop if already in a high pulse at start
                while self._lidar.value():
                    continue
                # get high pulse width in microseconds with 12 ms timeout
                pulse_width_temp = time_pulse_us(self._lidar, 1, 12_000)
            # sum the pulse width measurements for use in averaging
            pulse_width += pulse_width_temp
        # finish the averaging calulation after summing all samples in the loop
        pulse_width = pulse_width/cnst.SAMPLES_LIDAR
        # calculate the distance in centimeters to a precision of 0.1 cm
        distance = round(2 * (pulse_width-1000) / 10, 1)
        # prevent a negative value
        if distance < 0:
            distance = 0
        # check for values out of range and no detection (pulse width = 2 ms)
        if distance > cnst.LIDAR_MAX:
            distance = cnst.LIDAR_MAX
        # add offset of wheel base (i.e., report value from center of rotation)
        distance += cnst.LIDAR_OFFSET
        return distance


class IrRemote:
    """Configures IR receiver for remote control using a state machine.

    ...

    Notes
    -----
    The TSOP38438 IR receiver sits at the top of the PiBOT. A custom
    state machine receives and decodes the signals from the remote. The
    state machine is fault tolerant--it checks for the correct number
    and length of bursts coming in. If there is a fault caused by
    dropped bits because the remote is out of range, the program
    restarts and clears any partially received data. To ensure the user
    accessible remote commands are updated frequently, an instance of
    this class is passed as an argument when creating an instance of the
    Control class. The control loop runs at an average of 10 ms cycles
    on the second core of the RP2040. 
    
    Whenever a remote control button is pressed and released, the
    robot.remote.command property will return the new code for a short
    time until it is automatically cleared, at which point the property
    returns None. If the button is held down, the code value is retained
    and the robot.remote.repeat property will return True until the
    button is released and the code is returned to None. This timing can
    be tricky when working with remote control commands in a program.
    The examples provided below demonstate how to avoid timing problems
    using the three properties: command, stale, and repeat.

    """

    def __init__(self):
        """Creates state machine for IR receiver decoding."""

        # create instance of StateMachine on PIO 1 (i.e., sm_id=4)
        # in_base sets where the JMP instruction tests for high conditions
        base_pin = Pin(cnst.IR_REMOTE, Pin.IN)
        self._ir_remote = StateMachine(4, self._sm_ir_remote, freq=21_335,
                                       in_base=base_pin, jmp_pin=base_pin)
        # activate the state machine
        self._ir_remote.active(1)
        # create attributes to store new commands and detect repeat codes
        self._command = None
        self._stale = True
        self._repeat = False
        # create start time for testing timing of repeat codes
        self._start_time = None

    @property
    def command(self):
        """Gets the current remote command.

        Return
        ------
        int, None
            The current decoded command from the IR receiver. None
            indicates that no new code or repeat code has been received.

        Example
        -------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Print the current remote command at 0.1 second intervals.

        >>> print('Press a button on the remote control')
        >>> while True:
        >>>     print(robot.remote.command)
        >>>     time.sleep(.1)

        """

        command = self._command
        if command is not None:
            if not self._stale:
                self._stale = True
            return command
        else:
            return None

    @property
    def stale(self):
        """Determines if the remote command has been read and is stale.

        Return
        ------
        bool
            True if the remote command is stale (i.e., button press
            recorded and command read but not yet cleared from memory).

        Note
        ----
        The stale property is set False when a new command is received
        and decoded by the state machine, and before it is read by the
        user with the command property (i.e., robot.remote.command).
        Checking the command property, even just once in a conditional
        test, sets the stale property to True. Therefore, the stale
        property must always be checked BEFORE the command property or
        it will not provide any useful information.

        Example
        -------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Testing for stale commands allows the command to be read once
        and only once for each instance of a button press.

        >>> print('Press a button on the remote control')
        >>> while True:
        >>>     if not robot.remote.stale:
        >>>         command = robot.remote.command
        >>>         if command is not None:
        >>>             print(command)
        >>>     time.sleep(.1)

        """

        return self._stale

    @property
    def repeat(self):
        """Determines if the remote command is being repeated.

        Return
        ------
        bool
            True if the remote command is actively being repeated (i.e.,
            the button is being held down).

        Example
        -------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Print the command at 0.1 second intervals only when a button is
        held down.

        >>> print('Press and hold a button on the remote control')
        >>> while True:
        >>>     if robot.remote.repeat:
        >>>         command = robot.remote.command
        >>>         print(f'{command} button is held down.')
        >>>     time.sleep(.1)

        """

        return self._repeat

    @asm_pio(in_shiftdir=PIO.SHIFT_RIGHT)
    def _sm_ir_remote():
        """Special assembly instructions for the state machine

        Notes
        -----
        This state machine uses 30 of the available 32 instructions.
        There may be ways to make it more compact and efficient, but it
        serves the purpose well in its current form. The program watches
        for the start burst and start pause of a new code or repeat
        command. If the longer 4.5 ms pause is detected, the 32-bit
        signal is decoded based on decrement counters using the X and Y
        scratch registers to keep track of the pause duration (i.e.,
        high time) and the bit count. If the shorter 2.5 ms pause is
        detected, the program pushes a null code to the RX FIFO to
        indicate a repeat command was detected. If a start condition is
        not met, the program starts over. If an expected burst does not
        arrive after either a short pause (coding for a 0) or a long
        pause (coding for a 1), a fault condition is met and the program
        restarts and clears the ISR to get rid of any partial codes
        received. Only when all 32 bits are received as expected does
        the program push the full 32-bit ISR to the RX FIFO. The
        POSITION 0-4 markers in the comments of the assembly
        instructions correspond to features of the encoded IR signal.
        A separate annotated diagram is part of the PiBOT documentation.

        """

        label("restart")
        mov(isr, null) # reset ISR to zero to clear any partial codes received
        # wait for start burst and measure 3 ms before continuing
        set(x, 31) # 64 cycles, two jmp instructions (46.875*64 = 3000 us)
        wait(0, pin, 0) # wait for falling edge (POSITION 0)
        label("start_burst")
        jmp(pin, "restart") # if high, restart
        jmp(x_dec, "start_burst") # if X not zero keep timing start burst
        # wait for start pause and measure 3 ms before continuing
        set(x, 31) # 63 cycles, two jmp instructions (46.875*63 = 2953.125 us)
        wait(1, pin, 0) # wait for rising edge (POSITION 1)
        label("start_pause")
        jmp(x_dec, "start_pause_test") # if X not zero keep timing start pause
        # wait for code burst, measure pause, and decode
        set(y, 31) # 32 bit code counter
        label("decode_start")
        set(x, 8) # 17 cycles, two jmp instructions (46.875*17 = 796.875 us)
        wait(0, pin, 0) # wait for falling edge--start of code pulse
        wait(1, pin, 0) # wait for rising edge--start of code pause(POSITION 2)
        label("continue_decode_1")
        jmp(x_dec, "test_input_1") # if X not zero keep timing code pause
        in_(pins, 1) # if counter ends (input still high), a 1 should be stored
        # set counter for fault test
        set(x, 11) # ((17 + 2 + 23)*46.875 = 1,968.75 us total)
        label("continue_decode_2")
        jmp(x_dec, "test_input_2") # if X not zero keep timing code pause
        jmp("restart") # if expected burst (low input) doesn't arrive, restart
        # test for high or low at beginning code pause
        label("test_input_1")
        jmp(pin, "continue_decode_1") # if high, keep counting
        in_(pins, 1) # if low, a new burst arrived and a 0 should be stored
        label("test_decode_start")
        jmp(y_dec, "decode_start") # start decode over for next bit
        # continue if all 32 bits received
        push(noblock) # push the full 32-bit ISR to the RX FIFO
        wait(1, pin, 0) # wait for final burst to end, then restart(POSITION 3)
        jmp("restart") # restart after a successful decoding
        # test for high or low to ensure expected burst arrives for next bit
        label("test_input_2")
        jmp(pin, "continue_decode_2") # if high, keep counting
        # if low, the expected burst arrived and there is no fault
        jmp("test_decode_start") # return to decrement bit counter
        # test for high or low on start pause to ensure start condition is met
        label("start_pause_test")
        jmp(pin, "start_pause") # if high, keep counting
        # if low, repeat command detected
        mov(isr, null) # move null into ISR to indicate a repeat command
        push(noblock) # push the null command to the RX FIFO
        wait(1, pin, 0) # wait for idle line (high) before restart (POSITION 4)
        jmp("restart") # restart after a repeat command detected

    def _update_command(self):
        """Gets the remote commands decoded by the state machine.

        Notes
        -----
        The for loop does two important things. First, it makes sure
        there is a value ready to be retrieved using the .rx_fifo()
        method. If you use the .get() method when the RX FIFO is empty,
        the program is blocked while it waits for a value to return.
        Second, the for loop clears out any old codes waiting in the
        RX FIFO, which can hold up to four 32-bit words. If multiple
        codes were received before a user checks for them, then a single
        .get() call would only retrieve the oldest, stalest code. The
        for loop tosses out the oldest codes and gets only the most
        recent one. The raw code is also checked to determine if a
        repeat command was received (null from state machine). Finally,
        the elapsed_time variable is used to detemine if the repeat
        attribute needs to be cleared by testing the time between start
        pulses.

        """

        # purge out all old stale values from the RX FIFO
        for _ in range(self._ir_remote.rx_fifo()):
            # get the raw 32-bit remote code
            raw_code = self._ir_remote.get()
            self._start_time = ticks_us()
            # check for null code indicating repeat
            if raw_code == 0:
                self._repeat = True
            else:
                # save only 8-bit code in the command attribute
                self._command = (raw_code >> 16) & 0x00ff
                self._stale = False
                self._repeat = False
            return
        # measure the time elapsed since the last code was received
        elapsed_time = ticks_diff(ticks_us(), self._start_time)
        # check for conditions that end the repeat and clear the command
        if ((self._repeat and elapsed_time > 120_000) or
            (not self._repeat and elapsed_time > 80_000)):
            self._repeat = False
            self._command = None
            self._stale = False