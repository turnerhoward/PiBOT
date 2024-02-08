import constants as cnst
import neopixel
from utime import sleep_ms
from machine import Pin, PWM


class Buzzer:
    """Generates sound from built-in buzzer on MAKER-PI-RP2040 board."""

    def __init__(self):
        """Creates buzzer control attributes."""

        self._pwm = PWM(Pin(cnst.BUZZER_PIN, Pin.OUT))
        self._pulse_set = False
        sleep_ms(25)
        self._start = None

    def on(self, freq=1000, level=3):
        """Turns on the buzzer at desired frequency and volume.

        Parameters
        ----------
        freq : int, default=1000
            Frequency of the tone generated. Minimum is 8 Hz.
        level : int, default=3
            Volume of the buzzer with levels 1-5, 5 being the loudest.

        Notes
        -----
        The audible volume of the buzzer is highly dependent on the
        frequency and the pulse width. Sometimes increasing the level
        (i.e., pulse width) does not generate a louder sound but can
        affect the timbre.

        Examples
        --------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Turn the buzzer on for three seconds and then turn it off.

        >>> robot.buzzer.on(freq=440, level=1)
        >>> time.sleep(3)
        >>> robot.buzzer.off()

        """

        # check for valid arguments
        try:
            valid = self._valid_arguments(freq=freq, level=level)
        except:
            print('Error: invalid argument to buzzer.on() method')
            return
        else:
            if not valid:
                return
        # set user defined frequency in PWM instance
        self._pwm.freq(freq)
        # calculate a 16-bit PWM duty cycle from user defined level
        self._level = cnst.MIN_LEVEL << level
        # turn on buzzer by setting PWM duty cycle with 16-bit level value
        self._pwm.duty_u16(self._level)

    def off(self):
        """Turns off the buzzer."""

        # set PWM duty cycle to zero to silence the buzzer
        self._pwm.duty_u16(0)

    def pulse(self, on_time=100, bursts=1, freq=1000, level=3):
        """Generates single, multiple, or continuous buzzer pulses.

        Parameters
        ----------
        on_time : int, default=100
            Time in ms to turn on the buzzer. Value must be an integer
            multiple of 10 ms to match the control loop step time and no
            larger than 5000 ms.
        bursts : int, default=1
            Number of pulses to sound. Duty cycle is 50% (i.e., off time
            is equal to on time. Use -1 to generate continuous pulses.
        freq : int, default=1000
            Frequency of the tone generated. Minimum is 8 Hz.
        level : int, default=3
            Volume of the buzzer with levels 1-5, 5 being the loudest.

        Notes
        -----
        This pulse function works in conjunction with the Control class
        to turn the buzzer on and off in the robot control loop that
        runs on the second core of the RP2040.

        Examples
        --------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from pibot import PiBOT
        >>> robot = PiBOT()

        Create continuous pulses and reset after five seconds to cancel.

        >>> robot.buzzer.pulse(on_time=250, bursts=-1, freq=440, level=1)
        >>> time.sleep(5)
        >>> robot.buzzer.reset()

        A discrete number of bursts does not require a reset to cancel.

        >>> robot.buzzer.pulse(bursts=3)

        """

        # check for valid arguments
        try:
            valid = self._valid_arguments(on_time=on_time, bursts=bursts,
                                          freq=freq, level=level)
        except:
            print('Error: invalid argument to buzzer.pulse() method')
            return
        else:
            if not valid:
                return
        # set user defined frequency in PWM instance
        self._pwm.freq(freq)
        # calculate a 16-bit PWM duty cycle from user defined level
        self._level = cnst.MIN_LEVEL << level
        # calculate the number of control loop steps to run the buzzer pulse
        self._on_steps = on_time / 10
        self._bursts = bursts
        self._pulse_set = True

    def reset(self):
        """Clears buzzer pulses and reinitializes attributes."""

        self._pwm.duty_u16(0)
        # set _bursts to 0 to reset _pulse_set in the control loop
        self._bursts = 0

    @staticmethod
    def _valid_arguments(freq, level, on_time=None, bursts=None):
        """Checks for valid arguments from user calls.

        Accepts all the parameters to .on() and .pulse() to check if
        they are valid.

        """

        if type(freq) is not int or freq < 8:
            print('freq must be an integer 8 Hz or greater')
        elif type(level) is not int or level < 1 or level > 5:
            print('level must be an integer from 1 to 5')
        elif on_time is not None and (type(on_time) is not int
                                      or on_time % 10 != 0 or on_time < 10
                                      or on_time > 5000):
            print('on_time must be an integer multiple of 10 ms'
                             + ' and not more than 5000 ms')
        elif (bursts is not None
              and (type(bursts) is not int or bursts < -1 or bursts == 0)):
            print('bursts must be an integer >= 1 or = -1 for continuous')
        else:
            # if no invalid arguments are found, return True
            return True
        # if an invalid argument is found, return False
        return False


class LEDs:
    """
    Controls the RGB neopixel LEDs on the MAKER-PI-RP2040 board.

    Color constants are provided to allow for easier setting of common
    colors. The constants are 3-tuples of RGB values from 0 to 255.

    """

    # LED color table
    AMBER = (120, 50, 0)
    AQUA = (20, 70, 70)
    RED = (128, 0, 0)
    GOLD = (64, 55, 10)
    JADE = (0, 120, 25)
    YELLOW = (110, 80, 0)
    GREEN = (0, 128, 0)
    CYAN = (0, 64, 64)
    BLUE = (0, 0, 128)
    PURPLE = (80, 0, 64)
    MAGENTA = (128, 0, 20)
    WHITE = (64, 64, 64)
    ORANGE = (120, 20, 0)
    PINK = (64, 25, 25)
    TEAL = (0, 108, 54)

    def __init__(self):
        """Creates neopixel LED control attributes."""

        # create instance of NeoPixel class at the appropriate pin
        self._np = neopixel.NeoPixel(Pin(cnst.NEOPIXEL), 2)
        # create state variables to indicate LED and pulse status
        self._lit_left = False
        self._lit_right = False
        self._pulse_set = None
        self._pulse_left_set = None
        self._pulse_right_set = None
        sleep_ms(25)
        self._start = None
        self._start_left = None
        self._start_right = None
        self._bursts = 1
        self._bursts_left = 1
        self._bursts_right = 1
        self.off()

    def on(self, color, side=None):
        """Turns on left, right, or both LEDs to specified color.

        Parameters
        ----------
        color : 3-tuple
            Color (and brightness) set with RGB values from 0 to 255.
        side : {'left', 'right'}, default=None
            The side to illuminate. Default is None for both sides.

        Notes
        -----
        Pass one of the colors in the table or use a custom 3-tuple to
        select your own color and brightness.

        Examples
        --------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from robot import PiBOT
        >>> robot = PiBOT()

        Turn on both LEDs with the color red for three seconds.

        >>> robot.leds.on(robot.leds.RED)
        >>> time.sleep(3)
        >>> robot.leds.off()

        Use the optional side parameter to turn on only one LED.

        >>> robot.leds.on(robot.leds.AMBER, side='left')
        >>> robot.leds.on(robot.leds.TEAL, side='right')
        >>> time.sleep(3)
        >>> robot.leds.off('left')
        >>> time.sleep(3)
        >>> robot.leds.off('right')

        """

        # check for valid arguments
        try:
            valid = self._valid_arguments(color=color, side=side)
        except:
            print('Error: invalid argument to leds.on() method')
            return
        else:
            if not valid:
                return
        # default is to set both sides
        if side is None:
            self._np.fill(color)
            self._lit_left = True
            self._lit_right = True
            self._color = color
        # set only the right LED
        elif side == 'right':
            self._np[0] = color
            self._lit_right = True
            self._color_right = color
        # set only the left LED
        elif side == 'left':
            self._np[1] = color
            self._lit_left = True
            self._color_left = color
        # write to the neopixels only once before returning
        self._np.write()

    def off(self, side=None):
        """Turns off left, right, or both LEDs.

        Parameters
        ----------
        side : {'left', 'right'}, default=None
            The side to turn off. Default is None for both sides.

        """

        # check for valid arguments
        try:
            valid = self._valid_arguments(side=side)
        except:
            print('Error: invalid argument to leds.off() method')
            return
        else:
            if not valid:
                return
        # default is to turn off both sides
        if side is None:
            self._np.fill((0, 0, 0))
            self._lit_left = False
            self._lit_right = False
        # turn off only the right LED
        elif side == 'right':
            self._np[0] = (0, 0, 0)
            self._lit_right = False
        # turn off only the left LED
        elif side == 'left':
            self._np[1] = (0, 0, 0)
            self._lit_left = False
        # write to the neopixels only once before returning
        self._np.write()

    def pulse(self, color, on_time=100, bursts=1, side=None, fade=None):
        """Generates single, multiple, or continuous pulses on LEDs.

        Parameters
        ----------
        color : 3-tuple
            Color (and brightness) set with RGB values from 0 to 255.
        on_time : int, default=100
            Time in ms to turn on the buzzer. Value must be an integer
            multiple of 10 ms to match the control loop step time and no
            larger than 5000 ms.
        bursts : int, default=1
            Number of pulses on LED. Duty cycle is 50% (i.e., off time
            is equal to on time. Use -1 to generate continuous pulses.
        side : {'left', 'right'}, default=None
            The side to pulse. Default is None for both sides.
        fade : {True}, default=None
            Ramps the LED brightness up and down to create fade effect.
            Default is None for instantaneous illumination.

        Notes
        -----
        Pass one of the colors in the table or use a custom 3-tuple to
        select your own color and brightness.

        Examples
        --------

        Import time and create an instance of PiBOT.

        >>> import time
        >>> from robot import PiBOT
        >>> robot = PiBOT()

        Create continuous pulses and reset after five seconds to cancel.

        >>> robot.leds.pulse(robot.leds.RED, on_time=250, bursts=-1)
        >>> time.sleep(5)
        >>> robot.leds.reset()

        Creates three fading pulses with no need to reset.

        >>> robot.leds.pulse(robot.leds.AMBER, on_time=500, side='left', bursts=3, fade=True)
        >>> robot.leds.pulse(robot.leds.TEAL, on_time=500, side='right', bursts=3, fade=True)

        Pulses can also be called with the LEDs on to create dual color.

        >>> robot.leds.on(robot.leds.BLUE)
        >>> time.sleep(3)
        >>> robot.leds.pulse(robot.leds.RED, on_time=500, bursts=-1)
        >>> time.sleep(5)
        >>> robot.leds.reset()

        """

        # check for valid arguments
        try:
            valid = self._valid_arguments(color=color, on_time=on_time,
                                          bursts=bursts, side=side, fade=fade)
        except:
            print('Error: invalid argument to leds.pulse() method')
            return
        else:
            if not valid:
                return
        # default is to pulse both sides
        if side is None:
            self._pulse_set = (color, on_time/10, fade)
            self._bursts = bursts
        # pulse only the right LED
        elif side == 'right':
            self._pulse_right_set = (color, on_time/10, fade)
            self._bursts_right = bursts
        # pulse only the left LED
        elif side == 'left':
            self._pulse_left_set = (color, on_time/10, fade)
            self._bursts_left = bursts

    def reset(self):
        """Clears LED pulses and reinitializes attributes."""

        self.__init__()

    @staticmethod
    def _valid_arguments(color=None, side=None, on_time=None, bursts=None,
                         fade=None):
        """Checks for valid arguments from user calls.

        Accepts all the parameters to leds.on(), leds.off, and
        leds.pulse() to check if they are valid.

        """
         
        if color and not (type(color) is tuple and len(color) == 3
                and all(isinstance(n, int) and 0 <= n < 256 for n in color)):
            print('color must be a 3-tuple with RGB integers from 0 - 255')
        elif side is not None and side not in ('left', 'right'):
            print("side must be 'left' or 'right'")
        elif on_time is not None and (type(on_time) is not int
                                      or on_time % 10 != 0 or on_time < 10
                                      or on_time > 5000):
            print('on_time must be an integer multiple of 10 ms'
                             + ' and not more than 5000 ms')
        elif (bursts is not None
              and (type(bursts) is not int or bursts < -1 or bursts == 0)):
            print('bursts must be an integer >= 1 or = -1 for continuous')
        elif fade is not None and fade is not True:
            print('fade is optional and can only be set to True')
        else:
            # if no invalid arguments are found, return True
            return True
        # if an invalid argument is found, return False
        return False