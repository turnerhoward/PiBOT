"""PiBOT motion, control, output, and sensor constants"""

# MOTION AND PATH CONSTANTS
WHEEL_DIA = 6.5      # wheel diameter (cm)
WHEEL_SPAN = 11.5    # wheel span (cm)
SPD_MAX = 30         # maximum linear speed (cm/s)
SPD_MIN = 5          # minimum linear speed (cm/s)
ACC_MAX = 50         # maximum linear acceleration (cm/s^2)
ANG_MAX = 720        # maximum angle for discrete rotation (deg)
ANG_SPD_MAX = 180    # maximum angular speed for rotation (deg/sec)
ANG_SPD_MIN = 30     # minimum angular speed for rotation (deg/sec)
ANG_ACC = 540        # angular acceleration for rotation (deg/s^2)
MIN_RADIUS = 0.5     # minimum radius for arc motion (cm)
# MOTOR AND ENCODER CONSTANTS
FORWARD_LEFT = 8     # GPIO for forward direction of left motor
REVERSE_LEFT = 9     # GPIO for reverse direction of left motor
FORWARD_RIGHT = 10   # GPIO for forward direction of right motor
REVERSE_RIGHT = 11   # GPIO for reverse direction of right motor
ENCODER_LEFT_A = 4   # GPIO for left encoder channel A
ENCODER_LEFT_B = 5   # GPIO for left encoder channel B
ENCODER_RIGHT_A = 2  # GPIO for right encoder channel A
ENCODER_RIGHT_B = 3  # GPIO for right encoder channel B
PWM_FREQ = 10_000    # PWM frequency in Hz (max for MAKER-PI-RP2040 is 20 kHz)
MAX_ON_TIME = 75_000 # maximum on_time (pulse width) in ns to limit motor speed
ENC_RESOLUTION = 975 # encoder counts per wheel revolution
                     # (1:48.75 gear ratio * 5 pulses per rev. * 4 edges = 975)
# PID CONTROL CONSTANTS
K_P = 450_000        # proportional gain for PID controller # 600_000
K_I = 1_000_000      # integral gain for PID controller     # 1_500_000
K_D = 1000           # derivative gain for PID controller   # 1500
TAU_PID = .01        # filter time constant for derivative term
T_STEP = 0.01        # time step for control loop (s)
# CONTROL LOOP CONSTANTS
MEM_THRESH = 65_536  # 64 KB (25% of available memory used triggers collection)
# OUTPUTS CONSTANTS
BUZZER_PIN = 22      # GPIO pin connected to buzzer on MAKER-PI-RP2040 board
MIN_LEVEL = 0x0080   # 16-bit PWM duty cycle for lowest sound level
NEOPIXEL = 18        # GPIO pin connected to neopixels on MAKER-PI-RP2040 board
# SENSORS CONSTANTS
IR_LEFT = 7          # GPIO for left IR sensor
IR_LEFT_ENABLE = 28  # GPIO enable for left IR sensor
IR_RIGHT = 0         # GPIO for right IR sensor
IR_RIGHT_ENABLE = 1  # GPIO enable for right IR sensor
LIDAR = 16           # GPIO for LIDAR input
SAMPLES_LIDAR = 1    # number of samples to average for LIDAR reading
LIDAR_MAX = 130.5    # maximum value recorded from LIDAR read
LIDAR_OFFSET = 9.5   # offset in cm from sensor to robot center of rotation
BEAM_ANGLE = 5       # approximate effective beam angle of lidar
SLOPE_THRESHOLD = 1  # minimum threshold in lidar scan derivative for a peak
WHISKER_LEFT = 6     # GPIO for left whisker
WHISKER_RIGHT = 26   # GPIO for right whisker
IR_REMOTE = 27       # GPIO for IR receiver module