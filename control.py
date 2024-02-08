import gc
import _thread
import constants as cnst
from math import pi, sin, cos
from utime import ticks_ms, ticks_diff, ticks_add
from machine import Pin, PWM
from rp2 import StateMachine, asm_pio, PIO


class Control:
    """
    Maintains control of the robot in a fast loop on the second core.

    ...

    Notes
    -----
    This class contains no user attributes or methods.

    """

    def __init__(self, leds=None, buzzer=None, remote=None):
        """Creates motion control attributes."""

        # PASS IN OBJECTS TO HANDLE LEDS, BUZZER, AND REMOTE
        self._leds = leds
        self._buzzer = buzzer
        self._remote = remote
        # OBJECTS TO HANDLE MOTOR CONTROL
        # create instance of MotorPID to calculate control input
        self._pid = MotorPID()
        # create instance of MotorControl to command motors and read encoders
        self._motors = MotorControl()
        # ATTRIBUTES FOR CONTROL LOOP
        # starting time
        self._t_start = 0
        # counter for control loop steps
        self._steps = 0
        # current clock time in seconds for control loop
        self._t_current = 0
        # ATTRIBUTES FOR ROBOT WHEEL MOTION
        # commanded angles
        self._theta_L = 0
        self._theta_R = 0
        # current angles from encoders
        self._theta_curr_L = 0
        self._theta_curr_R = 0
        # previous angles from encoders
        self._theta_prev_L = 0
        self._theta_prev_R = 0
        # ATTRIBUTES FOR ROBOT MOTION AND PATH PLANNING
        # state variables for active motion
        self._protect = False
        # possible states: 'stop', 'pause', 'linear', 'rotate', 'arc', 'steer'
        self._motion_state = 'stop' 
        # variables to store user requested setpoints
        self._motion_queue = []
        self._motion_curr = ['ready']
        self._motion_prev = ['ready']
        self._motion_resume = ['ready']
        # active setpoint values
        self._velo_set = 0
        self._dist_set = 0
        self._ang_velo_set = 0
        self._ang_set = 0
        self._arc_len_set = 0
        # current values to compare to setpoint
        self._velo_curr = 0
        self._dist_curr = 0
        self._ang_velo_curr = 0
        self._ang_curr = 0
        self._arc_len_curr = 0
        # wheel differential values for arc and steer
        self._diff_L = 0
        self._diff_R = 0
        # ATTRIBUTES FOR TRACKING POSITION OF ROBOT ON FIELD
        # stores current x, y coordinate position of the center of wheelbase
        self._position = [0, 0]
        # stores current heading angle of robot
        self._heading = 0
        # lock to avoid setting heading or position during _tracking() call
        self._tracking_lock = False
        # STARTS THREAD ON SECOND CORE FOR MOTION CONTROL
        # set threshold for memory allocation to force a garbage collection
        gc.threshold(cnst.MEM_THRESH)
        # create flag used to exit control loop thread on second core
        self._thread_running = True
        _thread.start_new_thread(self._motion_thread, ())

    def _pause(self):
        """Smoothly pauses all motion by ramping down velocity."""

        # return if already stopped and the queue is empty
        if self._motion_state in ('stop', 'pause') and not self._motion_queue:
            return
        # set the current commanded motion to pause
        self._motion_curr = ['pause']

    def _resume(self):
        """Smoothly resumes motion after a pause."""

        # if already moving or ready for new command, return without resuming
        if self._motion_state != 'pause': # or self._motion_curr[0] == 'ready':
            return
        # check for queued sequences and pop one out
        elif self._motion_queue and self._motion_state != 'pause':
            self._motion_curr = self._motion_queue.pop(0)
        # check for stopped linear move
        elif self._motion_prev[0] == 'linear':
            # if discrete linear move
            if self._dist_set != 0 :
                # if above threshold, calculate remaining distance
                if abs(self._dist_set - self._dist_curr) > 0.1:
                    self._motion_curr = self._motion_prev.copy()
                    self._motion_curr[3] = self._dist_set - self._dist_curr
                # if below threshold and there is queued motion, pop one out
                elif self._motion_queue:
                    self._motion_curr = self._motion_queue.pop(0)
                # otherwise, there is nothing left to resume
                else:
                    self._motion_curr = ['ready']
            # if the motion was continuous, restore from previous
            else:
                self._motion_curr = self._motion_prev.copy()
        # check for stopped rotation
        elif self._motion_prev[0] == 'rotate':
            # if discrete rotation
            if self._ang_set != 0 :
                # if above threshold, calculate remaining angle
                if abs(self._ang_set - self._ang_curr) > 0.0175:
                    self._motion_curr = self._motion_prev.copy()
                    self._motion_curr[3] = self._ang_set - self._ang_curr
                # if below threshold and there is queued motion, pop one out
                elif self._motion_queue:
                    self._motion_curr = self._motion_queue.pop(0)
                # otherwise, there is nothing left to resume
                else:
                    self._motion_curr = ['ready']
            # if the motion was continuous, restore from previous
            else:
                self._motion_curr = self._motion_prev.copy()
        # check for stopped arc move
        elif self._motion_prev[0] == 'arc':
            # if discrete arc move
            if self._arc_len_set != 0:
                # if above threshold, calculate remaining arc length
                if abs(self._arc_len_set - self._arc_len_curr) > 0.1:
                    self._motion_curr = self._motion_prev.copy()
                    self._motion_curr[4] = (self._arc_len_set
                                            - self._arc_len_curr)
                # if below threshold and there is queued motion, pop one out
                elif self._motion_queue:
                    self._motion_curr = self._motion_queue.pop(0)
                # otherwise, there is nothing left to resume
                else:
                    self._motion_curr = ['ready']
            # if the motion was continuous, restore from previous
            else:
                self._motion_curr = self._motion_prev.copy()
        # check for stopped steer move
        elif self._motion_prev[0] == 'steer':
            # if discrete steer move
            if self._arc_len_set != 0:
                # if above threshold, calculate remaining arc length
                if abs(self._arc_len_set - self._arc_len_curr) > 0.1:
                    self._motion_curr = self._motion_prev.copy()
                    self._motion_curr[4] = (self._arc_len_set
                                                     - self._arc_len_curr)
                # if below threshold and there is queued motion, pop one out
                elif self._motion_queue:
                    self._motion_curr = self._motion_queue.pop(0)
                # otherwise, there is nothing left to resume
                else:
                    self._motion_curr = ['ready']
            # if the motion was continuous, restore from previous
            else:
                self._motion_curr = self._motion_prev.copy()

    def _motion_thread(self):
        """Starts a thread to run control loop on second core."""

        # _t_start used to keep track of the clock and maintain uniform steps
        self._t_start = ticks_ms()
        self._control_loop()

    def _control_loop(self):
        """Runs an infinite 10 ms polling loop to check for events.

        Notes
        -----
        This control loop is critical to maintain timing at 10 ms. In
        previous versions of this method, a garbage collection was
        performed in every loop to clean up memory so it wouldn't become
        exhausted after a few hundred cycles and cause an observable
        "hiccup" in motion every few seconds when the memory runs out
        and an automatic garbage collection occurs. When performed every
        loop, the garbage collection takes 4-5 ms, which generally left
        enough time to complete all other tasks in the loop without
        spilling over. If left to happen automatically, the garbage
        collection takes well over 10 ms. With the addition of the
        buzzer and led pulse control, the loop time was already
        stretched to the limit and on the edge of spilling over and
        creating inconsistent timing. The addition of lidar scanning
        routines that collect data in large lists finally broke the old
        method of running garbage collection every cycle because of
        occasional long (10-20 ms) garbage collections.
        
        The current method uses automatic garbage collection, but sets
        a memory threshold (see the constant MEM_THRESH) to force
        garage collection when a certain portion of memory is used. In
        practice, the collections happen every 10-20 loop cycles and
        usually spill over the 10 ms loop time by a small amount. After
        a long garbage collection loop, the following loop (or several
        loops) run at maximum possible speed without the timing delay to
        catch up to the proper time again. Interestingly, these
        unequally spaced time steps do not appear to cause motion
        "hiccups" that were observed in earlier iterations that used
        automatic garbage collection. When these irregular steps happen,
        the timing of other events like buzzer and LED pulses will also
        be slightly altered but not noticeble to a user. Overall,
        loosening the strict requirement of 10 ms steps in every loop
        allows for greater functionality in the control loop without a
        noticeable reduction in performance as observed by the user.

        """

        while self._thread_running:
            # check for queued sequences and pop out the first one in the list
            if (self._motion_queue and self._motion_state != 'pause'
                    and (not self._protect
                         or (self._protect and self._motion_state == 'stop'))):
                self._motion_curr = self._motion_queue.pop(0)
                self._update_state()
            # update state when _motion_curr is not in standby (i.e., 'ready')
            if self._motion_curr[0] != 'ready':
                self._update_state()
            # check for rotate command
            if self._motion_state == 'rotate':
                self._rotate_control()
            # check for linear command (i.e, forward or reverse)
            elif self._motion_state == 'linear':
                self._forward_control()
            # check for arc command
            elif self._motion_state == 'arc':
                self._arc_control()
            # check for steer command
            elif self._motion_state == 'steer':
                self._steer_control()
            # take control action in each loop
            self._control_action()
            # check for required buzzer and led pulses
            if self._buzzer._pulse_set:
                self._buzzer_control()
            if self._leds._pulse_set:
                self._leds_control()
            if self._leds._pulse_right_set:
                self._leds_control_right()
            if self._leds._pulse_left_set:
                self._leds_control_left()
            if (self._leds._pulse_set or self._leds._pulse_right_set
                    or self._leds._pulse_left_set):
                self._leds._np.write()
            # check for remote commands
            self._remote._update_command()
            # increment loop counter
            self._steps += 1
            # pause until next time step is reached to create uniform steps
            while ticks_diff(ticks_ms(),
                             ticks_add(self._t_start,
                                       round(cnst.T_STEP*
                                             self._steps*1000))) < 0:
                continue
            # update current time (s)
            self._t_current = ticks_diff(ticks_ms(), self._t_start) / 10**3

    def _update_state(self):
        """Updates state to determine which motion control to call.

        Notes
        -----
        Motion commands called by a user are stored in the _motion_curr
        attribute. When that attribute changes from the standby setting
        of 'ready', this method is called to update the state. The
        commanded motion (i.e., state) is updated at a predictable
        position in the control loop to avoid race conditions that might
        occur if the user commanded values from the main core were
        immediately updated in the motion thread at random times.

        """

        # create local copy to avoid changing state during update
        motion_curr = self._motion_curr.copy()
        # reset the wheel angles just before starting new motion
        if motion_curr[0] in ('linear', 'rotate', 'arc'):
            self._theta_L, self._theta_R = self._motors._read_angles()
            # reset previous wheel angles for tracking calculation
            self._theta_prev_L = self._theta_L
            self._theta_prev_R = self._theta_R
        # transition robot to stop state by setting velocity to zero
        if motion_curr[0] == 'pause':
            if self._velo_set != 0 or self._ang_velo_set != 0:
                self._velo_set = 0
                self._ang_velo_set = 0
            return
        # transition robot to linear motion
        elif motion_curr[0] == 'linear':
            self._protect = motion_curr[1]
            self._velo_set = motion_curr[2]
            self._dist_set = motion_curr[3]
            self._dist_curr = 0
            self._motion_state = 'linear'
        # transition robot to rotate motion
        elif motion_curr[0] == 'rotate':
            self._protect = motion_curr[1]
            self._ang_velo_set = motion_curr[2]
            self._ang_set = motion_curr[3]
            self._ang_curr = 0
            self._motion_state = 'rotate'
        # transition robot to arc motion
        elif motion_curr[0] == 'arc':
            self._protect = motion_curr[1]
            self._velo_set = motion_curr[2]
            self._arc_len_set = motion_curr[4]
            self._wheel_diff(motion_curr[3], motion_curr[5])
            self._arc_len_curr = 0
            self._motion_state = 'arc'
        # transition robot to steer motion
        elif motion_curr[0] == 'steer':
            # store current progress of discrete linear or arc motion
            if self._motion_prev[0] != 'steer':
                if self._motion_resume[0] =='linear':
                    self._motion_resume[3] = self._dist_set - self._dist_curr
                    self._dist_curr = 0
                elif self._motion_resume[0] == 'arc':
                    self._motion_resume[4] = (self._arc_len_set
                                              - self._arc_len_curr)
                    self._arc_len_curr = 0
            # update motion parameters based on user command
            self._velo_set = motion_curr[2]
            self._arc_len_set = motion_curr[4]
            self._wheel_diff(motion_curr[3], motion_curr[5])
            self._arc_len_curr = 0
            self._motion_state = 'steer'
        # reset _motion_prev before returning to control loop
        if self._motion_state != 'stop':
            self._motion_prev = motion_curr.copy()
            self._motion_curr = ['ready']

    def _forward_control(self):
        """Calculates kinematics and wheel rotation for linear motion.

        Notes
        -----
        At each time step, the velocity, distance, and wheel angles are
        calculated to command a forward (or reverse) linear path. The
        velocity ramps up at the start and down at the end of the path
        to approximate constant acceleration.

        """

        # check for end of discrete move
        if self._velo_set == 0 and self._velo_curr == 0:
            # if stopped short using pause method, return
            if self._motion_curr[0] == 'pause':
                self._motion_curr = ['ready'] # swapped to avoid race condition
                if not self._motion_queue:
                    self._protect = False
                self._motion_state = 'pause' # swapped to avoid race condition
                return
            # if actual end of move, check for remaining error and adjust
            elif abs(self._dist_set - self._dist_curr) > 0.1:
                self._theta_L += (self._dist_set
                                  - self._dist_curr) / (cnst.WHEEL_DIA/2)
                self._theta_R += (self._dist_set
                                  - self._dist_curr) / (cnst.WHEEL_DIA/2)
                self._dist_curr = self._dist_set
                return
            # at end, set motion_state to 'stop' to end move
            else:
                self._motion_state = 'stop'
                if not self._motion_queue:
                    self._protect = False
                return
        # ramp the velocity up or down or hold steady at setpoint
        self._velocity_ramp()
        # calculate current distance for discrete move
        if self._dist_set != 0:
            self._dist_curr += self._velo_curr * cnst.T_STEP
            # calculate remaining and rampdown distances
            dist_remain = self._dist_set - self._dist_curr
            dist_ramp = self._velo_curr**2 / (2*cnst.ACC_MAX)
            # check for ramp down at end of move
            if ((self._velo_set > 0 and dist_remain <= dist_ramp)
                    or (self._velo_set < 0 and dist_remain >= -dist_ramp)):
                self._velo_set = 0
        # calculate commanded wheel angles from velocity and time step
        self._theta_L += (self._velo_curr * cnst.T_STEP) / (cnst.WHEEL_DIA/2)
        self._theta_R += (self._velo_curr * cnst.T_STEP) / (cnst.WHEEL_DIA/2)

    def _rotate_control(self):
        """Calculates kinematics and wheel rotation for rotation.

        Notes
        -----
        At each time step, the angular velocity, angle, and wheel angles
        are calculated to command rotation in place. The wheel diameter
        and span are used to determine the amount of wheel rotation.
        The angular speed ramps up at the start and down at the end of
        the rotation to approximate constant angular acceleration.

        """

        # check for end of rotation and resume linear or arc motion as needed
        if self._ang_velo_set == 0 and self._ang_velo_curr == 0:
            # if stopped short using pause method, return
            if self._motion_curr[0] == 'pause':
                self._motion_curr = ['ready'] # swapped to avoid race condition
                if not self._motion_queue:
                    self._protect = False
                self._motion_state = 'pause' # swapped to avoid race condition
                return
            # if actual end of rotation, check for remaining error and adjust
            elif abs(self._ang_set - self._ang_curr) > 0.0087:
                self._theta_R += ((self._ang_set - self._ang_curr)
                                  * (cnst.WHEEL_SPAN/cnst.WHEEL_DIA))
                self._theta_L -= ((self._ang_set - self._ang_curr)
                                  * (cnst.WHEEL_SPAN/cnst.WHEEL_DIA))
                self._ang_curr = self._ang_set
                return
            # resume interrupted linear or arc motion
            elif self._motion_resume[0] != 'ready':
                self._motion_prev = self._motion_resume.copy()
                self._motion_resume = ['ready']
                self._motion_state = 'pause'
                self._motion_curr = ['pause']
                self._resume()
            # at end, set motion_state to 'stop' to end rotation
            else:
                self._motion_state = 'stop'
                if not self._motion_queue:
                    self._protect = False
                return
        # ramp the angular velocity up or down or hold steady at setpoint
        self._ang_velocity_ramp()
        # calculate current rotation angle for discrete move
        if self._ang_set != 0:
            self._ang_curr += self._ang_velo_curr * cnst.T_STEP
            # calculate remaining and rampdown angles
            ang_remain = self._ang_set - self._ang_curr
            ang_ramp = self._ang_velo_curr**2 / (2*cnst.ANG_ACC*(pi/180))
            # check for ramp down at end of rotation
            if ((self._ang_velo_set > 0 and ang_remain <= ang_ramp)
                    or (self._ang_velo_set < 0 and ang_remain >= -ang_ramp)):
                self._ang_velo_set = 0
        # calculate commanded wheel angles
        self._theta_R += (self._ang_velo_curr * cnst.T_STEP
                          * (cnst.WHEEL_SPAN/cnst.WHEEL_DIA))
        self._theta_L -= (self._ang_velo_curr * cnst.T_STEP
                          * (cnst.WHEEL_SPAN/cnst.WHEEL_DIA))

    def _arc_control(self):
        """Calculates kinematics and wheel rotation for arc motion.

        Notes
        -----
        At each time step, the velocity, arc length, and wheel angles
        are calculated to command a forward (or reverse) arc path. The
        velocity ramps up at the start and down at the end of the path
        to approximate constant acceleration.

        """

        # check for end of discrete move
        if self._velo_set == 0 and self._velo_curr == 0:
            # if stopped short using pause method, return
            if self._motion_curr[0] == 'pause':
                self._motion_curr = ['ready'] # swapped to avoid race condition
                if not self._motion_queue:
                    self._protect = False
                self._motion_state = 'pause' # swapped to avoid race condition
                return
            # if actual end of move, check for remaining error and adjust
            elif abs(self._arc_len_set - self._arc_len_curr) > 0.1:
                self._theta_L += ((self._diff_L * (self._arc_len_set
                                                   - self._arc_len_curr))
                                  / (cnst.WHEEL_DIA/2))
                self._theta_R += ((self._diff_R * (self._arc_len_set
                                                   - self._arc_len_curr))
                                  / (cnst.WHEEL_DIA/2))
                self._arc_len_curr = self._arc_len_set
                return
            # at end, set motion_state to 'stop' to end arc
            else:
                self._motion_state = 'stop'
                if not self._motion_queue:
                    self._protect = False
                return
        # ramp the velocity up or down or hold steady at setpoint
        self._velocity_ramp()
        # calculate current arc length for discrete move
        if self._arc_len_set != 0:
            self._arc_len_curr += self._velo_curr * cnst.T_STEP
            # calculate remaining and rampdown arc length
            arc_remain = self._arc_len_set - self._arc_len_curr
            arc_ramp = self._velo_curr**2 / (2*cnst.ACC_MAX)
            # check for ramp down at end of arc
            if ((self._velo_set > 0 and arc_remain <= arc_ramp)
                    or (self._velo_set < 0 and arc_remain >= -arc_ramp)):
                self._velo_set = 0
        # calculate commanded wheel angles from velocity and time step
        self._theta_L += ((self._diff_L * self._velo_curr * cnst.T_STEP)
                          / (cnst.WHEEL_DIA/2))
        self._theta_R += ((self._diff_R * self._velo_curr * cnst.T_STEP)
                          / (cnst.WHEEL_DIA/2))

    def _steer_control(self):
        """Calculates kinematics and wheel rotation for steer motion.

        Notes
        -----
        At each time step, the velocity, arc length, and wheel angles
        are calculated to command a forward (or reverse) steering path.
        The velocity ramps up at the start and down at the end of the
        path to approximate constant acceleration.

        """

        # check for stop command
        if self._velo_set == 0 and self._velo_curr == 0:
            # if stopped short using pause method, return
            if self._motion_curr[0] == 'pause':
                self._motion_curr = ['ready'] # swapped to avoid race condition
                if not self._motion_queue:
                    self._protect = False
                self._motion_state = 'pause' # swapped to avoid race condition
                return
        # check for end of steering command
        if ((self._arc_len_set > 0 and (self._arc_len_set
                                        - self._arc_len_curr) <= 0)
            or (self._arc_len_set < 0 and (self._arc_len_set
                                           - self._arc_len_curr) >= 0)):
            # resume interrupted linear or arc motion
            self._motion_curr = self._motion_resume.copy()
            self._motion_resume = ['ready']
            return
        # ramp the velocity up or down or hold steady at setpoint
        self._velocity_ramp()
        # update arc_len_curr at each time step
        self._arc_len_curr += self._velo_curr * cnst.T_STEP
        # calculate commanded wheel angles from velocity and time step
        self._theta_L += ((self._diff_L * self._velo_curr * cnst.T_STEP)
                          / (cnst.WHEEL_DIA/2))
        self._theta_R += ((self._diff_R * self._velo_curr * cnst.T_STEP)
                          / (cnst.WHEEL_DIA/2))

    def _control_action(self):
        """Checks encoders and sets motor output to take control action.

        Notes
        -----
        The current wheel angles are read from the encoders, and then
        the setpoint and actual values are passed to the PID algorithm
        to calculate the required settings for the motors. If the robot
        is stopped, no control action is taken and the motors are turned
        off. It is also possible to have the motors hold in place when
        the robot it stopped, but that option is not used because it
        would drain the batteries even while the robot is idle. The
        position and heading of the robot are also updated.

        """

        # test to see if control action is required
        if self._motion_state not in('stop', 'pause'):
            # get wheel angular positions from encoders
            self._theta_curr_L, self._theta_curr_R = self._motors._read_angles()
            # calculate position and heading from encoder incremental readings
            self._tracking()
            # update previous wheel angles for next control loop iteration
            self._theta_prev_L = self._theta_curr_L
            self._theta_prev_R = self._theta_curr_R
            # calculate closed-loop output
            u_L = self._pid._calculate(self._theta_L, self._theta_curr_L)
            u_R = self._pid._calculate(self._theta_R, self._theta_curr_R)
            # command motor output
            self._motors._set_output(round(u_L),round(u_R))
        # if no control action is required, turn off motors
        else:
            self._motors._set_output(0, 0)

    def _tracking(self):
        """Calculates the position and heading at each time step.

        Notes
        -----
        This tracking approach uses numerical integration to sum the
        small movements of the robot during each time step using the
        actual wheel angles measured by the encoders. The heading angle
        is also tracked by calculating and summing the rotation angle
        caused by the differential rotation of the wheels. This approach
        cannot account for wheel slippage and may not be perfectly
        calibrated to the wheel diameter and span, which will lead to
        accumulation of errors.

        """

        # lock to prevent setting position or heading while calculating value
        self._tracking_lock = True
        # start with incremental wheel rotation for current time step
        delta_theta_L = self._theta_curr_L-self._theta_prev_L
        delta_theta_R = self._theta_curr_R-self._theta_prev_R
        # set threshold for small or zero angle where r becomes infinite
        if abs(delta_theta_L - delta_theta_R) < .001:
            # increment [x, y] with forward wheel rotation and heading angle
            self._position[0] += ((cnst.WHEEL_DIA/2) * delta_theta_L
                                  * cos(self._heading))
            self._position[1] += ((cnst.WHEEL_DIA/2) * delta_theta_L
                                  * sin(self._heading))
        else:
            # increment [x, y] based on differential between wheels
            # radius of arc to center point of robot
            r = (cnst.WHEEL_SPAN/2) * ((delta_theta_R + delta_theta_L)
                                       / (delta_theta_R - delta_theta_L))
            # angle of arc created by differential between wheels
            alpha = ((cnst.WHEEL_DIA/(4*(cnst.WHEEL_SPAN/2)))
                     * (delta_theta_R - delta_theta_L))
            # length of vector from current position to new position
            l = 2 * r * sin(alpha/2)
            # calculate [x, y] based on vector, heading, and arc angle
            self._position[0] += l * cos(self._heading + alpha/2)
            self._position[1] += l * sin(self._heading + alpha/2)
            # update heading to include arc angle
            self._heading += alpha
            # adjust heading to keep range in -pi to pi
            if self._heading > pi:
                self._heading -= 2*pi
            elif self._heading < -pi:
                self._heading += 2*pi
        # unlock to allow setting position or heading after calculating value
        self._tracking_lock = False

    def _velocity_ramp(self):
        """Calculate the velocity for linear and arc motion."""

        # adjust current velocity only if different from set velocity
        if self._velo_curr != self._velo_set:
            # when current velocity is within threshold, assign as set velocity
            if (abs(self._velo_set - self._velo_curr)
                    < cnst.ACC_MAX * cnst.T_STEP):
                self._velo_curr = self._velo_set
            # when current velocity is lower than set velocity, ramp up
            elif self._velo_curr < self._velo_set:
                self._velo_curr += cnst.ACC_MAX * cnst.T_STEP
            # when current velocity is higher than set velocity, ramp down
            elif self._velo_curr > self._velo_set:
                self._velo_curr -= cnst.ACC_MAX * cnst.T_STEP

    def _ang_velocity_ramp(self):
        """Calculate the angular velocity for rotation."""

        # adjust current velocity only if different from set velocity
        if self._ang_velo_curr != self._ang_velo_set:
            # convert to rad/s^2
            ANG_ACC = cnst.ANG_ACC * (pi/180)
            # when current velocity is within threshold, assign as set velocity
            if (abs(self._ang_velo_set - self._ang_velo_curr)
                < ANG_ACC * cnst.T_STEP):
                self._ang_velo_curr = self._ang_velo_set
            # when current velocity is lower than set velocity, ramp up
            elif self._ang_velo_curr < self._ang_velo_set:
                self._ang_velo_curr += ANG_ACC * cnst.T_STEP
            # when current velocity is higher than set velocity, ramp down
            elif self._ang_velo_curr > self._ang_velo_set:
                self._ang_velo_curr -= ANG_ACC * cnst.T_STEP

    def _wheel_diff(self, radius, sense):
        """Calculate differential wheel rotation for desired arc motion.

        Parameters
        ----------
        radius : int or float
            The arc radius in cm (must be positive).
        sense : {'counterclockwise', 'clockwise'}
            The rotation sense for robot to traverse.

        """

        # calculate the differential rotation for outer and inner wheels
        diff_outer = 1 + ((cnst.WHEEL_SPAN/2)/radius)
        diff_inner = 1 - ((cnst.WHEEL_SPAN/2)/radius)
        # use sense to determine which wheel is the outer or inner
        if sense == 'counterclockwise':
            self._diff_R = diff_outer
            self._diff_L = diff_inner
        else:
            self._diff_L = diff_outer
            self._diff_R = diff_inner

    def _buzzer_control(self):
        """Controls the timing of buzzer pulses.

        Notes
        -----
        The control of buzzer pulses happens inside the control loop on
        the second core to allow for accurate timing of pulses while the
        main program thread is free to perform other tasks.

        """

        # reset the _buzzer._start attribute at the beginning of each pulse
        if self._buzzer._start is None:
            self._buzzer._start = self._steps
        # check to see if pulse should be on
        if self._steps < self._buzzer._start + self._buzzer._on_steps:
            self._buzzer._pwm.duty_u16(self._buzzer._level)
        # check to see if pulse should be off
        elif self._steps < self._buzzer._start + 2*self._buzzer._on_steps:
            self._buzzer._pwm.duty_u16(0)
        # check for continuous bursts and restart buzzer control immediately
        elif self._buzzer._bursts == -1:
            self._buzzer._start = None
            self._buzzer_control()
        # decrement bursts and restart buzzer control as needed to count pulses
        else:
            self._buzzer._bursts -= 1
            self._buzzer._start = None
            if self._buzzer._bursts > 0:
                self._buzzer_control()
            # when last pulse is reached, reset flag to turn off pulses
            elif self._buzzer._bursts <= 0:
                self._buzzer._pulse_set = False

    def _leds_control(self):
        """Controls the timing of LED pulses on both LEDs.

        Notes
        -----
        The control of LED pulses happens inside the control loop on
        the second core to allow for accurate timing of pulses while the
        main program thread is free to perform other tasks.

        """

        # create local copy to avoid issues with race conditions from user call
        pulse = self._leds._pulse_set
        if not pulse:
            return
        # reset the _leds._start attribute at the beginning of each pulse
        if self._leds._start == None:
            self._leds._start = self._steps
        # check to see if pulse should be on
        if self._steps < self._leds._start + pulse[1]:
            # check for fade setting
            if pulse[2]:
                color = list(pulse[0])
                # ramp up led brightness linearly based on number of steps
                for i in range(3):
                    color[i] = int(color[i]
                                   * (self._steps - self._leds._start)
                                   /pulse[1])
                self._leds._np.fill(tuple(color))
            # if fade is not set, simply set color for both LEDs
            else:
                self._leds._np.fill(pulse[0])
        # check to see if pulse should be off
        elif self._steps < self._leds._start + 2*pulse[1]:
            # check for fade setting
            if pulse[2]:
                color = list(pulse[0])
                # ramp down led brightness linearly based on number of steps
                for i in range(3):
                    color[i] = int(color[i]
                                   * (2 - (self._steps
                                           - self._leds._start
                                           + 1)/pulse[1]))
                self._leds._np.fill(tuple(color))
            # if fade is not set, check for already lit LEDs before turning off
            else:
                if self._leds._lit_left and self._leds._lit_right:
                    self._leds._np.fill(self._leds._color)
                elif self._leds._lit_right and not self._leds._lit_left:
                    self._leds._np[0] = self._leds._color_right
                    self._leds._np[1] = (0, 0, 0)
                elif self._leds._lit_left and not self._leds._lit_right:
                    self._leds._np[0] = (0, 0, 0)
                    self._leds._np[1] = self._leds._color_left
                else:
                    self._leds._np.fill((0, 0, 0))
        # check for continuous bursts and restart leds control immediately
        elif self._leds._bursts == -1:
            self._leds._start = self._steps
            self._leds_control()
        # decrement bursts and restart leds control as needed to count pulses
        else:
            self._leds._bursts -= 1
            if self._leds._bursts > 0:
                self._leds._start = self._steps
                self._leds_control()
            # when last pulse is reached, reset flag to turn off pulses
            elif self._leds._bursts == 0:
                self._leds._start = None
                self._leds._pulse_set = None
                # if fade is set at last pulse, check for already lit LEDs
                if pulse[2]:
                    if self._leds._lit_left and self._leds._lit_right:
                        self._leds._np.fill(self._leds._color)
                    elif self._leds._lit_right and not self._leds._lit_left:
                        self._leds._np[0] = self._leds._color_right
                        self._leds._np[1] = (0, 0, 0)
                    elif self._leds._lit_left and not self._leds._lit_right:
                        self._leds._np[0] = (0, 0, 0)
                        self._leds._np[1] = self._leds._color_left
                    self._leds._np.write()

    def _leds_control_right(self):
        """Controls the timing of LED pulses on the right LED only.

        Notes
        -----
        The control of LED pulses happens inside the control loop on
        the second core to allow for accurate timing of pulses while the
        main program thread is free to perform other tasks.

        """

        # create local copy to avoid issues with race conditions from user call
        pulse = self._leds._pulse_right_set
        if not pulse:
            return
        # reset the _leds._start_right attribute at the beginning of each pulse
        if self._leds._start_right == None:
            self._leds._start_right = self._steps
        # check to see if pulse should be on
        if self._steps < self._leds._start_right + pulse[1]:
            # check for fade setting
            if pulse[2]:
                color = list(pulse[0])
                # ramp up led brightness linearly based on number of steps
                for i in range(3):
                    color[i] = int(color[i]
                                   * (self._steps - self._leds._start_right)
                                   /pulse[1])
                self._leds._np[0] = tuple(color)
            # if fade is not set, simply set color for both LEDs
            else:
                self._leds._np[0] = pulse[0]
        # check to see if pulse should be off
        elif self._steps < self._leds._start_right + 2*pulse[1]:
            # check for fade setting
            if pulse[2]:
                color = list(pulse[0])
                # ramp down led brightness linearly based on number of steps
                for i in range(3):
                    color[i] = int(color[i]
                                   * (2 - (self._steps
                                           - self._leds._start_right
                                           + 1)/pulse[1]))
                self._leds._np[0] = tuple(color)
            # if fade is not set, check for already lit LEDs before turning off
            elif not self._leds._pulse_set:
                if self._leds._lit_left and self._leds._lit_right:
                    self._leds._np[0] = self._leds._color
                elif self._leds._lit_right and not self._leds._lit_left:
                    self._leds._np[0] = self._leds._color_right
                else:
                    self._leds._np[0] = (0, 0, 0)
        # check for continuous bursts and restart leds control immediately
        elif self._leds._bursts_right == -1:
            self._leds._start_right = self._steps
            self._leds_control_right()
        # decrement bursts and restart leds control as needed to count pulses
        else:
            self._leds._bursts_right -= 1
            if self._leds._bursts_right > 0:
                self._leds._start_right = self._steps
                self._leds_control_right()
            # when last pulse is reached, reset flag to turn off pulses
            elif self._leds._bursts_right == 0:
                self._leds._start_right = None
                self._leds._pulse_right_set = None
                # if fade is set at last pulse, check for already lit LEDs
                if pulse[2]:
                    if self._leds._lit_left and self._leds._lit_right:
                        self._leds._np[0] = self._leds._color
                    elif self._leds._lit_right and not self._leds._lit_left:
                        self._leds._np[0] = self._leds._color_right
                    self._leds._np.write()

    def _leds_control_left(self):
        """Controls the timing of LED pulses on the left LED only.

        Notes
        -----
        The control of LED pulses happens inside the control loop on
        the second core to allow for accurate timing of pulses while the
        main program thread is free to perform other tasks.

        """

        # create local copy to avoid issues with race conditions from user call
        pulse = self._leds._pulse_left_set
        if not pulse:
            return
        # reset the _leds._start_left attribute at the beginning of each pulse
        if self._leds._start_left == None:
            self._leds._start_left = self._steps
        # check to see if pulse should be on
        if self._steps < self._leds._start_left + pulse[1]:
            # check for fade setting
            if pulse[2]:
                color = list(pulse[0])
                # ramp up led brightness linearly based on number of steps
                for i in range(3):
                    color[i] = int(color[i]
                                   * (self._steps - self._leds._start_left)
                                   /pulse[1])
                self._leds._np[1] = tuple(color)
            # if fade is not set, simply set color for both LEDs
            else:
                self._leds._np[1] = pulse[0]
        # check to see if pulse should be off
        elif self._steps < self._leds._start_left + 2*pulse[1]:
            # check for fade setting
            if pulse[2]:
                color = list(pulse[0])
                # ramp down led brightness linearly based on number of steps
                for i in range(3):
                    color[i] = int(color[i]
                                   * (2 - (self._steps
                                           - self._leds._start_left
                                           + 1)/pulse[1]))
                self._leds._np[1] = tuple(color)
            # if fade is not set, check for already lit LEDs before turning off
            elif not self._leds._pulse_set:
                if self._leds._lit_left and self._leds._lit_right:
                    self._leds._np[1] = self._leds._color
                elif self._leds._lit_left and not self._leds._lit_right:
                    self._leds._np[1] = self._leds._color_left
                else:
                    self._leds._np[1] = (0, 0, 0)
        # check for continuous bursts and restart leds control immediately
        elif self._leds._bursts_left == -1:
            self._leds._start_left = self._steps
            self._leds_control_left()
        # decrement bursts and restart leds control as needed to count pulses
        else:
            self._leds._bursts_left -= 1
            if self._leds._bursts_left > 0:
                self._leds._start_left = self._steps
                self._leds_control_left()
            # when last pulse is reached, reset flag to turn off pulses
            elif self._leds._bursts_left == 0:
                self._leds._start_left = None
                self._leds._pulse_left_set = None
                # if fade is set at last pulse, check for already lit LEDs
                if pulse[2]:
                    if self._leds._lit_left and self._leds._lit_right:
                        self._leds._np[1] = self._leds._color
                    elif self._leds._lit_left and not self._leds._lit_right:
                        self._leds._np[1] = self._leds._color_left
                    self._leds._np.write()


class MotorPID:
    """
    Creates a discrete-time PID controller for PiBOT DC gearmotors.

    ...

    Notes
    -----
    Adapted for Physics 360 PiBOT at UW-Eau Claire from post by Eduardo
    Nigro, Mechatronics Engineer, April 7, 2022. Original post can be
    found at: https://thingsdaq.org/2022/04/07/digital-pid-controller/

    """

    def __init__(self):
        """Creates PID control attributes."""

        # initialize variables
        self._eprev = [0, 0]
        self._uprev = 0
        self._udfiltprev = 0

    def _calculate(self, ysp, y):
        """Calculate control action.

        Parameters
        ----------
        ysp : float
            The desired set point as a wheel rotation angle in radians.
        y : float
            The measured wheel angle in radians based on encoder.

        Returns
        -----
        float
            The calculated control action for the motor as a PWM on time
            in ns.

        """

        # calculate error e[n]
        e = ysp - y
        # calculate proportional term
        up = cnst.K_P * (e - self._eprev[0])
        # calculate integral term (with anti-windup)
        ui = cnst.K_I * cnst.T_STEP * e
        if self._uprev >= cnst.MAX_ON_TIME or self._uprev <= -cnst.MAX_ON_TIME:
            ui = 0
        # calculate derivative term
        ud = (cnst.K_D/cnst.T_STEP * (e - 2*self._eprev[0] + self._eprev[1]))
        # filter derivative term
        udfilt = ((cnst.TAU_PID/(cnst.TAU_PID+cnst.T_STEP))*self._udfiltprev
                  + cnst.T_STEP/(cnst.TAU_PID+cnst.T_STEP)*ud)
        # calculate PID controller output u[n]
        u = self._uprev + up + ui + udfilt
        if u > cnst.MAX_ON_TIME:
            u = cnst.MAX_ON_TIME
        if u < -cnst.MAX_ON_TIME:
            u = -cnst.MAX_ON_TIME
        # update previous time step errors e[n-1], e[n-2]
        self._eprev[1] = self._eprev[0]
        self._eprev[0] = e
        # update previous time step output value u[n-1]
        self._uprev = u
        # update previous time step derivative term filtered value
        self._udfiltprev = udfilt
        # return controller output at current time step
        return u


class MotorControl():
    """Low level motor and encoder functions for PiBOT."""

    def __init__(self):
        """Creates motor and rotary encoder control attributes."""
        
        # creates instances of the MotorPWM class to command motors
        self._wheel_L = MotorPWM(cnst.FORWARD_LEFT,
                                 cnst.REVERSE_LEFT)
        self._wheel_R = MotorPWM(cnst.FORWARD_RIGHT,
                                 cnst.REVERSE_RIGHT)
        # creates instances of the QuadEncoder class to track encoder counts
        self._count_L = QuadEncoder(0, (Pin(cnst.ENCODER_LEFT_A, Pin.IN),
                                        Pin(cnst.ENCODER_LEFT_B, Pin.IN)))
        self._count_R = QuadEncoder(1, (Pin(cnst.ENCODER_RIGHT_A, Pin.IN),
                                        Pin(cnst.ENCODER_RIGHT_B, Pin.IN)))

    def _set_output(self, on_time_L, on_time_R):
        """Sets the left and right motor outputs.

        Parameters
        ----------
        on_time_L : int
            On time in ns for the left motor PWM.
        on_time_R : int
            On time in ns for the right motor PWM.

        """

        self._wheel_L._output(on_time_L)
        self._wheel_R._output(on_time_R)

    def _read_angles(self):
        """Reads the current angles of the encoders.

        Returns
        -------
        2-tuple of floats
            The left and right wheel angles in radians

        Notes
        -----
        Gets raw counts of both left and right encoders and converts
        to wheel angles in radians using encoder resolution (i.e.,
        counts per wheel revolution).

        """
        # gets raw counts from encoders using instances of QuadEncoder class
        raw_count_L = self._count_L._get_count()
        raw_count_R = self._count_R._get_count()
        # calculate the wheel angles in radians
        theta_L = raw_count_L * (2*pi / cnst.ENC_RESOLUTION)
        theta_R = raw_count_R * (2*pi / cnst.ENC_RESOLUTION)
        # return wheel angles
        return theta_L, theta_R


class MotorPWM():
    """
    Sets speed and direction of a motor using machine.PWM class.

    ...

    Parameters
    ----------
    fwd_pin : int
        GPIO pin number for the forward rotation direction.
    rev_pin : int
        GPIO pin number for the reverse rotation direction.
    freq : int, default=PWM_FREQ
        PWM frequency value in Hz.

    Notes
    -----
    The forward and reverse directions are determined by MAKER-PI-RP2040
    pin assignments for motors 1 and 2. The maximum PWM frequency is 20
    kHz for the MAKER-PI board and is set at 10 kHz for the PiBOT.

    """

    def __init__(self, fwd_pin, rev_pin, freq=cnst.PWM_FREQ):
        """Creates PWM instances for forward/backward motor control."""

        self._fwd_motor = PWM(Pin(fwd_pin))
        self._rev_motor = PWM(Pin(rev_pin))
        self._fwd_motor.freq(freq)
        self._rev_motor.freq(freq)

    def _output(self, on_time):
        """Sends pulses to motor at specified on_time (pulse width).

        Parameters
        ----------
        on_time : int
            Pulse width in ns for the PWM motor speed control. Sign
            determines direction of rotation (positive is forward).
            Setting on_time to zero stops the motor.

        Notes
        -----
        The PWM duty cycle can be determined from PWM_FREQ and on_time.
        For example, with PWM_FREQ at 10_000 Hz, the period is 100
        microseconds (or 100_000 ns). With an on-time at the maximum of
        75_000 ns, the duty cycle will be 75%. This means the motor's
        limit is roughly 75% of its possible commanded speed. This limit
        is intentional because at higher speeds the PiBOT goes too fast,
        does wheelies, and could be damaged in collisions.

        """

        # limiting output to maximum
        if on_time > cnst.MAX_ON_TIME:
            on_time = cnst.MAX_ON_TIME
        elif on_time < -cnst.MAX_ON_TIME:
            on_time = -cnst.MAX_ON_TIME
        # setting forward rotation
        if on_time > 0:
            self._fwd_motor.duty_ns(on_time)
            self._rev_motor.duty_ns(0)
        # setting reverse rotation
        elif on_time < 0:
            self._fwd_motor.duty_ns(0)
            self._rev_motor.duty_ns(-on_time)    
        # stop motor
        elif on_time == 0:
            self._fwd_motor.duty_ns(0)
            self._rev_motor.duty_ns(0)


class QuadEncoder:
    """Creates a state machine using PIO to read encoders.

    ...

    Parameters
    ----------
    sm_id : {0, 1, 2, 3}
        The state machine address.
    pins : object of machine.Pin class
        Pins for A & B encoder channels (A & B must be consecutive pins
        or the state machine won't work).
    freq : int, default=125_000_000
        State machine clock frequency in Hz (system clock max).

    Notes
    -----
    The quadrature rotary incremental encoders on the PiBOT are read and
    their values are stored using the PIO (programmable input/output)
    features of the RP2040. The original version comes from:

    (c) 2021 pmarques-dev @ github
    (https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.pio)

    It was adapted and posted for micropython April 17, 2022 by rkompass
    (https://forum.micropython.org/viewtopic.php?t=12277&p=66659)
    SPDX-License-Identifier: BSD-3-Clause

    This PiBOT version was adapted from the post by rkompass.

    """
    def __init__(self, sm_id, pins, freq=125_000_000):
        """Creates state machines for A & B encoder channels."""

        # set the in_base to the GPIO for channel A
        in_base = pins[0]
        # create instance of StateMachine to call static method for PIO
        # in_base sets where the ISR starts to gather input from GPIO
        # out_base sets where the OSR starts setting outputs on GPIO
        self._state_machine_quad_enc = StateMachine(sm_id,
                                                    self._state_machine_quad_enc,
                                                    freq=freq, in_base=in_base,
                                                    out_base=in_base)
        # set X register to zero to clear encoder count at start
        self._state_machine_quad_enc.exec("set(x, 0)")
        # read in state of encoder A and B pins (starting at in_base pin)
        self._state_machine_quad_enc.exec("in_(pins, 2)")
        # activate the state machine
        self._state_machine_quad_enc.active(1)
    
    @asm_pio(in_shiftdir=PIO.SHIFT_LEFT, out_shiftdir=PIO.SHIFT_RIGHT)
    def _state_machine_quad_enc():
        """Special assembly instructions for the state machine

        Notes
        -----
        This state machine uses a state table concept with the jmp
        instruction to assign the next appropiate action to take based
        on the previous and current values of the A and B pins of the
        encoder. The encoder count is stored in the X scratch register
        and pushed to the RX FIFO via the ISR. Because the RX FIFO only
        has a 4-word (32-bit words) register length, it quickly fills.
        The push(noblock) instruction ensures the state machine does
        not stall when the FIFO fills, but it does not allow more words
        to be added until the FIFO register is cleared using the
        StateMachine.get() method. The nop() instructions at the end are
        place holders to ensure there are exactly the maximum of 32
        instructions (labels don't count as instructions). Filling the
        instruction memory ensures that the first instruction starts at
        0000 so that the state table works properly with the
        move(pc, isr) instruction that is used after reading the
        encoder. The state machine program is fast and uses no
        interrupts. The worst case sampling loop (a read and increment)
        takes 12 cycles, so this program is able to monitor encoder step
        rates up to sysclk / 12 (e.g., at 125 MHz, max step rate = 10.4
        Msteps/sec).

        """

        jmp("read")        # 0000 : from 00 to 00 = no change
        jmp("decr")        # 0001 : from 00 to 01 = reverse
        jmp("incr")        # 0010 : from 00 to 10 = forward
        jmp("read")        # 0011 : from 00 to 11 = error
        jmp("incr")        # 0100 : from 01 to 00 = forward
        jmp("read")        # 0101 : from 01 to 01 = no change
        jmp("read")        # 0110 : from 01 to 10 = error
        jmp("decr")        # 0111 : from 01 to 11 = reverse
        jmp("decr")        # 1000 : from 10 to 00 = reverse
        jmp("read")        # 1001 : from 10 to 01 = error
        jmp("read")        # 1010 : from 10 to 10 = no change
        jmp("incr")        # 1011 : from 10 to 11 = forward
        jmp("read")        # 1100 : from 11 to 00 = error
        jmp("incr")        # 1101 : from 11 to 01 = forward

        label("decr")      # jump here to decrement count and return to read
        jmp(x_dec, "read") # 1110 : from 11 to 10 = reverse
                           
        label("read")      # jump here to read encoder pin values
        mov(osr, isr)      # 1111 : from 11 to 11 = no change--at read, no jump
        mov(isr, x)        # previous pin input into OSR, X (enc cnt) into ISR
        push(noblock)      # push ISR to RX FIFO without blocking
        out(isr, 2)        # previous pin input from OSR back into ISR
        in_(pins, 2)       # combine previous pin input with current reading
        mov(pc, isr)       # jump into table at 4-bit position encoded in ISR

        label("incr")      # jump here to increment count
        mov(x, invert(x))  # start by inverting count
        jmp(x_dec, "here") # decrement inverted count
        label("here")      # this label is used to continue after decrement
        mov(x, invert(x))  # re-invert count to complete the increment
        jmp("read")        # return to read
        
        nop()              # nop() instructions to fill program memory to 32
        nop()
        nop()
        nop()
        nop()
        nop()
        nop()              # 11111 -- full program instructions

    def _get_count(self):
        """Gets the current encoder count from the X scratch register.

        Returns
        -------
        int
            The raw encoder count as a signed integer value.

        Notes
        -----
        The state machine runs at high clock speed and quickly fills the
        RX FIFO buffer. Once full, new values in the ISR are discarded
        because they can't fit. To get a fresh value, the FIFO buffer
        needs to be purged with the for routine (throws out four stale
        values). This is inefficient, but other methods are limited in
        MicroPython versus C. The use of
        range(self._state_machine_quad_enc.rx_fifo()) ensures the for
        loop range has the exact number needed to purge FIFO; however,
        the range is almost certainly going to be 4 (full FIFO) and
        could simply be replaced with 4.

        """
        # loop to purge stale values from RX FIFO.
        for _ in range(self._state_machine_quad_enc.rx_fifo()):
            self._state_machine_quad_enc.get()
        # get raw value of encoder count
        n = self._state_machine_quad_enc.get()
        # adjust 32-bit raw value to create signed value
        count = n if n < (1<<31) else n - (1<<32)
        # return signed integer count value
        return count