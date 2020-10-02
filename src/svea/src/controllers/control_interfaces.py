#!/usr/bin/env python

"""
ROS interface objects for sending control to the SVEA car's low-level
controller. The low-level controller expects unit-less steering and
velocity values from [-127, 127] that correspond to the minimum and
maximum steering angles and velocities. We implement the interfaces
for two reasons:
(1) our models typically expect steering angles in [rads] and velocities
in [m/s], and
(2) we would like to add some features on top of just sending the
control inputs, like persistent control or control blending.
"""

from threading import Thread
from math import radians, degrees, isnan

import rospy
from geometry_msgs.msg import Twist
from low_level_interface.msg import lli_ctrl_request
from svea_arduino.msg import lli_ctrl

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class ControlInterface():
    """Standard control interface. Make it easy to input steering
    angles in radians and velocities in m/s. Assumes very simplistic
    models of the low-level actuation. It assumes a quadratic steering
    model and a linear velocity model.

    :param vehicle_name: Name of vehicle being controlled; used to
                         create ROS topic names for subscription and
                         publications, defaults to empty string
    :type vehicle_name: str
    """

    # arduino's expected input frequency
    OPERATING_FREQ = 30 # [Hz]
    # saturation input limits to keep hardware healthy
    MAX_STEER_PERCENT = 80 # [%]
    MAX_VELOCITY_PERCENT = 90 # [%]
    # assumed max velocity, probably wrong
    MAX_VELOCITY = 1 # [m/s]
    # scaling factor, percentage to lli actuation
    PERC_TO_LLI_COEFF = 1.27
    # binary masks
    TRANS_MASK = 0b00000001
    FDIFF_MASK = 0b00000010
    RDIFF_MASK = 0b00000100

    def __init__(self, vehicle_name=""):

        self.vehicle_name = vehicle_name

        self.ctrl_request = lli_ctrl_request() # Stores the last controls sent
        self.ctrl_msg = lli_ctrl()  # Message to send to ROS
        self.last_ctrl_update = rospy.get_time()

        self.is_stop = False
        self.is_emergency = False
        self.is_reverse = False
        self.is_ready = False

        # log of control requests and control actuated
        self.ctrl_request_log = []
        self.ctrl_actuated_log = []
        self.remote_log = []

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: ControlInterface
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo(
                'Starting Controller Interface Node: \n'
                + str(self))
        self.node_name = self.vehicle_name + '_control_interface'

        self._start_publish()
        self._start_listen()
        self.is_ready = True
        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber(self.vehicle_name+'/lli/ctrl_actuated', lli_ctrl,
                         self._read_ctrl_actuated)
        rospy.Subscriber(self.vehicle_name+'/lli/remote', lli_ctrl,
                         self._read_remote)
        rospy.loginfo(
                "Controller Interface successfully initialized")

    def _start_publish(self):
        self.ctrl_request_pub = rospy.Publisher(self.vehicle_name+'/lli/ctrl_request',
                                                lli_ctrl,
                                                queue_size = 1)

    def _read_ctrl_actuated(self, msg):
        dead_zone = 2 # velocities with abs values <= than this = 0 to ESC
        velocity = msg.velocity
        try:
            prev_velocity = self.ctrl_actuated_log[-1].velocity
        except IndexError:
            self.is_reverse = False
            self.ctrl_actuated_log.append(msg)
            return
        reverse_detected = False
        if velocity > dead_zone:
            self.is_reverse = False
        elif prev_velocity < dead_zone and abs(velocity) <= dead_zone:
            reverse_detected = True
            self.is_reverse = True
        #rospy.loginfo("act velocity: {}, prev vel: {}, reverse: {}, reverse detect: {}".format(velocity, prev_velocity, self.is_reverse, reverse_detected))
        self.ctrl_actuated_log.append(msg)

    def _read_remote(self, msg):
        self.remote_log.append(msg)

    def _build_param_printout(self):
        # collect important params
        steering = self.ctrl_request.steering
        velocity = self.ctrl_request.velocity
        transmission = self.ctrl_request.transmission
        differential_front = self.ctrl_request.differential_front
        differential_rear = self.ctrl_request.differential_rear
        ctrl_code = self.ctrl_request.ctrl_code

        return ("## Vehicle: {0}\n".format(self.vehicle_name)
                +"  -ctrl request:\n"
                +"      steering   - {0}\n".format(steering)
                +"      velocity   - {0}\n".format(velocity)
                +"      trans      - {0}\n".format(transmission)
                +"      diff_front - {0}\n".format(differential_front)
                +"      diff_rear  - {0}\n".format(differential_rear)
                +"      ctrl_code  - {0}\n".format(ctrl_code)
                +"  -Is stopped: {0}\n".format(self.is_stop)
                +"  -In reverse: {0}\n".format(self.is_reverse))

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()

    def _steer_to_percent(self, steering):
        steering = float(steering)
        if degrees(steering) >= 0:
            steer_PWM = (1495 - 5.016 * degrees(steering)
                         - 0.2384 * degrees(steering)**2)
            steer_percent = (steer_PWM-1500)/600 * 100
        else:
            steer_PWM = (1495 - 5.016 * degrees(steering)
                          + 0.2384 * degrees(steering)**2)
            steer_percent = (steer_PWM-1500)/600 * 100

        steer_percent = -steer_percent # steering flipped

        return int(steer_percent)

    def _vel_to_percent(self, velocity):
        velocity = float(velocity)
        vel_percent = velocity/self.MAX_VELOCITY * 100
        return int(vel_percent)

    def _clip_ctrl(self, steer_percent, vel_percent):
        clipped_steering = self._clip_steering(steer_percent)
        clipped_velocity = self._clip_velocity(vel_percent)
        return clipped_steering, clipped_velocity

    def _clip_steering(self, steer_percent):
        return min(self.MAX_STEER_PERCENT,
                               max(-self.MAX_STEER_PERCENT, steer_percent))

    def _clip_velocity(self, vel_percent):
        return min(self.MAX_VELOCITY_PERCENT,
                               max(-self.MAX_VELOCITY_PERCENT, vel_percent))

    def _set_reverse(self, reverse):
        if self.is_reverse == reverse or self.is_stop:
            return
        try:
            current_velocity = self.ctrl_actuated_log[-1]
        except IndexError:
            current_velocity = 0
        reverse_msg = lli_ctrl()
        reverse_msg.steering = -128
        if reverse:
            reverse_msg.velocity = -10
            self.ctrl_request_pub.publish(reverse_msg)
            rospy.sleep(0.05)
            reverse_msg.velocity = 0
            self.ctrl_request_pub.publish(reverse_msg)
            rospy.sleep(0.05)
        else:
            reverse_msg.velocity = 10
            self.ctrl_request_pub.publish(reverse_msg)
            rospy.sleep(0.05)
            reverse_msg.velocity = current_velocity
            self.is_reverse = False

    def send_control(self,
                     steering=float('nan'),
                     velocity=float('nan'),
                     brake_force=0,
                     transmission=-1,
                     differential_front=-1,
                     differential_rear=-1,
                     ctrl_code=0):
        """Method for taking control inputs and implementing features
        over the control inputs. This method converts steering angles
        and velocities from radians and m/s to unit-less values that the
        low level system expects, saturates the unit-less values,
        implements a stopping feature for blocking control inputs (note,
        this is not necessarily a braking feature, it only blocks inputs
        from being published, thus it should not be used for emergency
        braking), and sends/publishes inputs to the low level interface.

        If an argument is left empty the low level interface will be
        told to use the last sent value. The same is true if the gear or
        differential arguments have any other values than 0 or 1. If you
        do *not* call send_control then the car will *not* do anything.

        :param steering: input steering angle for the car in [rad], if
                         argument left empty the low level system will
                         implement the last sent valid value, default
                         nan.
        :type steering: float
        :param velocity: input velocity for the car [m/s], if argument
                         left empty the low level system will implement
                         the last sent valid value, default nan
        :type velocity: float
        :param brake_force: brake force as a percentage [0, 100] of
                            maximum braking force, defaults to 0
        :type brake_force: float
        :param transmission: 0 means low gear, 1 means high gear, -1
                             means keep the currently set gear, defaults
                             to -1
        :type transmission: int
        :param differential_front: 0 means unlocked, 1 means locked, -1
                                   means keep the currently set lock
                                   state, defaults to -1
        :type differential_front: int
        :param differential_rear: 0 means unlocked, 1 means locked, -1
                                  means keep the currently set lock
                                  state, defaults to -1
        :type differential_rear: int
        :param ctrl_code: [deprecated]
        """
        if not isnan(steering):
            steer_percent = self._steer_to_percent(steering)
            steer_percent = self._clip_steering(steer_percent)
            self.ctrl_request.steering = steer_percent
            self.ctrl_msg.steering = \
                    round(steer_percent * self.PERC_TO_LLI_COEFF)
        else:
            self.ctrl_msg.steering = -128

        if brake_force > 0:
            if self.is_reverse:
                self._set_reverse(False)
            self.ctrl_msg.velocity = \
                    -round(brake_force * self.PERC_TO_LLI_COEFF)
            self.ctrl_request.velocity = 0
        elif not isnan(velocity):
            vel_percent = self._vel_to_percent(velocity)
            vel_percent = self._clip_velocity(vel_percent)
            self.ctrl_request.velocity = vel_percent
            self.ctrl_msg.velocity = \
                    round(vel_percent * self.PERC_TO_LLI_COEFF)
            if velocity < 0 and not self.is_reverse:
                self._set_reverse(True)
        else:
            self.ctrl_msg.velocity = -128

        self.ctrl_msg.trans_diff = 0
        if transmission in (0, 1):
            self.ctrl_request.transmission = transmission
            self.ctrl_msg.trans_diff ^= (1 << 3 ^ transmission << 0)
        if differential_front in (0, 1):
            self.ctrl_request.differential_front = differential_front
            self.ctrl_msg.trans_diff ^= (1 << 4 ^ differential_front << 1)
        if differential_front in (0, 1):
            self.ctrl_request.differential_rear = differential_rear
            self.ctrl_msg.trans_diff ^= (1 << 5 ^ differential_rear << 2)
        self.ctrl_msg.ctrl = ctrl_code
        self.ctrl_request.ctrl_code = ctrl_code

        if not self.is_stop:
            self.ctrl_request_pub.publish(self.ctrl_msg)
            self.ctrl_request_log.append(self.ctrl_request)

    def set_is_stop(self, is_stop = True):
        """Setter function for stopping the car. **This is not the same
        as emergency braking.**
        (will be updated to use properties)

        :param is_stop: flag for blocking or unblocking the control
                        inputs to the car, defaults to True.
        :type is_stop: bool
        """
        self.is_stop = is_stop


class ControlInterfaceWTeleop(ControlInterface):
    """
    Inherits from standard control interface but layers functions for
    receiving and implementing remote teleop control inputs from WebRTC
    app. This control interface is designed to handle two input sources
    and blend the inputs into one input; in particular, this is useful
    when there is a remote operator and on-board automation sending
    control inputs at the same time.

    :param vehicle_name: Name of vehicle being controlled; used to
                         create ROS topic names for subscription and
                         publications, defaults to empty string
    :type vehicle_name: str
    :param percent_teleop: percent from [0, 100], defaults to 100
    :type percent_teleop: float
    """

    def __init__(self, vehicle_name="", percent_teleop=100):
        ControlInterface.__init__(self, vehicle_name)

        self.teleop_cmd = Twist()

        self.percent_teleop = percent_teleop

    def _start_listen(self):
        rospy.Subscriber(self.vehicle_name+'/lli/ctrl_actuated', lli_ctrl_actuated,
                         self._read_ctrl_actuated)
        rospy.Subscriber('/teleop_cmd', Twist,
                         self._read_ctrl_teleop)
        rospy.loginfo(
                "Controller Interface successfully initialized")

    def _read_ctrl_teleop(self, msg):
        self.teleop_cmd = msg

    def _build_param_printout(self):
        # collect important params
        steering = self.ctrl_request.steering
        velocity = self.ctrl_request.velocity
        transmission = self.ctrl_request.transmission
        differential_front = self.ctrl_request.differential_front
        differential_rear = self.ctrl_request.differential_rear
        ctrl_code = self.ctrl_request.ctrl_code


        return ("## Vehicle: {0}\n".format(self.vehicle_name)
                +"  -ctrl request:\n"
                +"      steering   - {0}\n".format(steering)
                +"      velocity   - {0}\n".format(velocity)
                +"      trans      - {0}\n".format(transmission)
                +"      diff_front - {0}\n".format(differential_front)
                +"      diff_rear  - {0}\n".format(differential_rear)
                +"      ctrl_code  - {0}\n".format(ctrl_code)
                +"  -% teleop: {0}\n".format(self.percent_teleop)
                +"  -Is stopped: {0}\n".format(self.is_stop)
                +"  -Is emergency: {0}\n".format(self.is_emergency))

    def _combine(self, auto_steering, auto_velocity):
        teleop_steering = self.teleop_cmd.angular.z
        teleop_velocity = self.teleop_cmd.linear.x

        p = self.percent_teleop / 100
        steering = p * teleop_steering + (1-p) * auto_steering
        velocity = p * teleop_velocity + (1-p) * auto_velocity

        return int(steering), int(velocity)

    def send_control(self, steering = 0, velocity = 0,
                     transmission = 0,
                     differential_front = 0,
                     differential_rear = 0,
                     ctrl_code = 0):
        """
        Triggers control input to be sent over low-level-interface,
        computes linear combination of teleop control input and control
        inputs given as arguments to this function based on the set
        blending percent.

        :param steering: input steering angle for the car in [rad],
                         defaults to 0
        :type steering: float
        :param velocity: input velocity for the car in [m/s], defaults to
                         0
        :type velocity: float
        :param transmission: [deprecated]
        :param differential_front: [deprecated]
        :param differential_rear: [deprecated]
        :param ctrl_code: [deprecated]
        """

        steer_percent = self._steer_to_percent(steering)
        vel_percent = self._vel_to_percent(velocity)

        # bound input
        steer_percent, vel_percent = self._clip_ctrl(steer_percent, vel_percent)
        # fuse automation and teleop inputs
        steer_percent, vel_percent = self._combine(steer_percent, vel_percent)

        self.ctrl_request.steering = steer_percent
        self.ctrl_request.velocity = vel_percent
        self.ctrl_request.transmission = transmission
        self.ctrl_request.differential_front = differential_front
        self.ctrl_request.differential_rear = differential_rear
        self.ctrl_request.ctrl_code = ctrl_code

        if not self.is_emergency or not self.is_stop or not self.is_persistent:
            self.ctrl_request_pub.publish(self.ctrl_request)
            self.ctrl_request_log.append(self.ctrl_request)

    def set_ctrl_blend(percent_teleop):
        """
        Setter function for percent blend between remote control input
        and automated control input. Implemented as
        :math:`u = p u_h + (1-p) u_a`, where :math:`u_h, u_a, p` are
        the teleop input, automated input, and the percent/100.
        (will be updated to use properties)

        :param percent_teleop: perentage from [0, 100]
        :type percent_teleop: float
        """
        if percent_teleop >= 0 and percent_teleop <= 100:
            self.percent_teleop = percent_teleop
        else:
            rospy.loginfo("Value not between [0, 1]. Not setting control blend"
                          + "ratio")


class OldControlInterface():
    """
    [Deprecated] Control interface compliant with old arduino firmware
    where low level control is set by raw PWM values from 900-2100.
    """

    MAX_STEER_ANGLE = radians(30) #radians
    MAX_VELOCITY = 1 #[m/s]

    def __init__(self, vehicle_name):

        self.vehicle_name = vehicle_name

        self.ctrl_request = Twist()
        self.brake_request = Twist()
        self.last_ctrl_update = rospy.get_time()

        self.is_stop = False
        self.is_emergency = False

        # log of control requests and control actuated
        self.ctrl_request_log = []
        self.ctrl_actuated_log = []

    def start(self):
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo(
                'Starting Old Controller Interface Node: \n'
                + str(self))
        self.node_name = self.vehicle_name + '_old_control_interface'

        self._start_listen()
        self._start_publish()
        rospy.spin()

    def _start_listen(self):
        # nothing to subscribe to
        rospy.loginfo(
                "Old Controller Interface successfully initialized")

    def _start_publish(self):
        self.ctrl_request_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    def __repr__(self):
        # rospy.get_param('/blah')
        return "* <params should be here> *"
    def __str__(self):
        # rospy.get_param('/blah')
        return "* <params should be here> *"

    def brake(self):
        #TODO add timed braking
        self.brake_request.angular.z = self.ctrl_request.angular.z
        self.ctrl_request_pub.publish(self.brake_request)
        self.ctrl_request_log.append(self.brake_request)

    def send_PWM(self, steer_PWM, vel_PWM):
        self.ctrl_request.angular.z = steer_PWM
        self.ctrl_request.linear.x = vel_PWM

        if self.is_emergency or self.is_stop:
            self.brake()
        else:
            self.ctrl_request_pub.publish(self.ctrl_request)
            self.ctrl_request_log.append(self.ctrl_request)

    def _steer_to_PWM(self, steering):
        if degrees(steering) >= 0:
            steer_PWM = int(1495 - 5.016 * degrees(steering)
                            - 0.2384 * degrees(steering)**2)
        else:
            steer_PWM = int(1495 - 5.016 * degrees(steering)
                            + 0.2384 * degrees(steering)**2)
        return steer_PWM

    def _vel_to_PWM(self, velocity):
        vel_PWM = 1630
        return vel_PWM

    def _clip_ctrl(self, steering, velocity):
        clipped_steering = min(self.MAX_STEER_ANGLE,
                               max(-self.MAX_STEER_ANGLE, steering))
        #TODO add this in after finding velocity model
        clipped_velocity = min(self.MAX_VELOCITY,
                               max(-self.MAX_VELOCITY, steering))
        clipped_velocity = velocity

        return clipped_steering, clipped_velocity

    def send_control(self, steering, velocity,
                     gear, transmission = 0,
                     differential_front = 0,
                     differential_rear = 0,
                     ctrl_code = 0):

        steering, velocity = self._clip_ctrl(steering, velocity)

        steer_PWM = self._steer_to_PWM(steering)
        vel_PWM = self._vel_to_PWM(velocity)

        self.send_PWM(steer_PWM, vel_PWM)

    def set_is_stop(self, is_stop = True):
        self.is_stop = is_stop

    def set_is_emergency(self, is_emergency = True):
        self.is_emergency = is_emergency
        self.is_stop = is_emergency
        if is_emergency == True:
            rospy.loginfo("Blocking {0} for emergency".format(self.node_name))
        else:
            rospy.loginfo("Unblocking {0} after emergency".format(self.node_name))
