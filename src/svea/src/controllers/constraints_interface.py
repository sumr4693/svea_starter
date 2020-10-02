#!/usr/bin/env python


"""
The file includes two classes:
    ConstraintsInterface: An extension to ControlInterface.
    It handles speed control of the vehicle.

    BrakeControl: Sends brake signals to car is there is an emergency.
"""

import sys
import os
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from threading import Thread
from math import radians, degrees, isnan
from low_level_interface.msg import lli_ctrl_request
from svea_arduino.msg import lli_ctrl
from nav_msgs.msg import Odometry
from svea.msg import average_velocity
from std_msgs.msg import Bool

from control_interfaces import ControlInterface

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../scripts/real')
sys.path.append(os.path.abspath(svea))

from read_speed import ReadSpeed


class ConstraintsInterface(ControlInterface):
    """
    Extends the control interface to add constraints on speed limit using PID.
    
    :param vehicle_name: Name of vehicle being controlled; used to
                         create ROS topic names for subscription and
                         publications, defaults to empty string
    :type vehicle_name: str
    :param speed_limit: The speed limit of the vehicle. The controller in the
                        interface will make sure the speed_limit is never passed.
    :type speed_limit: float
    :param max_vel_of_full_push: Relevant when controlling the car with remote.
                                 It scales the signal of the remote so that the
                                 maximim signal of the remote is max_vel_of_full_push.
                                 Defaults to 4.
    :type max_vel_of_full_push: float
    """
    def __init__(self, vehicle_name='', speed_limit=0.5, max_vel_of_full_push=4):
        ControlInterface.__init__(self, vehicle_name)
        self.PERC_TO_LLI_COEFF = 1.27
        self.MAX_VELOCITY = 1
        self.max_vel_of_full_push = max_vel_of_full_push
        self.odometry_node = ReadSpeed(3).start()
        self.speed_limit = speed_limit

        # PID params
        self.k_p = 0.5
        self.k_i = 0.03
        self.k_d = 100 # 0.02
        self.k_w = 1
        self.integral = 0
        self.last_error = 0

        # remote velocity parameters
        self.velocity_max = speed_limit
        self.velocity_current = None
        self.velocity_remote = None
        self.velocity_ref = None
        self.velocity_limited = None
        # other remote signals
        self.remote_steering = None
        self.remote_transmission = -1
        self.remote_diff_front = -1  # for now the differentials are always at default
        self.remote_diff_rear = -1
        self.remote_ctrl = None

        # Parameter for emergency
        self.is_emergency = False

    def _start_publish(self):
        self.ctrl_request_pub = rospy.Publisher(self.vehicle_name+'/lli/ctrl_request',
                                                lli_ctrl,
                                                queue_size=1)
        self.averaged_velocity_pub = rospy.Publisher(self.vehicle_name+'/avg_velocity',
                                                     average_velocity,
                                                     queue_size=1)

    def _start_listen(self):
        rospy.Subscriber(self.vehicle_name+'/lli/ctrl_actuated',
                         lli_ctrl, self._read_ctrl_actuated)
        rospy.Subscriber(self.vehicle_name+'/lli/remote', lli_ctrl,
                         self._read_remote)
        rospy.Subscriber('/lli/remote', lli_ctrl, self._read_remote2)
        rospy.Subscriber('/isEmergency', Bool, self._read_is_emergency)
        rospy.Subscriber('/odometry/filtered', Odometry, self._read_velocity_current)
        rospy.loginfo("Constraints Interface successfully initialized")

    def _read_is_emergency(self, msg):
        """Reads to see if there is an emergency and the car needs to brake."""
        self.is_emergency = msg.data

    def _read_remote2(self, msg):
        """Reads the commands sent from the remote controller."""
        steering = msg.steering
        self.velocity_remote = msg.velocity
        velocity = msg.velocity
        trans_diff = msg.trans_diff
        self.remote_ctrl = msg.ctrl

        velocity = velocity / self.PERC_TO_LLI_COEFF
        velocity = self._percent_to_vel(velocity)
        steering = steering / self.PERC_TO_LLI_COEFF
        self.remote_steering = self._percent_to_steer(steering)

        if bin(trans_diff)[-1] == '1':
            self.remote_transmission = 1
        else:
            self.remote_transmission = 0

    def _read_position(self, msg):
        """Reads positional estimate from localization node"""
        self.position_log.append(msg)

    def pid_controller(self, error):
        self.integral = self.integral + error  # integral part
        difference = self.last_error - error   # differential part

        # PID 
        control_signal = self.k_p * error + self.k_i * self.integral + difference/self.k_d
        control_before_clipping = control_signal

        # clipping the input
        if control_signal > self.MAX_VELOCITY:
            control_signal = self.MAX_VELOCITY
        if control_signal < -self.MAX_VELOCITY:
            control_signal = -self.MAX_VELOCITY

        # updating integral with anti-windup
        self.integral = self.integral + self.k_w * (control_signal - control_before_clipping)
        self.last_error = error
        return control_signal

    def _percent_to_steer(self, steer_percent):
        steer_percent = -float(steer_percent) # force float-based computations
        if steer_percent >= 0:
            steer_PWM = steer_percent / 100 * 600 + 1500
            sqrt_term = math.sqrt(5.016**2 - 4 * 0.2384 * (1495-steer_PWM))
            steering = (5.016 - sqrt_term) / (2*0.2384)
        else:
            steer_PWM = steer_percent / 100 * 600 + 1500
            sqrt_term = math.sqrt(5.016**2 - 4 *-0.2384 * (1495-steer_PWM))
            steering = (5.016 - sqrt_term) / (2*-0.2384)
        return math.radians(steering)

    def _percent_to_vel(self, vel_percent):
        vel_percent = float(vel_percent)
        velocity = vel_percent*self.MAX_VELOCITY / 100
        return velocity

    def _read_velocity_current(self, odom_msg):
        self.velocity_current = np.sign(odom_msg.twist.twist.linear.x) * np.linalg.norm([odom_msg.twist.twist.linear.x,
                                                                                         odom_msg.twist.twist.linear.y])


    def _convert_signal_to_velocity(self, max_vel_of_full_push=4.0):

        self.velocity_ref = (float(self.velocity_remote) / 127) * max_vel_of_full_push

    def _limited_velocity(self):
        temp = 0
        if self.velocity_ref < -self.velocity_max:
            temp = -self.velocity_max
        elif self.velocity_ref > self.velocity_max:
            temp = self.velocity_max
        else:
            temp = self.velocity_ref
        return temp

    def control_error(self):
        """
        Used when controlling car with remote.
        Calculates the control error which will be the input for the PID.

        :return: difference between the reference velocity from the controller
                 and the current velocity of the car.
        :rtype: float
        """
        self._convert_signal_to_velocity(self.max_vel_of_full_push)
        limited_velocity = self._limited_velocity()
        v = self.odometry_node.get_velocity()

        # Publish for plotjuggler
        self.averaged_velocity_pub.publish(v)
        return limited_velocity - v

    def send_constrained_control(self,
                                 steering=float('nan'),
                                 brake_force=0,
                                 transmission=-1,
                                 differential_front=-1,
                                 differential_rear=-1):
        """
        Calculates the velocity signal to send based on the current speed limit.
        In this case the speed limit is used as a reference.

        :param steering:    Desired steering for the car. Defaults to nan.
        :type steering: float
        :param brake_force: Desired brake signal to send to the car.
                            In range 0-100. Default is 0.
        :type brake_force: int
        :param transmission: Transmission to send to the car. -1 is default.
                             0 is low gear. 1 is high gear.
        :type transmission: int
        :param differential_front: 0 means unlocked, 1 means locked, -1
                                   means keep the currently set lock
                                   state, defaults to -1
        :type differential_front: int
        :param differential_rear: 0 means unlocked, 1 means locked, -1
                                  means keep the currently set lock
                                  state, defaults to -1
        :type differential_rear: int
        """

        error = self.speed_limit - self.odometry_node.get_velocity()
        velocity = self.pid_controller(error)
        self.send_control(steering,
                          velocity,
                          brake_force,
                          transmission,
                          differential_front,
                          differential_rear)


class BrakeControl():
    """Class dealing with braking of the car. Initializes an instance of
    the class ControlInterface.
    
    :param vehicle_name: Name of vehicle being controlled; used to
                         create ROS topic names for subscription and
                         publications, defaults to empty string.
    :type vehicle_name: str"""
    def __init__(self, vehicle_name):
        self.steering_remote = None
        self.velocity_remote = None
        self.steering_tx2 = None
        self.velocity_tx2 = None
        self.is_emergency = None
        self.vehicle_name = vehicle_name
        self.current_velocity = None

        self.control_interface = ControlInterface(self.vehicle_name).start()
        while not self.control_interface.is_ready and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.Subscriber('lli/remote', lli_ctrl, self._read_remote)
        rospy.Subscriber(self.vehicle_name+'/lli/ctrl_request', lli_ctrl, self._read_tx2)
        rospy.Subscriber('isEmergency', Bool, self._read_emergency)
        rospy.Subscriber('odometry/filtered', Odometry, self._read_velocity)

        rospy.sleep(0.2)

    def _read_remote(self, ctrl_msg):
        self.steering_remote = ctrl_msg.steering/1.27
        self.velocity_remote = ctrl_msg.velocity

    def _read_tx2(self, ctrl_msg):
        self.steering_tx2 = ctrl_msg.steering/1.27
        self.velocity_tx2 = ctrl_msg.velocity

    def _read_emergency(self, emergency_msg):
        self.is_emergency = emergency_msg.data

    def _percent_to_steer(self, steer_percent):
        steer_percent = -float(steer_percent) # force float-based computations
        if steer_percent >= 0:
            steer_PWM = steer_percent / 100 * 600 + 1500
            sqrt_term = math.sqrt(5.016**2 - 4 * 0.2384 * (1495-steer_PWM))
            steering = (5.016 - sqrt_term) / (2*0.2384)
        else:
            steer_PWM = steer_percent / 100 * 600 + 1500
            sqrt_term = math.sqrt(5.016**2 - 4 * -0.2384 * (1495-steer_PWM))
            steering = (5.016 - sqrt_term) / (2*-0.2384)
        return math.radians(steering)

    def brake_car_remote(self):
        """
        Used when the car is controlled by remote.
        Brakes the car if there is an emergency.
        Sends brake signal to control interface if the variable is_emergency is true.
        """
        if self.is_emergency is not None:
            if self.is_emergency:
                if self.velocity_remote > 0.05:
                    steering = self._percent_to_steer(self.steering_remote)
                    self.control_interface.send_control(steering, 0, 100, 0)

                if self.velocity_tx2 > 0:
                    steering = self._percent_to_steer(self.steering_tx2)
                    self.control_interface.send_control(steering, 0, 100, 0)
                    
    def brake_car(self, steering = 0):
        """Sends full brake signal to the car.
        
        :param steering: steering to send to the car. Defaults to 0
        :type steering: float
        """
        self.control_interface.send_control(steering, 0, 100)                   

    def _read_velocity(self, odo_msg):
        self.current_velocity = odo_msg.twist.twist.linear.x
