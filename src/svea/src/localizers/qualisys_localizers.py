#!/usr/bin/env python

"""
ROS interface objects for localization with Qualisys odom. The launch
files to be used in conjunction with these objects are::

    qualisys.launch
    qualisys_odom.launch model_name:=<insert your model name>

It is recommended you include these launch files in your project
roslaunch
"""

import sys
import os
import numpy as np
from threading import Thread

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from math import sqrt

from models.bicycle_simple import SimpleBicycleState

__license__ = "MIT"
__maintainer__ = "Frank Jiang"
__email__ = "frankji@kth.se"
__status__ = "Development"


class State(object):
    """
    Class representing the state of a vehicle. State is purposefully
    unit-less to support different applications.

    :param x: x position, defaults to 0.0
    :type x: float
    :param y: y position, defaults to 0.0
    :type y: float
    :param yaw: yaw, defaults to 0.0
    :type yaw: float
    :param v: velocity, defaults to 0.0
    :type v: float
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class QualisysOdom():
    """
    Wrapper for taking Qualisys Odometry msgs and converting it directly
    into a State object that is convenient for cars. Units are
    [m, rad, s, m/s]

    :param qualisys_model_name: Name of model given in the Qualisys
                                software.
    :type qualisys_model_name: str
    """

    def __init__(self, qualisys_model_name):
        self.qualisys_model_name = qualisys_model_name
        # rospy.init_node(self.qualisys_model_name + '_qualisys_odom')

        self.state = State()
        self.last_time = None

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: QualisysOdom
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Qualisys Odometry Interface Node: \n"
                      + str(self))
        self.node_name = self.qualisys_model_name + '_qualisys_odom'
        self._collect_srvs()
        self._start_listen()

    def _collect_srvs(self):
        # rospy.wait_for_service('set_pose')
        # self.set_pose = rospy.ServiceProxy('set_pose', SetPose)
        pass

    def _start_listen(self):
        rospy.Subscriber(self.qualisys_model_name + '/odom', Odometry,
                         self._read_qualisys_odom_msg)
        rospy.loginfo("Qualisys Odometry Interface successfully initialized")
        rospy.spin()

    def _read_qualisys_odom_msg(self, msg):
        pose = msg.pose.pose.position
        vel = msg.twist.twist.linear

        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        self.state.x = pose.x
        self.state.y = pose.y
        self.state.yaw = yaw
        self.state.v = sqrt(vel.x**2 + vel.y**2)

        self.last_time = rospy.get_time()

    def __repr__(self):
        return ""
    def __str__(self):
        return ""

    # def set_pose(self, qualisys_model_name, pose_to_set):
        # try:
            # self.set_pose(qualisys_model_name, pose_to_set)
        # except rospy.ServiceException as exc:
            # print(self.node_name + ": Set Pose service failed: " + str(exc))

    def is_publishing(self):

        if self.last_time is not None:
            is_publishing = (rospy.get_time() - self.last_time) < 0.2
            return is_publishing

        return False

    def get_state_obj(self):
        """
        (will be updated to use properties)

        :return: Current state as a State object
        :rtype: State
        """
        return self.state

    def get_state(self):
        """
        (will be updated to use properties)

        :return: Current state as a list [x, y, yaw, v] [m, m, rad, m/s]
        :rtype: list
        """
        return [self.state.x, self.state.y, self.state.yaw, self.state.v]

    def get_state_np(self):
        """
        (will be updated to use properties)

        :return: Current state as numpy array [x, y, yaw, v]
                 [m, m, rad, m/s]
        :rtype: numpy.ndarray
        """
        return np.array(self.get_state())


class QualisysSimpleOdom(QualisysOdom):

    def __init__(self, qualisys_model_name):
        QualisysOdom.__init__(self, qualisys_model_name)
        self.state = SimpleBicycleState()
