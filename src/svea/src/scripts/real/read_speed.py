#!/usr/bin/env python

"""
Speed module that reads velocity from odometry/filtered.
"""

from threading import Thread

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from math import pi, floor, ceil


class ReadSpeed():
    """Class that estimates the speed of the vehicle using its sensors.
    The speed estimate is created by averaging the absolute value
    of the linear velocity over multiple samples (defaults to 1). """
    def __init__(self, _velocity_averaging=1):
        self.node_name = "Default odometry handler"
        self.raw_data = Odometry()

        self._velocity_averaging = _velocity_averaging
        self._observed_velocities = np.zeros(_velocity_averaging)  # type: np.ndarray
        self._velocity_position = 0

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data
        :return: itself
        :rtype: ReadSpeed
        """        
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Odometry Interface Node: \n" + str(self))
        self._start_listen()

    def _start_listen(self):
        rospy.Subscriber('/odometry/filtered', Odometry, self._read_odometry)
        rospy.loginfo("Odometry reader successfully initialized")
        rospy.spin()

    def _read_odometry(self, data):
        """
        :type scan_msg: Odometry
        """
        self.raw_data = data
        self._observed_velocities[self._velocity_position] = np.sign(data.twist.twist.linear.x)*np.linalg.norm([data.twist.twist.linear.x, data.twist.twist.linear.y])
        self._velocity_position = (self._velocity_position + 1) % self._velocity_averaging

    def get_velocity(self):
        """
        Returns the mean of the velocity norm
        """
        return self._observed_velocities.mean()

    def _build_param_printout(self):
        return "PARAMS OF ODOMETRY SHOULD BE HERE!!!"
        # param_str = "{0}:\n".format(self.node_name)

        # for param_name in self.IMPORTANT_ROS_PARAMS:
        #     curr_param = rospy.get_param(self.ROS_PARAM_PREFIX+ '/' + param_name)
        #     param_str += "  - {0}: {1}\n".format(param_name, curr_param)

        # return param_str

    def __repr__(self):
        return self._build_param_printout()
    def __str__(self):
        return self._build_param_printout()
