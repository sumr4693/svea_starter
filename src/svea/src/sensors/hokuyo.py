#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from threading import Thread

class HokuyoLidar():
    """
    Interface handling the HokuyoLidar. Collects and stores the most recent
    scan and handles stopping and starting the HokuyoLidar. Implements a
    couple extra feature(s): emergency detection.

    :param emergency_dist: Distance threshold for simple detection of
                           scenarios that could be considered an
                           emergency, defaults to 0.2 [m]
    :type emergency_dist: float
    """

    def __init__(self, emergency_dist=0.2):
        self.ranges = []

        self.emergency_dist = emergency_dist
        self.is_emergency = False

        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.time_increment = None
        self.last_scan_time = None


    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: RPLidar
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Lidar Interface Node: \n" + str(self))
        self._collect_srvs()
        self._start_listen()

    def _start_listen(self):
        rospy.Subscriber('scan', LaserScan, self._read_scan)
        rospy.loginfo("Lidar Interface successfully initialized")
        rospy.spin()

    def _read_scan(self, scan_msg):
        self.ranges = scan_msg.ranges

        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.angle_increment = scan_msg.angle_increment

        self.time_increment = scan_msg.time_increment

        self.last_scan_time = scan_msg.scan_time

