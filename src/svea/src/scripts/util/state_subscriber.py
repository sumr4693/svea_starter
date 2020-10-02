import sys
import os
import rospy
import math
import numpy as np
from threading import Thread
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class StateSubscriber:
    """
    Class which creates object that holds the state of the car.
    It subscribes to the topics /robot_pose and /odometry/filtered.
    """
    def __init__(self):
        self.x = None
        self.y = None
        self.yaw = None
        self.v = None

    def __str__(self):
        string = 'SVEA state : ' + 'x : ' + str(self.x) + ', y : ' + str(self.y) + ', yaw : ' + str(self.yaw)
        return string

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving data.
        :return: itself
        :rtype: StateSubscriber
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo('Starting Car State Subscriber Node')
        self.node_name = 'point_subscriber'
        self._start_listen()
        self.is_ready = True
        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber('/robot_pose', PoseStamped, self._update_pose, queue_size=1)
        rospy.Subscriber('/odometry/filtered', Odometry, self._read_velocity, queue_size=1)

    def _read_velocity(self, msg):
        self.v = np.sign(msg.twist.twist.linear.x) * np.linalg.norm([msg.twist.twist.linear.x,
                                                                     msg.twist.twist.linear.y])

    def _update_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        quaternions = (msg.pose.orientation.x,
                       msg.pose.orientation.y,
                       msg.pose.orientation.z,
                       msg.pose.orientation.w)
        euler = euler_from_quaternion(quaternions)
        self.yaw = euler[2]
