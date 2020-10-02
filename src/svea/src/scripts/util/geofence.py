#!/usr/bin/env python


"""
Module to create geofence for the donut task.
"""


import sys
import os
import rospy
import math
import numpy as np
from threading import Thread
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from controllers.control_interfaces import ControlInterface
from controllers.constraints_interface import ConstraintsInterface, BrakeControl

smooth_path = os.path.join(dirname,
                                    '../util/')
sys.path.append(os.path.abspath(smooth_path))

from create_smooth_path import *
from state_subscriber import *

class Point:
    """
    Point class which creates object to store x and y coordinate.
    """
    def __init__(self):
        self.x = None
        self.y = None
    def __str__(self):
        return 'x:' + str(self.x) + ', y:' + str(self.y)

class Geofence:
    """Geofence interface. Object holds center point and radius of the geofence."""
    def __init__(self):
        self.point = Point()
        self.center = Point()
        self.radius = None

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
                'Starting Geofence Node')
        self.node_name = 'point_subscriber'
        self._start_listen()
        self._start_publish()
        self.is_ready = True
        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber('/clicked_point', PointStamped, self._read_point, queue_size=1)

    def _start_publish(self):
        self.geofence_circle_pub = rospy.Publisher('/visualization_marker',
                                                   Marker,
                                                   queue_size=1)

    def _read_point(self, msg):
        self.point.x = msg.point.x
        self.point.y = msg.point.y
    
    def set_center(self):
        """
        Waits for a point to be published to topic /clicked_point.
        Sets center of geofence to those coordinates.
        """
        rospy.wait_for_message('/clicked_point', PointStamped)
        self.center.x = self.point.x
        self.center.y = self.point.y
        rospy.loginfo('Center point set to : ' + str(self.center))
    def set_center_radius(self, radius):
        """
        Waits for a point to be published to topic /clicked_point.
        Sets center of geofence to those coordinates and self.radius to radius.

        :param radius: radius of the geofence.
        :type radius: float
        """
        rospy.wait_for_message('/clicked_point', PointStamped)
        self.center.x = self.point.x
        self.center.y = self.point.y
        self.radius = radius
        rospy.loginfo('Center point set to : ' + str(self.center))
        rospy.loginfo('Radius set to : ' + str(self.radius))  
    def set_radius(self):
        """
        Waits for a point to be published to topic /clicked_point.
        Sets radius of geofence to those coordinates.
        """
        rospy.wait_for_message('/clicked_point', PointStamped)
        self.radius = np.linalg.norm([self.center.x-self.point.x,
                                      self.center.y-self.point.y], 2)
        rospy.loginfo('Radius set to : ' + str(self.radius))

    def define_marker(self, color=[0, 1, 0], height=1, alpha=0.2):
        """
        Creates a cylindrical visulization marker with position defined by
        self.center and radius defined by self.radius.

        :param color:   RBG values for the color of the cylinder.
                        values are given in range 0 to 1. Defaults to [0, 1, 0] 
        :type color: list
        :param height:  Height of the cylinder in meters. Defaults to 1.
        :type height: float
        :param alpha:   Opacity of the cylinder. In range 0.0 - 1.0. Default is 0.2.
        :type alpha: float
        """
        self.marker = Marker()
        self.marker.type = self.marker.CYLINDER
        self.marker.action = self.marker.ADD
        self.marker.header.frame_id = '/map'
        self.marker.pose.position.x = self.center.x
        self.marker.pose.position.y = self.center.y
        self.marker.pose.position.z = 0
        self.marker.scale.x = 2*self.radius
        self.marker.scale.y = 2*self.radius
        self.marker.scale.z = height
        self.marker.color.a = alpha
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.pose.orientation.w = 1.0

    def publish_marker(self):
        """Publish the marker to the /visualization_marker topic. """
        self.geofence_circle_pub.publish(self.marker)
        rospy.loginfo('Publishing geofence marker')
        
    def is_outside_geofence(self, car_position):
        """
        Finds out if the car is ourside the geofence.

        :param car_position: object which has the state of the car.
        :type car_position: StateSubscriber

        :return: True if the car is outside the geofence. False otherwise
        :rtype: Bool
        """
        norm = np.linalg.norm([car_position.x - self.center.x,
                               car_position.y - self.center.y], 2)
        if norm > self.radius:
            return True
        else:
            return False


def main():
    rospy.init_node('geofence_node')

    # start instance of brake_controller
    brake_controller = BrakeControl('geofence')

    # publishers
    car_poly_pub = rospy.Publisher("/3D_car", PolygonStamped, queue_size=1)
    pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)
    path_plan_pub = rospy.Publisher("/path_plan", Path, queue_size=1)
    past_path_pub = rospy.Publisher("/past_path", Path, queue_size=1)
    target_pub = rospy.Publisher("/target", PointStamped, queue_size=1)

    # starting car position subscriber
    car_position = StateSubscriber().start()

    # starting geofence node
    geofence = Geofence().start()
    rospy.sleep(1)

    # define the geofence
    geofence.set_center()
    geofence.set_radius()

    # publish the geofence to rviz
    geofence.define_marker()
    geofence.publish_marker()

    # control loop
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        norm = np.linalg.norm([car_position.x - geofence.center.x,
                               car_position.y - geofence.center.y], 2)
        if norm > geofence.radius:
            brake_controller.brake_car()
    rate.sleep()

if __name__ == '__main__':
    main()
