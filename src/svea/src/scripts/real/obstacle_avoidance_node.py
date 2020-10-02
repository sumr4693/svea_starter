#!/usr/bin/env python

"""
ROS node (along with floor2_pure_pursuit_node.py) which does task2, the obstacle avoidance.
It detects the obstacle, replans the path accordingly and publishes it to floor2_pure_pursuit_node.py.
"""

import os
import sys
import rospy
import numpy as np
import json
np.set_printoptions(threshold=sys.maxsize)
from math import cos, sin, atan2, fabs, sqrt
import matplotlib.pyplot as plt
from threading import Thread
from visualization_msgs.msg import Marker, MarkerArray
import tf

# messages
from svea.msg import path_pure_pursuit
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool, Float32MultiArray, Float32
from sensor_msgs.msg import LaserScan
from svea_arduino.msg import lli_ctrl
from nav_msgs.msg import Odometry

from getMapInfo import GetMapInfo
dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from controllers.control_interfaces import ControlInterface
from sensors.lidars import RPLidar
from sensors.hokuyo import HokuyoLidar
from read_speed import ReadSpeed
from simulators.viz_utils import publish_path, publish_target

smooth_path = os.path.join(dirname,
                                    '../util/')
sys.path.append(os.path.abspath(smooth_path))

from create_smooth_path import *

dirname = os.path.dirname(__file__)
potential_field = os.path.join(dirname,
                            '../../PythonRobotics/PathPlanning/PotentialFieldPlanning/')
sys.path.append(os.path.abspath(potential_field))

import potential_field_planning as pf

dirname = os.path.dirname(__file__)
a_star_file = os.path.join(dirname,
                            '../../PythonRobotics/PathPlanning/AStar/')
sys.path.append(os.path.abspath(a_star_file))

from a_star_kazushi import AStarPlanner 

__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"

def get_yaw(q):
    """
    Returns the Euler yaw from a quaternion.
    :param q: yaw in quaternion form
    :type q: list
    
    :return: yaw in euler form
    :rtype: float
    """
    return atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                    1 - 2 * (q[1] * q[1] + q[2] * q[2]))

class Position:
    """
    Class to get the position, yaw and velocity of the car.
    """
    def __init__(self):
        self.x = None
        self.y = None
        self.yaw = None
        self.v = None

    def __str__(self):
        string = 'SVEA data : ' + 'x : ' + str(self.x)+', y : '+str(self.y)+', yaw : '+str(self.yaw)
        return string

###############################################################################
########################### OUR CLASS #########################################
###############################################################################
class ObstacleAvoidance():
    """
    Class for doing obstacle detection and avoidance. 
    
    :param cone_width: lidar scan angle width only whose data should be considered
    :type cone_width: int
    :param subsampling_rate: rate indicating how often data to be sampled in a given array or list
    :type subsampling_rate: int
    :param robot_radius: radius of the car from its center.
    :type robot_radius: float
    """

    def __init__(self, cone_width=20, subsampling_rate=1, robot_radius=0.17):

        # Frame transformation
        self.tfBuffer = None
        self.rate = rospy.Rate(4)
        
        # Laser measurement data
        self.ranges = []
        self.angles = []
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.subsampling_rate = subsampling_rate
        
        # Obstacle detection data
        self.cone_width = cone_width*np.pi/180
        self.obstacle_detected = False      # flag is true if obstacle detected
        self.observed_obstacle_points = []
        self.narrowed_obstacle_points = []
        
        # Map
        self.map_service = GetMapInfo().start()
        rospy.sleep(5)
        
        # Planner
        self.a_star = AStarPlanner(self.map_service.map.shape, self.map_service.resolution, robot_radius, self.map_service.origin.x, self.map_service.origin.y)
        self.plan_longer = False
        
        
        # Car data
        self.steering = 0
        self.vehicle_name = "SVEA3"
        self.car_position = Position()

        # Path    
        xs = [-1.7, -6.95, -2.7, 11.97, 7.5 ]
        ys = [3.47, -3.31, -7.12, 10.83, 14.25]     
        self.cx = np.linspace(xs[0], xs[1],150).tolist() + np.linspace(xs[1], xs[2],100).tolist() \
                    + np.linspace(xs[2], xs[3],400).tolist() + np.linspace(xs[3], xs[4],100).tolist() \
                    + np.linspace(xs[4], xs[0],400).tolist()
        
        self.cy = np.linspace(ys[0], ys[1],150).tolist() + np.linspace(ys[1], ys[2],100).tolist() \
                    + np.linspace(ys[2], ys[3],400).tolist() + np.linspace(ys[3], ys[4],100).tolist() \
                    + np.linspace(ys[4], ys[0],400).tolist()

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data
        :return: itself
        :rtype: ObstacleAvoidance
        """        
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        """
        Initializes subscribers and publishers 
        """
        rospy.Subscriber('scan', LaserScan, self._read_scan)
        rospy.Subscriber('/robot_pose', PoseStamped, self._update_car_position, queue_size=1)
        rospy.Subscriber("/plan_longer", Bool, self._update_plann_longer, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/visualization_obstacles', MarkerArray, queue_size=10)
        self.target_pub = rospy.Publisher("/obstacle_target", Marker, queue_size=10)
        self.path_pub = rospy.Publisher("/path_plan", Path, queue_size=1)
        self.path_update = rospy.Publisher("/path_floor_2", path_pure_pursuit, queue_size=1)
        rospy.sleep(3)
        self.tf = tf.TransformListener()

        rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        while not self.car_position.x:
            pass
        rospy.sleep(1)

        rospy.spin()
        
    def _init_marker(self, x, y, i, color=[0,1,0]):
        """
        Creates a cylindrical visulization marker with position defined by
        self.center and radius defined by self.radius.

        :param x: x-coordinate 
        :type x: float
        :param y: y-coordinate
        :type y: float
        :param i: index of x- and y-coordinates in their respective lists
        :type i: int
        :param color: RBG for the color of the cylinder. Each element is in range 0.0-1.0.
        :type color: list

        :return: marker     cylindrical marker for visualization 
        :rtype: Marker
        """
        height=1
        alpha=0.8
        marker = Marker()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.header.frame_id = '/map'
        marker.pose.position.z = 0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = height
        marker.color.a = alpha
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.id = i
        
        return marker
        
    def pub_marker(self, xlist, ylist):
        """
        Vizualizes list of coordinates. Publishing to visualization_obstacles.
        
        :param xlist: list of x-coordinates
        :type xlist: list
        :param ylist: list of y-coordinates
        :type ylist: list
        """
        markerlist = MarkerArray()
        for i in range(len(xlist)):
            markerlist.markers.append(self._init_marker(xlist[i], ylist[i], i))
        self.obstacle_pub.publish(markerlist)
        rospy.loginfo('Publishing obstacle marker')
        
    def _read_scan(self, data):
        """ 
        Callback for reading lidar scans. 

        :param data: laser scan data containing ranges and angles
        :type data: LaserScan
        """
        self.ranges = np.array(data.ranges)
        self.angles = np.linspace(data.angle_min, data.angle_max, len(data.ranges))
        
    def _update_car_position(self, msg):
        """ 
        Callback for updating car position.

        :param msg: car's current pose
        :type msg: PoseStamped
        """
        self.car_position.x = msg.pose.position.x
        self.car_position.y = msg.pose.position.y
        
    def _update_plann_longer(self, msg):
        """ 
        Callback for cases when planning longer is needed.

        :param msg: flag to enable or disable the invoking of a-star long plan
        :type msg: bool
        """
        self.plan_longer = msg.data

        
    def detect_obstacle(self):
        """ 
        Filters the obstacle data from lidar scan and transforms to map frame.
        Updates map with the obstacle data.
        Initiates replan for avoiding obstacles.
        """
        angles = self.angles
        ranges = self.ranges
        # filter out lidar meassurements with angles bigger than pi/2
        indexes = np.logical_and(np.logical_and(angles<np.pi/2 ,angles > -np.pi/2),ranges<5)
        ranges = ranges[indexes]   
        angles = angles[indexes]
        (trans,rot) = self.tf.lookupTransform('map', 'laser', rospy.Time(0))
        laser_yaw = get_yaw(rot)
        map_rot_laser = np.array( [cos(laser_yaw), -sin(laser_yaw), 
                                   sin(laser_yaw), cos(laser_yaw)] ).reshape(2,2)
        x = ranges * np.cos(np.array(angles))
        y = ranges * np.sin(np.array(angles))
        pose_from_laser = np.vstack((x,y))            
        pose_from_map = np.matmul( map_rot_laser, pose_from_laser) + np.array([trans[0], trans[1]]).reshape((2,1))
   

        self.observed_obstacle_points = pose_from_map[:,40:-40:self.subsampling_rate]
        self.map_service.update_map(self.observed_obstacle_points)  
        self.observed_obstacle_points = self.observed_obstacle_points.tolist()
        self.new_path()
    
        
        
    def new_path(self):
        """ 
        Calculates obstacle avoiding path and publishes it for the pure pursuit node.
        For the distant obstacles, the path planning is done taking shorter time.
        For the near obstacles, the path planning is done taking longer time.
        Publishes the obstacle markers for visualization in rviz.

        :return: returning null during exception and empty path
        :rtype: null
        """
        try:
            #1)Find index of where we are
            dx = [ self.car_position.x - icx for icx in self.cx]
            dy = [ self.car_position.y - icy for icy in self.cy]
            d = [abs(sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
            index = d.index(min(d))  # index of point nearest to car
            
            #2)Find start index and point
            d_temp = [abs(x) for x in d[index:index+500]]
            s_ind = d_temp.index(min(d_temp)) + index
            start_x = self.cx[s_ind]
            start_y = self.cy[s_ind]
            
            # Help index to avoid planning from start to end through the big corridor
            temp_length = len(self.cx)
            end_ind =-1
            if s_ind < temp_length/2:
                end_ind = int(2*s_ind)
            
            #3)Find target index and point
            dx = [ start_x - icx for icx in self.cx[s_ind:end_ind]]
            dy = [ start_y - icy for icy in self.cy[s_ind:end_ind]]
            d_temp = [abs(sqrt(idx ** 2 + idy ** 2)-4) for (idx, idy) in zip(dx, dy)]
            t_ind = d_temp.index(min(d_temp)) + s_ind
            target_x = self.cx[t_ind]
            target_y = self.cy[t_ind]
        except:
            print("Index error")
            return
        # Publish start and target for the replanner
        self.target_pub.publish(self._init_marker(target_x, target_y, 1,[0,0,1] ))
        self.target_pub.publish(self._init_marker(start_x, start_y, 4, [1,1,0]))

        print("Planning running")
        try:
            if self.plan_longer:
                # Plan longer with timeout 11
                self.plan_longer = False
                x,y = self.a_star.planning_longer(start_x, start_y, target_x, target_y, self.map_service.get_map(),inflate=True)
            else:
                # Plann with timout 2
                x,y = self.a_star.planning(start_x, start_y, target_x, target_y, self.map_service.get_map(),inflate=True )
            if len(x) <25:
                return
        except:
            print("Couldnt find any path")
            return
    
        if len(x)>0:
            # New Path
            cx_new = self.cx[0:index]
            cx_new.extend(x)
            cx_new.extend(self.cx[t_ind:])
            cy_new = self.cy[0:index]
            cy_new.extend(y)
            cy_new.extend(self.cy[t_ind:])
            self.cx = cx_new
            self.cy = cy_new
            self.pub_marker(self.cx,self.cy)
            
            # Create message for new path and publish
            new_path = path_pure_pursuit()
            new_path.cx = cx_new
            new_path.cy = cy_new
            publish_path(self.path_pub, cx_new, cy_new)
            self.path_update.publish(new_path)
            
        else:
            rospy.loginfo("Got an empty path and moving on.")
            return     
###############################################################################
########################## END OF OUR CLASS ###################################
###############################################################################


def main():
    """
    Starts the instance of ObstacleAvoidance.
    Waits for the initial pose estimate from rviz.
    Detects the obstacles, replans accordingly and publish the new map with obstacles to rviz.
    """
    rospy.init_node('obstacle_avoidance_node')
    pp = ObstacleAvoidance().start()
    rospy.sleep(5)
    rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
    while not rospy.is_shutdown():
        if len(pp.ranges) > 0:
            pp.detect_obstacle()
            pp.rate.sleep()
            pp.map_service.publish_map2_data()
    print("I shut down")

if __name__ == '__main__':
    main()