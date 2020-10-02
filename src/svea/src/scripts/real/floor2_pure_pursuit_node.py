#!/usr/bin/env python

"""
ROS node (along with obstacle_avoidance_node.py) which does task2, the obstacle avoidance.
It tracks the dynamic path, which is the combination of the pre-computed obstacle-free path,
and the newly computed obstacle avoidance path (given by obstacle_avoidance_node.py).
Steering is calculated using pure pursuit.
The velocity is controlled using ConstraintsInterface, an extension to
ControlInterface which also incldes speed control.
"""

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import json
from threading import Thread

from tf.transformations import euler_from_quaternion
from obstacle_avoidance import ObstacleAvoidance

# messages
from std_msgs.msg import Bool
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped, PoseWithCovarianceStamped, Point
from svea.msg import path_pure_pursuit
from nav_msgs.msg import Path, OccupancyGrid

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from models.bicycle import SimpleBicycleState
from simulators.sim_SVEA import SimSVEA
from simulators.viz_utils import plot_car, publish_3Dcar
from simulators.viz_utils import publish_path, publish_target
from controllers.control_interfaces import ControlInterface
from controllers.constraints_interface import ConstraintsInterface, BrakeControl



dirname = os.path.dirname(__file__)
pure_pursuit = os.path.join(dirname,
                            '../../PythonRobotics/PathTracking/pure_pursuit/')
sys.path.append(os.path.abspath(pure_pursuit))


############################################# Our work #############################################
import pure_pursuit

dubins_path_planning = os.path.join(dirname,
                                    '../../PythonRobotics/PathPlanning/DubinsPath/')
sys.path.append(os.path.abspath(dubins_path_planning))
import dubins_path_planning as dpp


smooth_path = os.path.join(dirname,
                                    '../util/')
sys.path.append(os.path.abspath(smooth_path))
from create_smooth_path import *

__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"

# SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA3"
###############################################################################


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
# Some global Functions
def init():
    """
    Initialization handles use with just python or in a launch file.

    :param speed_limit: The desired target speed of the car
    :type speed_limit: float
    :param cx: The x-coordinates of the track path
    :type cx: list
    :param cy: The y-coordinates of the track path
    :type cy: list
    """
    curvature_param = rospy.search_param('curvature')
    curvature = rospy.get_param(curvature_param)
    curvature = float(curvature)

    speed_limit_param = rospy.search_param('speed_limit')
    speed_limit = rospy.get_param(speed_limit_param)
    speed_limit = float(speed_limit)

    coords_param = rospy.search_param('coords_param')
    coords = rospy.get_param(coords_param)
    coords = json.loads(coords)

    # make the path smooth
    cx, cy, cyaw = create_smooth_path(coords, curvature)

    return speed_limit, cx, cy
###############################################################################


###############################################################################
########################### OUR CLASS #########################################
###############################################################################
class Purepursuit():
    """
    Class for doing pure pursuit on given path.
    """
    def __init__(self):
        self.car_position = Position()
        # PURE PURSUIT PARAMS
        self.pure_pursuit = pure_pursuit
        self.pure_pursuit.k = 0.1  # look forward gain
        self.pure_pursuit.Lfc = 0.2  # look-ahead distance
        self.pure_pursuit.L = 0.324  # [m] wheel base of vehicle

        self.speed_limit, self.cx, self.cy = init()
        self.cont_intf = ConstraintsInterface(vehicle_name).start()

        # MAP PARAMS
        self.obstacle_map = None
        self.resolution = None
        self.width = None
        self.height = None
        self.future_x = None
        self.future_y = None
        self.origin = Point()

        # EMERGENCY PARAMS
        self.emergency_flag = False

        while not self.cont_intf.is_ready and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.brake_controller = BrakeControl(vehicle_name)

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data.
        :return: itself
        :rtype: Purepursuit
        """        
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        """
        Initializes subscribers and publishers. 
        """        
        self.car_poly_pub = rospy.Publisher("/3D_car", PolygonStamped, queue_size=1)
        self.pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)
        self.path_plan_pub = rospy.Publisher("/path_plan", Path, queue_size=1)
        self.target_pub = rospy.Publisher("/target", PointStamped, queue_size=1)
        self.longer_pub = rospy.Publisher("/plan_longer", Bool, queue_size=1)
        rospy.Subscriber('/robot_pose', PoseStamped, self.update_pose, queue_size=1)
        # publishes the new cx cy from the pathplanner when obstacle is detected
        rospy.Subscriber("/path_floor_2", path_pure_pursuit, self._update_path, queue_size=1)
        rospy.Subscriber('/map_obstacle', OccupancyGrid, self._get_obsmap_cb, queue_size=1)
        rospy.sleep(3)
        ## Initial Pose
        print('Waiting for initial pose')
        rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        print('Received initial pose')
        while not self.car_position.x:
            print('waiting for initial car position')
        rospy.sleep(1)

        rospy.spin()

    def update_pose(self,msg):
        """ 
        Callback that updates pose of car. 

        :param msg: car's current pose
        :type msg: PoseStamped        
        """
        self.car_position.x = msg.pose.position.x
        self.car_position.y = msg.pose.position.y
        quaternions = (msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w)
        euler = euler_from_quaternion(quaternions)
        self.car_position.yaw = euler[2]

    def _update_path(self,msg):
        """
        Callback to update the new a-star path to avoid obstacle.

        :param msg: path details
        :type msg: path_pure_pursuit
        """
        self.cx = msg.cx
        self.cy = msg.cy
        self.lastIndex = len(self.cx) - 1

    def calc_xyindex(self, x, y):
        """ 
        Calculates indexes from x,y position.

        :param x: x-coordinate in map frame
        :type x: float
        :param y: y-coordinate in map frame
        :type y: float
        
        :return: x,y grid coordinates
        :rtype: tuple
        """
        grid_x = int(round((x - self.origin.x) / self.resolution))
        grid_y = int(round((y - self.origin.y) / self.resolution))
        return (grid_x, grid_y)

    def _get_obsmap_cb(self,msg):
        """ 
        Callback that updates the occupancy map.

        :param msg: Obstacle map data
        :type msg: OccupancyGrid
        """
        temp_map = np.array(msg.data)
        self.width = msg.info.width
        self.height = msg.info.height
        self.obstacle_map = np.array(temp_map).reshape(self.height, self.width)
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position

    def visualize_everything(self):
        """ 
        Publish car, path and target index for rviz. 
        """
        publish_3Dcar(self.car_poly_pub, self.pose_pub, self.car_position.x, self.car_position.y, self.car_position.yaw)
        publish_path(self.path_plan_pub, self.cx, self.cy)
        publish_target(self.target_pub, self.cx[self.target_ind], self.cy[self.target_ind])

    def run_pure_pursuit(self):
        """
        Pure pursuit tracks the dynamic path created for distant obstacles.
        If the car senses the obstacles too close, it brakes and reverses, 
        and retries the same path forward 3 times.
        If the result is the same even during the 3rd consecutive retry, 
        it starts to plan the new path taking longer time.
        Visualizes publishers in rviz.
        """
        self.car_position.v = self.cont_intf.odometry_node.get_velocity()
        # initialize pure pursuit variables
        self.lastIndex = len(self.cx) - 1
        self.target_ind = pure_pursuit.calc_target_index(self.car_position, self.cx, self.cy)
        # simualtion + animation loop
        steering = 0.0
        rate = rospy.Rate(30)
        tries = 0
        extra_dist = 0
        while self.lastIndex > self.target_ind and not rospy.is_shutdown():

            pure_pursuit.Lfc = 0.4
            # compute control input via pure pursuit
            self.car_position.v = self.cont_intf.odometry_node.get_velocity()
            self.target_ind = pure_pursuit.calc_target_index(self.car_position, self.cx, self.cy)
            if tries == 3 and self.future_path_path_check(self.cx,self.cy):
                continue
            steering, self.target_ind = self.pure_pursuit.pure_pursuit_control(self.car_position, self.cx, self.cy, self.target_ind)

            if self.brake_controller.is_emergency or self.emergency_flag:
                if not self.emergency_flag: # the first time we are in emergency
                    print("--------------------------EMERGENCY--------------------------")
                    self.brake_controller.brake_car(0)
                    car_x = self.car_position.x
                    car_y = self.car_position.y
                    self.emergency_flag = True
                    self.cont_intf.integral = 0
                    self.cont_intf.last_error = 0
                    tries = (tries + 1)%4  
                    if tries == 3: # the third time it drives in reverse extra 0.2 m  
                        extra_dist = 0.2
                        messa = Bool()
                        messa.data = True
                        self.longer_pub.publish(messa) # after two unsuccesful tries publishes message for the obst avoidance to plan longer
                # go reverse
                self.cont_intf.send_control(-steering*2/3, -0.5, 0,0)
                # if went backwards reset everything
                if np.hypot(car_x - self.car_position.x, car_y - self.car_position.y)>=0.3+extra_dist:
                    self.emergency_flag = False
                    self.cont_intf.integral = 0
                    self.cont_intf.last_error = 0
		    rospy.sleep(2)
            else:
                error = self.speed_limit - self.car_position.v
                velocity = self.cont_intf.pid_controller(error)
                self.cont_intf.send_control(steering, velocity, 0,0) #0 break force and low gear
            self.visualize_everything()
            rate.sleep()
        if not rospy.is_shutdown():
            rospy.loginfo("Trajectory finished.")
        self.brake_controller.brake_car(0)

    def future_path_path_check(self, cx, cy):
        """
        Checks if future path is valid (0.7m).

        :param cx: The x-coordinates list of the track path
        :type cx: list
        :param cy: The y-coordinates list of the track path
        :type cy: list

        :return: flag indicating valid (true) or invalid (false) future path
        :rtype: bool
        """
        # Index nearest to car
        dx = [ self.car_position.x - icx for icx in cx]
        dy = [ self.car_position.y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        index = d.index(min(d))  # index of point nearest to car

        # Index 0.7 m ahead
        d_temp = [abs(x-0.7) for x in d[index:]]
        t_ind = d_temp.index(min(d_temp)) + index
        
        # check all points of path
        for i in range(index, t_ind):
            if self.check_point(cx[i],cy[i]):
                return True
        return False

    def check_point(self, x, y):
        """ 
        Checks if the point is valid for the robot to be.
        :param x: x-coordinate of the track path
        :type x: float
        :param y: y-coordinate of the track path
        :type y: float

        :return: flag indicating valid (true) or invalid (false) point
        :rtype: bool
        """
        grid_x = round((x - self.origin.x) / self.resolution)
        grid_y = round((y - self.origin.y) / self.resolution)
        rradius = int(0.17/self.resolution)
        if (self.obstacle_map[grid_y-rradius:grid_y+rradius, grid_x-rradius:grid_x+rradius] == 1).any():
            return True
        return False
###############################################################################
########################## END OF OUR CLASS ###################################
###############################################################################

def main():
    """
    Starts the instance of Purepursuit.
    Waits for the initial pose estimate from rviz.
    Runs the pure pursuit method for tracking the dynamic path to avoid obstacle.
    """
    rospy.init_node('floor2_pure_pursuit_node')
    pp = Purepursuit().start()
    rospy.sleep(3)
    rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
    if pp.car_position.x:
        pp.run_pure_pursuit()


if __name__ == '__main__':
    main()