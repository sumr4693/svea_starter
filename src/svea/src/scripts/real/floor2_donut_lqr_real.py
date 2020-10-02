#!/usr/bin/env python



"""
Node for driving in donuts or circle race (Task 3) with car using lqr controller.
"""

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import json

from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped
from nav_msgs.msg import Path

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

import pure_pursuit

bezier_path = os.path.join(dirname,
        '../../PythonRobotics/PathPlanning/BezierPath/')
sys.path.append(os.path.abspath(bezier_path))

import bezier_path as bp

util = os.path.join(dirname, '../util/')
sys.path.append(os.path.abspath(util))

from state_subscriber import *
from geofence import Geofence

dirname = os.path.dirname(__file__)
lqr_sc = os.path.join(dirname,
                      '../../PythonRobotics/PathTracking/lqr_steer_control/')
sys.path.append(os.path.abspath(lqr_sc))


import lqr_steer_control

dirname = os.path.dirname(__file__)
cubic_spline_planner = os.path.join(dirname, '../../PythonRobotics/PathPlanning/CubicSpline/')
sys.path.append(os.path.abspath(cubic_spline_planner))

import cubic_spline_planner as csp

__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"

## LQR params ##########################################################
lqr_steer_control.L = 0.324
lqr_steer_control.Q = np.array([[10, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 5, 0],
                                [0, 0, 0, 0]])
lqr_steer_control.R = np.array([[1]])
max_steer = math.radians(45.0)  # maximum steering angle[rad]
###############################################################################


def smooth_donut_path(c, r, phase=0):
    """
    Create circular path using cubic spline.

    :param c:  center point of circle.
    :type c: Point
    :param r:   radius of the circle
    :type r: float
    :param phase:   phase angle for start of the circular path. Defaults to 0
    :type phase: float

    :return:    cx      - list of x-coordinates
                cy      - list of y-coordinates
                cyaw    - list of yaw angles
                ck      - list of curvatures for each (x,y) point
    :rtype: cx, cy, cyaw, ck list     
    """
    xs = []
    ys = []

    theta = np.linspace(0, 2*math.pi, 10)

    for angle in theta:
        xs.append(r*math.cos(angle - phase) + c.x)
        ys.append(r*math.sin(angle - phase) + c.y)
    cx, cy, cyaw, ck, s = csp.calc_spline_course(xs, ys, ds=0.1)
    return cx, cy, cyaw, ck

def init():
    """
    Grabs parameters from launch file, parses them and returns them.
    :return: speed_limit        The desired target speed for the car.
    :rtype: float
    :return: path_radius        Radius for the circular path
    :rtype: float
    :return: geofence radius        Radius for the geofence.
    :rtype: float
    :return: gear                   The gear at which we want the car to drive. 0 or 1.
    :rtype: int
    :return: target_laps            How many laps the car is supposed to drive.
    :rtype: int
    :return: front_differentials    Front differentials for the car. Locked or unlocked.
    :rtype: int
    :return: rear_differentials     Rear differentials for the car. Locked or unlocked.
    :rtype: int
    """

    speed_limit_param = rospy.search_param('speed_limit')
    speed_limit = rospy.get_param(speed_limit_param)
    print('speed limit')
    print(speed_limit)
    speed_limit = float(speed_limit)

    path_radius_param = rospy.search_param('path_radius')
    path_radius = rospy.get_param(path_radius_param)
    path_radius = float(path_radius)
    
    geofence_radius = rospy.search_param('geofence_radius')
    geofence_radius = rospy.get_param(geofence_radius)
    geofence_radius = float(geofence_radius)

    gear_param = rospy.search_param('gear')
    gear = rospy.get_param(gear_param)
    gear = int(gear)


    target_laps_param = rospy.search_param('target_laps')
    target_laps = rospy.get_param(target_laps_param)
    target_laps = float(target_laps)

    front_differentials_param = rospy.search_param('front_differentials')
    front_differentials = rospy.get_param(front_differentials_param)
    front_differentials = int(front_differentials)

    rear_differentials_param = rospy.search_param('rear_differentials')
    rear_differentials = rospy.get_param(rear_differentials_param)
    rear_differentials = int(rear_differentials)

    return speed_limit, path_radius, geofence_radius, gear, target_laps, front_differentials, rear_differentials 


def main():
    """
    Initializes parameters for doing donuts.
    Starts instance of ConstraintsInterface and BrakeControl.
    Creates publishers for visualization.
    Starts instance of Geofence.
    Then waits for a pose estimate for the car and geofence/donut center point to be given from rviz.
    Smooth circular or donut path is created.
    Car will start to follow path after start signal is given through /initialpose topic from rviz.
    As soon as the car completes a lap, a new circular path with a shift in starting point is created.
    LQR is used for path tracking, PID is used for velocity control.
    Geofence, circular track path and car's tracking path are visualized in rviz.
    """    
    rospy.init_node('donuts_node')

    speed_limit, path_radius, geofence_radius, gear, target_laps, front_differentials, rear_differentials = init()

    # vehicle name
    vehicle_name = rospy.get_param(rospy.get_name() + '/vehicle_name')

    # initialize constraints interface
    cont_intf = ConstraintsInterface(vehicle_name,
                                     speed_limit).start()
    
    cont_intf.k_p = 0.3
    cont_intf.k_d = 50

    # starts instance of brake control interface.
    brake_controller = BrakeControl(vehicle_name)

    rospy.sleep(2)

    # start publishers
    car_poly_pub = rospy.Publisher("/3D_car", PolygonStamped, queue_size=1)
    pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)
    path_plan_pub = rospy.Publisher("/path_plan", Path, queue_size=1)
    past_path_pub = rospy.Publisher("/past_path", Path, queue_size=1)
    target_pub = rospy.Publisher("/target", PointStamped, queue_size=1)
    rospy.sleep(0.2)

    # start geofence
    geofence = Geofence().start()

    # starting car position subscriber
    car_position = StateSubscriber().start()
    rospy.sleep(1)
    rospy.loginfo('Waiting for initial pose')
    rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
    rospy.loginfo('Received initial pose')

    # define the geofence
    geofence.set_center_radius(geofence_radius)

    # create circular path
    cx, cy, cyaw, ck = smooth_donut_path(geofence.center, path_radius)
    publish_path(path_plan_pub, cx, cy)

    # publish the geofence to rviz
    geofence.define_marker()
    geofence.publish_marker()
    # wait for the initial position of the car
    rospy.loginfo('Waiting for start signal')
    rospy.wait_for_message('/clicked_point', PointStamped)
    rospy.loginfo('Received start signal')

    while not car_position.x:
        print('waiting for initial car position')

    # initialize lqr variables
    lastIndex = len(cx) - 1
    target_ind = lqr_steer_control.calc_nearest_index(car_position, cx, cy, cyaw)


    # donut loop
    x = []
    y = []
    yaw = []
    time = 0.0
    steering = 0.0
    e, e_th = 0.0, 0.0
    rate = rospy.Rate(30)
    laps = 0
    while laps < target_laps and not rospy.is_shutdown():
        cx, cy, cyaw, ck = smooth_donut_path(geofence.center, path_radius, laps*math.pi/3)
        lastIndex = len(cx) - 1
        target_ind = 0
        while lastIndex > target_ind and not rospy.is_shutdown():
            steering, target_ind, e, e_th = \
                lqr_steer_control.lqr_steering_control(car_position, cx, cy, cyaw, ck, e, e_th)
                
            if geofence.is_outside_geofence(car_position):
                brake_controller.brake_car(0)
                cont_intf.integral = 0  # reset integrator in PID
            else:
                #cont_intf.send_constrained_control(steering, 0, gear)
                error = speed_limit - car_position.v 
                velocity = cont_intf.pid_controller(error)
                cont_intf.send_control(steering, velocity, 0, gear, front_differentials, rear_differentials)

            # publish for rviz    
            x.append(car_position.x)
            y.append(car_position.y)
            yaw.append(car_position.yaw)
            publish_3Dcar(car_poly_pub, pose_pub,
                          car_position.x,
                          car_position.y,
                          car_position.yaw)
            publish_path(path_plan_pub, cx, cy)
            publish_path(past_path_pub, x, y, yaw)
            publish_target(target_pub, cx[target_ind], cy[target_ind])

            rate.sleep()
        laps = laps + 1
        rospy.loginfo('finished lap')

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished.")

if __name__ == '__main__':
    main()
