#!/usr/bin/env python

"""
ROS node which does task 1, the track race.
Given waypoints, the program generates a path, using dubins path planning.
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
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
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

dirname = os.path.dirname(__file__)
dubins_path_planning = os.path.join(dirname,
                                    '../../PythonRobotics/PathPlanning/DubinsPath/')
sys.path.append(os.path.abspath(dubins_path_planning))
import dubins_path_planning as dpp

dirname = os.path.dirname(__file__)
smooth_path = os.path.join(dirname, '../util/')
sys.path.append(os.path.abspath(smooth_path))

from create_smooth_path import *
from state_subscriber import *

__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"


def init():
    """
    Grabs parameters from launch file.
    Creates path from waypoint parameters.
    Returns the generated path and the rest of the parameters
    
    :return: target_speed       The desired target speed for the car.
    :rtype: float
    :return: cx                 The x-coordinates of the track path.
    :rtype: list
    :return: cy                 The y-coordinates of the track path.
    :rtype: list
    :return: gear               The gear at which we want the car to drive. 0 or 1.
    :rtype: int
    :return: long_look_ahead    The default look ahead distance for pure pursuit.
    :rtype: float
    :return: short_look_ahead   The look ahead used around the table. 
    :rtype: float
    """
    curvature_param = rospy.search_param('curvature')
    curvature = rospy.get_param(curvature_param)
    curvature = float(curvature)

    speed_limit_param = rospy.search_param('speed_limit')
    target_speed = rospy.get_param(speed_limit_param)
    target_speed = float(target_speed)
    
    long_look_ahead = rospy.search_param('long_look_ahead')
    long_look_ahead = rospy.get_param(long_look_ahead)
    long_look_ahead = float(long_look_ahead)
     
    short_look_ahead = rospy.search_param('short_look_ahead')
    short_look_ahead = rospy.get_param(short_look_ahead)
    short_look_ahead = float(short_look_ahead)
         
    gear_param = rospy.search_param('gear')
    gear = rospy.get_param(gear_param)
    gear = int(gear)

    coords_param = rospy.search_param('coords_param')
    coords = rospy.get_param(coords_param)
    coords = json.loads(coords)

    # make the path
    cx, cy, _ = create_smooth_path(coords, curvature)

    return target_speed, cx, cy, gear, long_look_ahead, short_look_ahead

def calc_look_ahead(car_state):
    """
    Calculates which look ahead gain to use for pure pursuit.
    The look ahead is shorter when going around the table.

    :param car_state:    Object holding the state of the car (x, y, yaw, v)
    :type car_state: StateSubscriber

    :return: desired look ahead based on where the car is.
    :rtype: float
    """
    # points for look ahead around table
    p1 = (-1.287, -1.69)
    p2 = (1.35, 1.63)
    table_line = (p2[0]-p1[0],p2[1]-p1[1])

    # projection of car onto table_line
    d_car_table = np.dot((car_state.x,car_state.y), table_line)
    d_top_table = np.dot(p2, table_line)
    d_bottom_table = np.dot(p1, table_line)

    if d_car_table > d_top_table or d_car_table < d_bottom_table:
        return long_look_ahead 
    else:
        return short_look_ahead


def main():
    """
    Initializes parameters for pure pursuit.
    Starts instance of ConstraintsInterface and BrakeControl.
    Starts instance of StateSubscriber.
    Creates publishers for visualization.
    Then waits for a pose estimate for the car to be given.
    Car will start to follow path after start signal is given,
    which is done by publishing to the /clicked_point topic.
    """
    global short_look_ahead, long_look_ahead
    rospy.init_node('floor2_pure_pursuit')
    vehicle_name = "SVEA3"

    target_speed, cx, cy, gear, long_look_ahead, short_look_ahead = init()
    
    final_point = (-8.45582103729,-5.04866838455)
    cx.append(final_point[0])
    cy.append(final_point[1])

    cont_intf = ConstraintsInterface(vehicle_name, target_speed).start()
    cont_intf.k_d = 100

    while not cont_intf.is_ready and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    # start emergency brake node
    brake_controller = BrakeControl(vehicle_name)

    # start car position and velocity subscriber
    car_state = StateSubscriber().start()

    car_poly_pub = rospy.Publisher("/3D_car", PolygonStamped, queue_size=1)
    pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)
    path_plan_pub = rospy.Publisher("/path_plan", Path, queue_size=1)
    past_path_pub = rospy.Publisher("/past_path", Path, queue_size=1)
    target_pub = rospy.Publisher("/target", PointStamped, queue_size=1)
    rospy.sleep(3)
    
    print('Waiting for initial pose')
    rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
    print('Received initial pose')
    publish_path(path_plan_pub, cx, cy)
    print('Waiting for start signal')
    rospy.wait_for_message('/clicked_point', PointStamped)
    print('Received start signal')
    
    while not car_state.x:
        print('waiting for initial car position')

    # PURE PURSUIT PARAMS ########################################################
    pure_pursuit.k = 0.1  # look forward gain
    pure_pursuit.Lfc = long_look_ahead  # look-ahead distance
    pure_pursuit.L = 0.324  # [m] wheel base of vehicle
    ###############################################################################

    lastIndex = len(cx) - 2
    target_ind = pure_pursuit.calc_target_index(car_state, cx, cy)

    x = []
    y = []
    yaw = []

    # pure pursuit loop
    # follow path until second to last index.
    steering = 0.0
    rate = rospy.Rate(30)
    cont_intf.send_control(steering, 0, 0, gear)  # send low gear
    
    while lastIndex > target_ind and not rospy.is_shutdown():

        pure_pursuit.Lfc = calc_look_ahead(car_state)
        
        # compute control input via pure pursuit
        steering, target_ind = \
            pure_pursuit.pure_pursuit_control(car_state, cx, cy, target_ind)
        
        if not brake_controller.is_emergency:
            cont_intf.send_constrained_control(steering,
                                               0,   # zero brake force
                                               gear)  # send low gear
        else:
            brake_controller.brake_car(steering)
        
        # publish car, path and target index for rviz
        x.append(car_state.x)
        y.append(car_state.y)
        yaw.append(car_state.yaw)
        publish_3Dcar(car_poly_pub, pose_pub,
                      car_state.x,
                      car_state.y,
                      car_state.yaw)  
        publish_path(path_plan_pub, cx, cy)
        publish_path(past_path_pub, x, y, yaw)            
        publish_target(target_pub, cx[target_ind], cy[target_ind])
        rate.sleep()


    # create a straight path for points inbetween the last two points

    # points to project goal onto
    p1 = (-2.33859300613, 3.73132610321)
    p2 = (-2.86509466171, 3.14682602882)
    line = (p2[0]-p1[0], p2[1]-p1[1])
    proj_p2 = np.dot(p2,line)
    cx = cx[:-1]
    cy = cy[:-1]
    new_x = np.linspace(cx[-1], final_point[0], 50)
    new_y = np.linspace(cy[-1], final_point[1], 50)
    cx.extend(new_x)
    cy.extend(new_y)

    # continue following final point until goal is reached.
    while not rospy.is_shutdown():
        steering, target_ind = \
            pure_pursuit.pure_pursuit_control(car_state, cx, cy, target_ind)

        # calculate car projection on line
        proj_car = np.dot((car_state.x, car_state.y), line)
        if brake_controller.is_emergency:
            brake_controller.brake_car(steering)
        elif proj_car < proj_p2:
            cont_intf.send_constrained_control(steering, 0, gear)
        else:
            # brake if goal is reached.
            brake_controller.brake_car(steering)
            rospy.sleep(0.1)
            break

        # update visualizations
        x.append(car_state.x)
        y.append(car_state.y)
        yaw.append(car_state.yaw)
        publish_3Dcar(car_poly_pub, pose_pub,
                      car_state.x,
                      car_state.y,
                      car_state.yaw)
        publish_path(path_plan_pub, cx, cy)
        publish_path(past_path_pub, x, y, yaw)
        publish_target(target_pub, cx[target_ind], cy[target_ind])

        rate.sleep()
    
        
    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished.")
    
    while brake_controller.is_emergency:  
        brake_controller.brake_car(steering)

    rospy.spin()

if __name__ == '__main__':
    main()


