#!/usr/bin/env python

"""
Node for path tracking in simulation using lqr controller.
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

svea = os.path.join(dirname, '../')
sys.path.append(os.path.abspath(svea))


dirname = os.path.dirname(__file__)
lqr_sc = os.path.join(dirname,
                            '../../PythonRobotics/PathTracking/lqr_steer_control/')
sys.path.append(os.path.abspath(lqr_sc))


############################################# Our work #############################################
import lqr_steer_control

dirname = os.path.dirname(__file__)
lqr_ssc = os.path.join(dirname,
                            '../../PythonRobotics/PathTracking/lqr_speed_steer_control/')
sys.path.append(os.path.abspath(lqr_ssc))

import lqr_speed_steer_control

dirname = os.path.dirname(__file__)
rear_wheel_fb = os.path.join(dirname,
                            '../../PythonRobotics/PathTracking/rear_wheel_feedback/')
sys.path.append(os.path.abspath(rear_wheel_fb))

import rear_wheel_feedback

dubins_path_planning = os.path.join(dirname,
                                    '../../PythonRobotics/PathPlanning/DubinsPath/')
sys.path.append(os.path.abspath(dubins_path_planning))

import dubins_path_planning as dpp


__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"

# SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA3"
target_speed = 1.0  # [m/s]
dt = 0.01
max_steer = math.radians(45.0)  # maximum steering angle[rad]
###############################################################################

# INIT #######################################################################
init_pt = [0.0, 0.0, 0.0, 0.0]  # [x, y, yaw, v], units: [m, m, rad, m/s]
visualize_plot = True
###############################################################################


# PURE PURSUIT PARAMS ########################################################
#pure_pursuit.k = 0.1  # look forward gain
#pure_pursuit.Lfc = 0.1  # look-ahead distance
#pure_pursuit.L = 0.324  # [m] wheel base of vehicle
###############################################################################


def create_smooth_path(coords):
    """
    Takes list of list of coords where each entry is in the form of [x, y , yaw]
    returns extrapolated path using dubins path planning in three separate lists,
    such as x coordinates, y coordinates and yaw values, respectively.
    
    :return: cx     List of x coordinates
    :rtype: list
    :return: cy     List of y coordinates
    :rtype: list
    :return cyaw    List of yaw values
    :rtype: list
    :return ck      List of curvature values
    :rtype: list    
    """
    cx = []
    cy = []
    cyaw = []
    ck = []
    curvature = 0.4

    cx_init, cy_init, yaw_init, _, _ = dpp.dubins_path_planning(coords[0][0],
                                                                coords[0][1],
                                                                coords[0][2],
                                                                coords[1][0],
                                                                coords[1][1],
                                                                coords[1][2],
                                                                curvature)
    cx.extend(cx_init)
    cy.extend(cy_init)
    cyaw.extend(yaw_init)
    #print(len(cx_init),len(cy_init),len(yaw_init))
    curve_arr = [curvature] * len(cx_init)
    ck.extend(curve_arr)
    i = 1
    cx_init = cx_init[-1]
    cy_init = cy_init[-1]
    yaw_init = yaw_init[-1]
    while i < len(coords)-1:
        cx1, cy1, yaw1, _, _ = dpp.dubins_path_planning(cx_init,
                                                        cy_init,
                                                        yaw_init,
                                                        coords[i+1][0],
                                                        coords[i+1][1],
                                                        coords[i+1][2],
                                                        curvature)
        cx.extend(cx1)
        cy.extend(cy1)
        cyaw.extend(yaw1)
        #print(cx1,cy1,yaw1)
        curve_arr = [curvature] * len(cx1)
        ck.extend(curve_arr)
        cx_init = cx1[-1]
        cy_init = cy1[-1]
        yaw_init = yaw1[-1]
        i = i + 1

    return cx, cy, cyaw, ck

def init():
    """
    Initialization handles use with just python or in a launch file

    :return: start_pt   Starting point (x,y) where the car needs to start
    :rtype: list
    :return: use_rviz   Flag to enable/disable rviz visualization
    :rtype: bool
    :return: use_matplotlib     Flag to enable/disable pure pursuit animation
    :rtype: bool
    :return: cx     List of x coordinates
    :rtype: list
    :return: cy     List of y coordinates
    :rtype: list
    :return cyaw    List of yaw values
    :rtype: list
    :return ck      List of curvature values
    :rtype: list    
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')
    start_pt = rospy.get_param(start_pt_param, init_pt)
    if type(start_pt) == type(''):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]

    coords_param = rospy.search_param('coords_param')
    coords = rospy.get_param(coords_param)
    # print(coords_param)
    coords = json.loads(coords)
    # print(coords)
#    cx, cy = create_smooth_path(coords)
    cx, cy, cyaw, ck = create_smooth_path(coords)

    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, visualize_plot)

    return start_pt, use_rviz, use_matplotlib, cx, cy, cyaw, ck


def main():
    """
    Initializes the parameters from the launch file.
    Creates path using dubins planner.
    Starts the instances of SimpleBicycleState, ControlInterface and SimSVEA.
    Creates publishers for visualization.
    LQR is used for path tracking. PID is used for limiting velocity.
    """
    rospy.init_node('floor2_sim_lqr')
#    start_pt, use_rviz, use_matplotlib, cx, cy = init()
    start_pt, use_rviz, use_matplotlib, cx, cy, cyaw, ck = init()

    # initialize simulated model and control interface
    simple_bicycle_model = SimpleBicycleState(*start_pt, dt=dt)
    ctrl_interface = ControlInterface(vehicle_name).start()
    while not ctrl_interface.is_ready and not rospy.is_shutdown():
        rospy.sleep(0.1)

    # start background simulation thread
    simulator = SimSVEA(vehicle_name, simple_bicycle_model, dt)
    simulator.start()

    car_poly_pub = rospy.Publisher("/3D_car", PolygonStamped, queue_size=1)
    pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)
    path_plan_pub = rospy.Publisher("/path_plan", Path, queue_size=1)
    past_path_pub = rospy.Publisher("/past_path", Path, queue_size=1)
    target_pub = rospy.Publisher("/target", PointStamped, queue_size=1)
    rospy.sleep(0.2)

    # log data
    x = []
    y = []
    yaw = []
    v = []
    t = []

    # initialize pure pursuit variables
    lastIndex = len(cx) - 1
#    target_ind = pure_pursuit.calc_target_index(simple_bicycle_model, cx, cy)

#    target_ind, tmp = lqr_steer_control.calc_nearest_index(simple_bicycle_model, cx, cy, cyaw)

    target_ind, tmp = lqr_speed_steer_control.calc_nearest_index(simple_bicycle_model, cx, cy, cyaw)
    speed_profile = lqr_speed_steer_control.calc_speed_profile(cx, cy, cyaw, target_speed)

#    target_ind, tmp = rear_wheel_feedback.calc_nearest_index(simple_bicycle_model, cx, cy, cyaw)

    print("tar:", target_ind)
    # simualtion + animation loop
    time = 0.0
    steering = 0.0
    e, e_th = 0.0, 0.0
    while lastIndex > target_ind and not rospy.is_shutdown():

        # compute control input via pure pursuit
#        steering, target_ind = \
#            pure_pursuit.pure_pursuit_control(simple_bicycle_model, cx, cy, target_ind)

#        steering, target_ind, e, e_th = lqr_steer_control.lqr_steering_control(
#            simple_bicycle_model, cx, cy, cyaw, ck, e, e_th)

        steering, target_ind, e, e_th, ai = \
            lqr_speed_steer_control.lqr_steering_control(simple_bicycle_model, cx, cy, cyaw, ck, e, e_th, speed_profile)
#        if steering >= max_steer:
#            steering = max_steer
#        if steering <= - max_steer:
#            steering = - max_steer

#        steering, target_ind = \
#            rear_wheel_feedback.rear_wheel_feedback_control(simple_bicycle_model, cx, cy, cyaw, ck, target_ind)

        ctrl_interface.send_control(steering, target_speed)

        # log data; mostly used for visualization
        x.append(simple_bicycle_model.x)
        y.append(simple_bicycle_model.y)
        yaw.append(simple_bicycle_model.yaw)
        v.append(simple_bicycle_model.v)
        t.append(time)

        # update visualizations
        if use_rviz:
            publish_3Dcar(car_poly_pub, pose_pub,
                          simple_bicycle_model.x,
                          simple_bicycle_model.y,
                          simple_bicycle_model.yaw)
            publish_path(path_plan_pub, cx, cy)
            publish_path(past_path_pub, x, y, yaw)
            publish_target(target_pub, cx[target_ind], cy[target_ind])
        if use_matplotlib:
            to_plot = (simple_bicycle_model,
                       x, y,
                       steering, target_ind)
            animate_pure_pursuit(*to_plot)
        if not (use_rviz or use_matplotlib):
            rospy.loginfo_throttle(1.5, simple_bicycle_model)

        time += dt

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished.")

    if use_matplotlib:
        plt.close()
        to_plot = (simple_bicycle_model,
                   x, y,
                   steering, target_ind)
        animate_pure_pursuit(*to_plot)
        plt.show()
    rospy.spin()


def plot_trajectory(car_model, x, y):
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Heading[deg]: "
              + str(math.degrees(car_model.yaw))[:4]
              + " | Speed[m/s]:"
              + str(car_model.v)[:4])


def animate_pure_pursuit(car_model, x, y, steering, target_ind):
    """ Single animation update for pure pursuit."""
    plt.cla()  # clear last plot/frame
    plot_trajectory(car_model, x, y)
    # plot pure pursuit current target
    plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
    # plot car current state
    plot_car(car_model.x,
             car_model.y,
             car_model.yaw,
             steering)
    plt.pause(0.001)


if __name__ == '__main__':
    main()
