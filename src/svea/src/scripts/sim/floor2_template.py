#!/usr/bin/env python

"""
Node for path tracking in simulation using pure pursuit controller.
"""

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import json
from tf.transformations import euler_from_quaternion
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
from controllers.constraints_interface import ConstraintsInterface

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

smooth = os.path.join(dirname,
                                    '../../scripts/util/')
sys.path.append(os.path.abspath(smooth))

from create_smooth_path import *

__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"

# SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA3"
target_speed = 1.0  # [m/s]
dt = 0.01
###############################################################################

# INIT #######################################################################
init_pt = [0.0, 0.0, 0.0, 0.0]  # [x, y, yaw, v], units: [m, m, rad, m/s]
visualize_plot = True
###############################################################################


# PURE PURSUIT PARAMS ########################################################
pure_pursuit.k = 0.1  # look forward gain
pure_pursuit.Lfc = 0.1  # look-ahead distance
pure_pursuit.L = 0.324  # [m] wheel base of vehicle
###############################################################################


class Position:
    """
    Class to get car's position and heading
    """
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0


position = Position()

def update_pose(msg):
    """
    Callback function to update the car's position

    :param msg: Contains the time stamped pose information of car
    :type msg: PoseStamped
    """
    global position
    position.x = msg.pose.position.x
    position.y = msg.pose.position.y
    # print(msg.pose.orientation)
    quaternions = (msg.pose.orientation.x,
                   msg.pose.orientation.y,
                   msg.pose.orientation.z,
                   msg.pose.orientation.w)
    euler = euler_from_quaternion(quaternions)
    position.yaw = euler[2]
    # print(euler)


rospy.Subscriber('/robot_pose', PoseStamped, update_pose)


def init():
    """
    Initialization handles use with just python or in a launch file
    
    :return: start_pt   Starting point (x,y) where the car needs to start
    :rtype: list
    :return: speed_limit    The velocity within which the car should be limited.
    :rype: float
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
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')

    curvature_param = rospy.search_param('curvature')
    curvature = rospy.get_param(curvature_param)
    curvature = float(curvature)

    speed_limit_param = rospy.search_param('speed_limit')
    speed_limit = rospy.get_param(speed_limit_param)
    speed_limit = float(speed_limit)

    start_pt = rospy.get_param(start_pt_param, init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]

    coords_param = rospy.search_param('coords_param')
    coords = rospy.get_param(coords_param)
    coords = json.loads(coords)

    # make the path smooth
    cx, cy, cyaw = create_smooth_path(coords, curvature)
    cyaw = smooth_yaw(cyaw)

    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, visualize_plot)

    return start_pt, speed_limit, use_rviz, use_matplotlib, cx, cy, cyaw


def main():
    """
    Initializes the parameters from the launch file.
    Creates path using dubins planner.
    Starts the instances of SimpleBicycleState, ConstraintsInterface and SimSVEA.
    Creates publishers for visualization.
    Pure pusuit is used for path tracking. PID is used for limiting velocity.
    """    
    rospy.init_node('floor2_example')
    start_pt, speed_limit, use_rviz, use_matplotlib, cx, cy, cyaw = init()

    final_point = (-8.45582103729,-5.04866838455)
    cx.append(final_point[0])
    cy.append(final_point[1])

    # initialize simulated model and control interface
    simple_bicycle_model = SimpleBicycleState(*start_pt, dt=dt)
    ctrl_interface = ConstraintsInterface(vehicle_name, speed_limit).start()
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
    lastIndex = len(cx) - 2
    target_ind = pure_pursuit.calc_target_index(simple_bicycle_model, cx, cy)
    # simualtion + animation loop
    time = 0.0
    steering = 0.0
    while lastIndex > target_ind and not rospy.is_shutdown():

        # compute control input via pure pursuit
        error = speed_limit - simple_bicycle_model.v
        velocity = ctrl_interface.pid_controller(error)

        steering, target_ind = \
            pure_pursuit.pure_pursuit_control(simple_bicycle_model, cx, cy, target_ind)
        ctrl_interface.send_control(steering, velocity)

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


    # points to project goal onto
    p1 = (-2.33859300613, 3.73132610321)
    p2 = (-2.86509466171, 3.14682602882)
    line = (p2[0]-p1[0], p2[1]-p1[1])
    proj_p2 = np.dot(p2,line)

    # continue following final point until goal is reached.
    while not rospy.is_shutdown():
        steering, target_ind = \
            pure_pursuit.pure_pursuit_control(simple_bicycle_model, cx, cy, target_ind)
        proj_car = np.dot((simple_bicycle_model.x, simple_bicycle_model.y), line)
        if proj_car < proj_p2:
            ctrl_interface.send_control(steering, velocity)
        else:
            # brake if goal is reached.
            ctrl_interface.send_control(0,0,100)
            rospy.sleep(0.1)
            break
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
