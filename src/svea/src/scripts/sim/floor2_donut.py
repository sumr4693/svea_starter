#!/usr/bin/env python

"""
Node to perform donuts in simulation using pure pursuit control.
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

dirname = os.path.dirname(__file__)
pure_pursuit = os.path.join(dirname,
        '../../PythonRobotics/PathTracking/pure_pursuit/')
sys.path.append(os.path.abspath(pure_pursuit))

import pure_pursuit

bezier_path = os.path.join(dirname,
        '../../PythonRobotics/PathPlanning/BezierPath/')
sys.path.append(os.path.abspath(bezier_path))

import bezier_path as bp

__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"

## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA0"
target_speed = 1.0 # [m/s]
dt = 0.01

###############################################################################

## INIT #######################################################################
init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
visualize_plot = True
###############################################################################


## PURE PURSUIT PARAMS ########################################################
pure_pursuit.k = 0.1  # look forward gain
pure_pursuit.Lfc = 0.1  # look-ahead distance
pure_pursuit.L = 0.324  # [m] wheel base of vehicle
###############################################################################

def smooth_donut_path():
    """
    Creates circular path using bezier path planning.

    :return: cx     List of x coordinates
    :rtype: list
    :return: cy     List of y coordinates
    :rtype: list 
    """
    xs = []
    ys = []
    r = 1
    theta = np.linspace(0, 2*math.pi, 100)

    for angle in theta:
        xs.append(r*math.cos(angle))
        ys.append(r*math.sin(angle))

    control_points = []
    for i in range(len(xs)):
        control_points = control_points + [[xs[i], ys[i]]]

    control_points = np.array(control_points)
    n_points = 500
    path = bp.calc_bezier_path(control_points, n_points)
    cx = []
    cy = []
    for cord in path:
        cx = cx + [cord[0]]
        cy = cy + [cord[1]]
    return cx, cy



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
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')
    start_pt = rospy.get_param(start_pt_param, init_pt)
    if type(start_pt) == type(''):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]

    cx, cy = smooth_donut_path()
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, visualize_plot)

    return start_pt, use_rviz, use_matplotlib, cx, cy


def main():
    """
    Initializes the parameters from the launch file.
    Creates circular path.
    Starts the instances of SimpleBicycleState, ControlInterface and SimSVEA.
    Creates publishers for visualization.
    Pure pursuit is used for path tracking. PID is used for limiting velocity.
    """
    rospy.init_node('floor2_example')
    start_pt, use_rviz, use_matplotlib, cx, cy = init()

    # initialize simulated model and control interface
    simple_bicycle_model = SimpleBicycleState(*start_pt, dt=dt)
    ctrl_interface = ControlInterface(vehicle_name).start()
    rospy.sleep(0.2)

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
    target_ind = pure_pursuit.calc_target_index(simple_bicycle_model, cx, cy)
    print(target_ind)
    # simualtion + animation loop
    time = 0.0
    steering = 0.0
    while lastIndex > target_ind and not rospy.is_shutdown():

        # compute control input via pure pursuit
        steering, target_ind = \
            pure_pursuit.pure_pursuit_control(simple_bicycle_model, cx, cy, target_ind)
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


def plot_trajectory(car_model, x, y):
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Heading[deg]: "
                + str(math.degrees(car_model.yaw))[:4]
                +" | Speed[m/s]:"
                + str(car_model.v)[:4])

def animate_pure_pursuit(car_model, x, y, steering, target_ind):
    """ Single animation update for pure pursuit."""
    plt.cla() #clear last plot/frame
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
