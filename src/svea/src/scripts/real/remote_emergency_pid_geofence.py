#!/usr/bin/env python


"""
Node which is used to test the geofence using the remote controller.
"""


import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import json
import time

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from simulators.viz_utils import plot_car, publish_3Dcar
from simulators.viz_utils import publish_path, publish_target
from controllers.control_interfaces import ControlInterface
from controllers.constraints_interface import ConstraintsInterface, BrakeControl

smooth_path = os.path.join(dirname, '../util/')
sys.path.append(os.path.abspath(smooth_path))

from state_subscriber import *
from geofence import Geofence

def main():
    """
    Initializes the parameters from launch file.
    Creates publishers for visualization.
    Starts the instances of ConstraintsInterface, BrakeControl, Geofence and StateSubscriber.
    Waits for center point and radius to be given in rviz.
    Creates geofence, can be seen in rviz.
    When operated through remote: If the car senses the emergency, it stops. 
                                  Or if the car is outside the geofence, it stops.
                                  Otherwise, it runs with limited velocity.
    """    
    rospy.init_node('velocity_constraint_node')

    # Gets the current speed limit from the launch file.
    speed_limit_param = rospy.search_param('speed_limit')
    speed_limit = rospy.get_param(speed_limit_param)
    speed_limit = float(speed_limit)

    # Gets the trigger sensitivity from the launch file
    trigger_param = rospy.search_param('trigger_sensitivity')
    trigger_sensitivity = rospy.get_param(trigger_param)
    trigger_sensitivity = float(trigger_sensitivity)


    car_poly_pub = rospy.Publisher("/3D_car", PolygonStamped, queue_size=1)
    pose_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=1)

    # wait for the initial position of the car
    rospy.loginfo('Waiting for initial pose')
    rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
    rospy.loginfo('Received initial pose')

    vehicle_name = rospy.get_param(rospy.get_name() + '/vehicle_name')

    # starts instance of control interface with speed constraints functionality
    cont_intf = ConstraintsInterface(vehicle_name,
                                     speed_limit,
                                     trigger_sensitivity).start()

    # starts instance of brake control interface.
    brake_controller = BrakeControl(vehicle_name)

    # start geofence
    geofence = Geofence().start()

    # starting car position subscriber
    car_position = StateSubscriber().start()

    rospy.sleep(1)
    # define the geofence
    geofence.set_center()
    geofence.set_radius()

    # publish the geofence to rviz
    geofence.define_marker()
    geofence.publish_marker()

    rate = rospy.Rate(30)
    # sends commands from controller but with constrained speed.
    # if there is an emergency the car brakes.
    while not rospy.is_shutdown():
        
        if brake_controller.is_emergency:
            brake_controller.brake_car()
            print('brake')
            cont_intf.integral = 0  # reset integrator in PID
        if geofence.is_outside_geofence(car_position):
            brake_controller.brake_car_extreme()
            cont_intf.integral = 0  # reset integrator in PID
        else:
            error = cont_intf.control_error()
            velocity = cont_intf.pid_controller(error)
            cont_intf.send_control(cont_intf.remote_steering,
                                   velocity,
                                   0,  # zero brake force
                                   cont_intf.remote_transmission)
        publish_3Dcar(car_poly_pub, pose_pub,
                      car_position.x,
                      car_position.y,
                      car_position.yaw)
        rate.sleep()

if __name__ == '__main__':
    main()
