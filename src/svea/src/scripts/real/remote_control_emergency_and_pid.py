#!/usr/bin/env python


"""
Node to test the speed limit and the emergency brake using the remote controller.
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

from controllers.constraints_interface import ConstraintsInterface, BrakeControl


def main():
    """
    Initializes the parameters from launch file.
    Starts the instance of ConstraintsInterface and BrakeControl.
    When operated through remote: If the car senses the emergency, it stops. 
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

    vehicle_name = rospy.get_param(rospy.get_name() + '/vehicle_name')

    # starts instance of control interface with speed constraints functionality
    cont_intf = ConstraintsInterface(vehicle_name, speed_limit, trigger_sensitivity).start()

    # starts instance of brake control interface.
    brake_controller = BrakeControl(vehicle_name)

    rospy.sleep(2)
    rate = rospy.Rate(30)

    # sends commands from controller but with constrained speed.
    # if there is an emergency the car brakes.
    while not rospy.is_shutdown():
        if not brake_controller.is_emergency:
            error = cont_intf.control_error()
            velocity = cont_intf.pid_controller(error)
            cont_intf.send_control(cont_intf.remote_steering,
                                   velocity,
                                   0,  # zero brake force
                                   cont_intf.remote_transmission)
        else:
            brake_controller.brake_car()
            cont_intf.integral = 0  # reset integrator in PID
        rate.sleep()

if __name__ == '__main__':
    main()
