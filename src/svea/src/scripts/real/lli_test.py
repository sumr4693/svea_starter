#!/usr/bin/env python

"""
Node to test the lli control interface by giving different directional commands to the car.
"""

import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped
from nav_msgs.msg import Path
from controllers.control_interfaces import ControlInterface

def main():
    """
    Starts instance of ControlInterface and waits for initialization.
    Sends move forward, move backward, turn left and turn right commands to the car for 3 seconds each.
    """
    rospy.init_node('lli_testing')
    vehicle_name = 'SVEA3'
    ctrl_interface = ControlInterface(vehicle_name).start()
    rate = 10
    r = rospy.Rate(rate)
    while not ctrl_interface.is_ready and not rospy.is_shutdown():
        rospy.sleep(0.1)
    rospy.loginfo(ctrl_interface)
    run_time = 3 
    """   
    rospy.loginfo("Engaging transmission for 3 seconds")
    i = 0
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(0, 0, transmission=0)
        r.sleep()
        i += 1
    """
    rospy.loginfo("Going forward for 3 seconds")
    i = 0
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(0, 0.3, transmission=0)
        r.sleep()
        i += 1
    
    rospy.loginfo("Going backwards for 3 seconds")
    i = 0
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(0, -0.3, transmission=0)
        r.sleep()
        i += 1
    
    i = 0
    rospy.loginfo("Braking for 3 seconds")
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(brake_force=20, transmission=0)
        r.sleep()
        i += 1
    
    i = 0
    rospy.loginfo("Going backwards for 3 seconds")
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(0, -0.3, transmission=0)
        r.sleep()
        i += 1
    
    i = 0
    rospy.loginfo("Turning left for 3 seconds")
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(10, 0, transmission=0)
        r.sleep()
        i += 1
    i = 0
    rospy.loginfo("Turning right for 3 seconds")
    while not rospy.is_shutdown() and i < rate*run_time:
        ctrl_interface.send_control(-10, 0, transmission=0)
        r.sleep()
        i += 1
        
    rospy.loginfo("Going idle")
    rospy.spin()
    
if __name__ == '__main__': main()
