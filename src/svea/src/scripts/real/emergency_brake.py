#!/usr/bin/env python

"""
Node which checks if there is a risk for the car to hit something.
It will raise an emergency if that is the case.
"""

import os
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray, Float32
from sensor_msgs.msg import LaserScan
from svea_arduino.msg import lli_ctrl
from nav_msgs.msg import Odometry


dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from controllers.control_interfaces import ControlInterface
from sensors.lidars import RPLidar
from sensors.hokuyo import HokuyoLidar
from read_speed import ReadSpeed


__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"

class ScanReader():
    '''
    Class for reading scans from Hokuyo lidar.
    
    :param cone_width: The range of angles from the scan which will be used.
                       Value given in degrees.
    :type cone_width: float
    '''
    def __init__(self, cone_width):
        self.ranges = []
        self.angles = []
        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.cone_width = cone_width*np.pi/180
        self.steering = 0
        self.vehicle_name = "SVEA3"
    

        rospy.Subscriber('scan', LaserScan, self._read_scan)
        rospy.Subscriber(self.vehicle_name+'/lli/ctrl_request', lli_ctrl, self._get_steering)

    def _read_scan(self, data):
        self.ranges = data.ranges
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment

    def _get_steering(self, msg):
        self.steering = msg.steering
        
    def _get_angles_indexes(self):
        self.angles = []
        self.angles.append(self.angle_min)
        cone_mid = np.pi/4*float(self.steering)/128
        cone_low_index = round((cone_mid - self.angle_min - self.cone_width/2)/self.angle_increment)
        cone_upper_index = round((cone_mid - self.angle_min + self.cone_width/2)/self.angle_increment)
        list_length = (self.angle_max - self.angle_min)/self.angle_increment
        if cone_low_index < 0:
            cone_low_index = 0
        if cone_upper_index > list_length - 1:
            cone_upper_index = list_length - 1
        return int(cone_low_index), int(cone_upper_index)

def main():
    """
    Emergency node which checks if the car is in an emergency situation or not.
    If it is in an emergency it publishes true to the topic isEmergency.
    """
    emergency_pub = rospy.Publisher("isEmergency", Bool, queue_size=30)
    rospy.init_node('emergency_brake')
    rate = rospy.Rate(30)
    scan_data = ScanReader(20)
    velocity_reader = ReadSpeed(3).start()
    emergency_msg = Bool()

    while not rospy.is_shutdown():
        if len(scan_data.ranges) > 0:
            il, iu = scan_data._get_angles_indexes()
            
            ranges = scan_data.ranges
            current_velocity = velocity_reader.get_velocity()
            emergency_distance = 0.4 + 0.4 * current_velocity
            if min(ranges[il:iu]) < emergency_distance:
                emergency_msg.data = True
            else:
                emergency_msg.data = False
            
            #print(emergency_msg)
            emergency_pub.publish(emergency_msg)
            rate.sleep()

if __name__ == '__main__':
    main()
