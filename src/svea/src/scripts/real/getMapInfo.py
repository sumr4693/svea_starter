#!/usr/bin/env python


import sys
import os
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import json

from threading import Thread
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, OccupancyGrid

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

util = os.path.join(dirname, '../util/')
sys.path.append(os.path.abspath(util))

from data_pooling import *

__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"


class GetMapInfo():
    """
    Class to include the obstacle info in the existing map and create an obstacle occupancy map with memory.

    :param memory_span: No. of seconds that the obstacle map data to be remembered.
    :type memory_span: int
    """
    def __init__(self,memory_span = 4):
        self.map = None
        self._memory_index = 0
        self.memory_span = memory_span
        self.map_memory = None
        self.is_data_available = False
        self.visualize = True
        self.is_ready = False

        self.resolution = None
        self.width = None
        self.height = None
        self.origin = Point()
        self.occ_map_data = OccupancyGrid()

    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data
        :return: itself
        :rtype: getMapInfo
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo(
                'Get Map Info Node: \n'
                + str(self))
        self._start_publish()
        self._start_listen()
        self.is_ready = True
        rospy.spin()

    def _start_listen(self):
        rospy.Subscriber('/map', OccupancyGrid, self._get_map_cb)
        rospy.loginfo("Get Map Info successfully initialized")

    def _start_publish(self):
        self.map2_pub = rospy.Publisher("/map_obstacle", OccupancyGrid, queue_size=1)

    def _get_map_cb(self, msg):
        if self.is_data_available == False:
            threshold = 65
            self.occ_map_data = msg
            temp_map = np.array(msg.data)
            self.width = msg.info.width
            self.height = msg.info.height
            self.map = np.array((temp_map > threshold)*1).reshape(self.height, self.width)
            self.occ_map_data.data = list(temp_map*(temp_map > threshold)*1)
            self.resolution = msg.info.resolution
            self.origin = msg.info.origin.position
            self.is_data_available = True
            #self._add_the_walls()
            #self.subsample(2)
            self.init_memory_map()
            self.visualize_map()
            
    def init_memory_map(self):
        self.map_memory = np.zeros([self.memory_span, self.map.shape[0],self.map.shape[1]])
        for i  in range(self.memory_span):
            self.map_memory[i,:,:] = np.copy(self.map)
            
                    
    def _add_the_walls(self):
        """
        Adds extra walls for the map
        """
        self.add_walls([-9.12,-3.59],[-6.89,-1.93])
        self.add_walls([7.97,5.26],[4.02,-0.06])
        self.add_walls([-9.37,-4.03],[-7.32,-5.12])
        self.add_walls([-4.27,-7.88],[-2.02,-8.37])
        self.add_walls([7.14,15.68],[12.87,10.72])
        self.add_walls([12.11,9.98],[7.95,5.33])
        self.add_walls([7.26,15.68],[-1.11,4.73])
        self.add_walls([9.46,9.18],[-2.34,-5.48])
        self.add_walls([7.5,13.26],[-5.62,-2.84])
        self.add_walls([10.51,10.97],[7.46,13.38])
        self.add_walls([3.19,-0.94],[2.83,-1.75])
        self.add_walls([-1.83,3.85],[-2.16,3.78])
        self.add_walls([-6.726,-1.568],[-6.708,-1.79])
        self.add_walls([-1.42,-6.59],[-1.67,-6.88])
    
    def subsample(self,sub_ratio):
        """
        Subsamples map with given sub_ratio

        :param sub_ratio: Subsampling factor
        :type sub_ratio: int
        """
        self.map=poolingOverlap(self.map,(sub_ratio,sub_ratio))
        height, width = self.map.shape
        self.width = width
        self.height = height        
        self.resolution = self.resolution*sub_ratio
        
    def visualize_map(self):
        """
        Makes ready the msg for visualization
        """
        self.occ_map_data.info.resolution = self.resolution
        self.occ_map_data.info.width = self.width
        self.occ_map_data.info.height = self.height
        self.occ_map_data.data = np.dot(100,self.get_map().reshape(self.width*self.height)).tolist()

        
    def add_walls(self, p1,p2):
        """ 
        Adds walls in the occupancy map

        :param p1: A list that contains x coordinates of walls
        :type p1: list
        :param p2: A list that contains y coordinates of walls
        :type p2: list
        """
        # interpolate
        spacing = round(np.linalg.norm(np.array(p1)-np.array(p2))/self.resolution)
        x, y =[np.linspace(p1[i], p2[i], spacing+1) for i in range(len(p1))]
        grid_x = np.round((x - self.origin.x) / self.resolution).astype(int)
        grid_y = np.round((y - self.origin.y) / self.resolution).astype(int)

        # add in map
        for i in range(len(grid_x)):
            self.map[grid_y[i], grid_x[i]] = 1
            
    def update_map(self, obs):
        """
        Updates map with the given obstacles in obs

        :param obs: A list containing x and y coordinates of obstacles in separate indices
        :type obs: list
        """
        self.map_memory[self._memory_index,:,:] = np.copy(self.map)
        grid_x = np.round((obs[0] - self.origin.x) / self.resolution).astype(int)
        grid_y = np.round((obs[1] - self.origin.y) / self.resolution).astype(int)
        # add in map
        for i in range(len(grid_x)):
            self.map_memory[self._memory_index, grid_y[i], grid_x[i]] = 1
        self._memory_index = (self._memory_index + 1) % self.memory_span 

    def get_map(self):
        """
        Returns the memory occupancy map
        """
        return np.logical_or.reduce(self.map_memory)*1
            
    def publish_map2_data(self):
        """
        Publishes the memory occupancy map to visualize in rviz
        """
        self.visualize_map()
        if self.is_data_available == True and self.visualize:
            self.map2_pub.publish(self.occ_map_data)

def main():
    """
    Starts the instance of getMapInfo.
    Publishes the memory occupancy map.
    """
    rospy.init_node('get_map_info')
    getmap = getMapInfo().start()
    rospy.sleep(0.2)
    while not rospy.is_shutdown():
        getmap.publish_map2_data()


if __name__ == '__main__':
    main()