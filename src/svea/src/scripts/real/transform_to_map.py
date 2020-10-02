#!/usr/bin/env python

"""
Node which transforms the cars pose data into the map frame and publishes it.
"""


import rospy
import math
import tf2_ros
from geometry_msgs.msg import PoseStamped



if __name__ == '__main__':
    rospy.init_node('transform_to_map')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub_transform = rospy.Publisher('robot_pose', PoseStamped, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

   

        transform_msg = PoseStamped()
        transform_msg.header.frame_id = 'map'
        
        transform_msg.pose.position.x = trans.transform.translation.x
        transform_msg.pose.position.y = trans.transform.translation.y
        transform_msg.pose.position.z = trans.transform.translation.z
        transform_msg.pose.orientation.x = trans.transform.rotation.x
        transform_msg.pose.orientation.y = trans.transform.rotation.y
        transform_msg.pose.orientation.z = trans.transform.rotation.z
        transform_msg.pose.orientation.w = trans.transform.rotation.w

        pub_transform.publish(transform_msg)
        rate.sleep()
