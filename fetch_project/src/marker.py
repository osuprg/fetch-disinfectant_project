#!/usr/bin/env python
import rospy

import sys, os
import moveit_msgs.msg
# from moveit_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal
import tf.transformations
# import std_msgs.msg


import serial
import time
#from qrtestRGB import main_f
import subprocess
from visualization_msgs.msg import *
from geometry_msgs.msg import Quaternion, Pose, Point
from tf2_msgs.msg import TFMessage
i_marker = 0


def callback_marker(tf_msg):
	global i_marker
        if (i_marker%2 == 0):    
	    marker1 = Marker()
	    marker1.header.frame_id = "/gripper_link"
	    marker1.type = marker1.ARROW
	    marker1.action = marker1.ADD
	    lifetime=rospy.Duration(15000)
	    marker1.scale.x = 0.2
	    marker1.scale.y = 0.015
	    marker1.scale.z = 0.015
	    marker1.color.a = 1.0
	    marker1.color.r = 1.0
	    marker1.color.g = 1.0
	    marker1.color.b = 0.0
	    marker1.id = i_marker
	    #marker1.pose.orientation.w = 1.0
	    marker1.pose = Pose(Point(0,0,0), Quaternion(0, 0.7071068, 0, 0.7071068))
	    publisher.publish(marker1)
	
        else:
	    marker2 = Marker()
	    marker2.header.frame_id = "/gripper_link"
	    marker2.type = marker2.SPHERE
	    marker2.action = marker2.ADD
	    lifetime=rospy.Duration(15000)
	    marker2.scale.x = 0.005
	    marker2.scale.y = 0.05
	    marker2.scale.z = 0.05
	    marker2.color.a = 1.0
	    marker2.color.r = 1.0
	    marker2.color.g = 0.0
	    marker2.color.b = 0.0
	    marker2.id = i_marker
	    marker2.pose.orientation.w = 1.0
	    marker2.pose = Pose(Point(0,0,-0.2), Quaternion(0, 0.7071068, 0, 0.7071068))
	    publisher.publish(marker2)
        
        i_marker += 1



if __name__ == '__main__':
	rospy.init_node('marker')
	topic = 'visualization_marker'
	publisher = rospy.Publisher(topic, Marker, queue_size=5)
	sub = rospy.Subscriber('/tf', TFMessage, callback_marker)
	rospy.spin()
