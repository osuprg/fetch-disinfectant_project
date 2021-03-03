#!/usr/bin/env python

# Import what we need.
import rospy
import sys
import tf
import numpy as np

from geometry_msgs.msg import Point, Pose, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from best_fit import fit


class Laser_filter:
	def __init__(self):
		# Set up a subscriber the subscribes to a topic called "base_scan" with
		# a message type LaserScan. This will be sent to the laser_callback function.
		# self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

		# Initialize publishers
		self.heatmap_marker_pub  = rospy.Publisher('text_marker',	 Marker,	queue_size=10)

		# Setup header
		self.header = Header()
		self.header.frame_id = "/gripper_link"
		self.header.stamp = rospy.Time.now()

		# Create triangle_list marker to fill in best fit plane
		self.heatmap = Marker()
		self.heatmap.header = self.header
		self.heatmap.type = Marker.SPHERE
		self.heatmap.action = Marker.ADD
		self.heatmap.color.a = .05
		self.heatmap.scale.z = 0.001
		self.heatmap.pose.position.x =  0.0
		self.heatmap.pose.position.y =  0.0
		self.heatmap.pose.position.z =  -.3
		self.heatmap.pose.orientation.w = -1.0

		self.model = fit(16, plotter = False)


		self.id = 0
		self.alpha = np.linspace(0.05, 0.0005, 10)
		self.dist  = np.linspace(-0.19, -.21, 10)
		self.scale = np.linspace(0.01, 0.1,10)
		self.vals  = self.model(np.linspace(0,5,10))
		self.color = []
		for i in self.vals:
			self.color.append(1 - (i/self.vals[0]))

		# print(self.color)

	def heatmap_marker(self):

		for c, s, d, a in zip(self.color,self.scale, self.dist,self.alpha):

			self.heatmap.scale.x = s
			self.heatmap.scale.y = s
			self.heatmap.scale.z = 0.001
			self.heatmap.color.r = 1.0
			self.heatmap.color.g = c
			self.heatmap.color.b = 0
			self.heatmap.pose.position.z = d
			self.heatmap.color.a = a
			self.heatmap.id = self.id
			self.id += 1

			self.heatmap_marker_pub.publish(self.heatmap)





if __name__ == '__main__':
	# Initialize the node
 	rospy.init_node('heatmap')
	func = Laser_filter()
	# Loop at 10 Hz, dropping breadcrumbs on each loop.
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		func.heatmap_marker()

		rate.sleep()
