#!/usr/bin/env python

import sys
import actionlib
import rospy
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from scipy import spatial
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32, PolygonStamped, Pose
from std_msgs.msg import Header
from shapely import geometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from std_msgs.msg import Header


class Irradiance_simulation(object):

    def __init__(self):
        # Initialize Subscribers
        self.gui_input_sub = rospy.Subscriber('gui_input', String, self.interface_callback)
        self.pointcloud2_sub = rospy.Subscriber("/octomap_point_cloud_centers"  , PointCloud2    ,self.pointcloud_data, queue_size=1)

        # Initialize Publishers
        self.waypoints_marker_pub = rospy.Publisher('irradiance', Marker , queue_size=1)

        # Initialize transform listener
        self.listener = tf.TransformListener()

        # Initialize self.cloud for data storage in pointcloud_data callback function
        self.cloud = None

        self.listener = tf.TransformListener()

        # Initialize the camera PointCloud message type. This will be transfomed
        # to a the base_link frame_id
        self.camera_cloud = PointCloud()
        self.camera_cloud.header = Header()
        self.camera_cloud.header.stamp = rospy.Time.now()
        self.camera_cloud.header.frame_id = '/odom'

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Initialize waypoint_markers and all of the other feature values
        self.waypoints_marker = Marker()
        self.waypoints_marker.header = self.header
        self.waypoints_marker.type = Marker.ARROW
        self.waypoints_marker.scale.x = 0.03
        self.waypoints_marker.scale.y = 0.01
        self.waypoints_marker.scale.z = 0.005
        self.waypoints_marker.color.a = 1
        self.waypoints_marker.color.r = 1
        self.waypoints_marker.color.g = 0
        self.waypoints_marker.color.b = 0
        self.waypoints_marker.id = 1

    def pointcloud_data(self,ros_cloud):
        # Store pointcloud2 data
        self.cloud = ros_cloud
        # print("made it here")

    def interface_callback(self,gui_input):
        #
        if gui_input.data == "1": #1
            self.distance_calculator()


    def distance_calculator(self):
        # self.camera_cloud.points=[]
        # # For loop to extract ros_cloud data into a list of x,y,z, and RGB (float)
        # for data in pc2.read_points(self.cloud, skip_nans=True):
        #     self.camera_cloud.points.append(Point32(data[0],data[1],data[2]))
        #
        # transformed_cloud = self.transform_pointcloud(self.camera_cloud)
        #
        # x = []
        # y = []
        # z = []
        # for i in range(len(transformed_cloud.points)):
        #     x.append(transformed_cloud.points[i].x)
        #     y.append(transformed_cloud.points[i].y)
        #     z.append(transformed_cloud.points[i].z)
        #
        # tree = spatial.KDTree(np.c_[x,y,z])

        rate = rospy.Rate(10.0)
        start = rospy.get_time()


        while not rospy.is_shutdown():
            trans, rot = self.find_ee_pose()
            poses = []

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            x = np.cos(yaw)*np.cos(pitch)
            y = np.sin(yaw)*np.cos(pitch)
            z = np.sin(pitch)

            p = Pose()
            p.position.x = trans[0] + (0.3*x)
            p.position.y = trans[1] + (0.3*y)
            p.position.z = trans[2] - (0.3*z)
            p.orientation.x = rot[0]
            p.orientation.y = rot[1]
            p.orientation.z = rot[2]
            p.orientation.w = rot[3]
            poses.append(p)

            self.waypoints_marker.pose = p
            self.waypoints_marker_pub.publish(self.waypoints_marker)

            if (rospy.get_time() - start) > 15:
                break

    def find_ee_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform( '/base_link', '/ee_link',rospy.Time(0))
                return [trans,rot]
                if trans:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def transform_pointcloud(self,point_cloud):
        while not rospy.is_shutdown():
            try:
                new_cloud = self.listener.transformPointCloud("/base_link" ,point_cloud)
                # print("made it here")
                return new_cloud
                if new_cloud:
                    break
            except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                    pass


if __name__=="__main__":
    rospy.init_node('irradiance_simulation',anonymous=True)
    Irradiance_simulation()
    rospy.spin()
