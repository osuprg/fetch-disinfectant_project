#!/usr/bin/env python

# Import modules
import rospy
import numpy as np
import ctypes
import struct
import tf
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from transform_frame_node import transformer
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32, PolygonStamped
from std_msgs.msg import Header
from shapely import geometry

class PointCloudFilter:
    def __init__(self):
        # Initialize Subscribers
        self.pointcloud2_sub = rospy.Subscriber("/head_camera/depth_downsample/points"  , PointCloud2    ,self.pointcloud_data, queue_size=1)
        self.plane_poly_sub  = rospy.Subscriber("/best_fit_plane_poly"                  , PolygonStamped ,self.pcl_filter, queue_size=1)

        # Initialize PointCloud Publisher
        self.pointcloud_pub = rospy.Publisher("/filtered_cloud", PointCloud, queue_size=1)

        # Initialize filt_cloud as a PointCloud message type
        self.filt_cloud = PointCloud()
        self.filt_cloud.header = Header()
        self.filt_cloud.header.stamp = rospy.Time.now()
        self.filt_cloud.header.frame_id = 'base_link'

        # Intialize pcl_data as an empty list
        self.pcl_data = []

        # Initialize self.cloud for data storage in pointcloud_data callback function
        self.cloud = None

    def pointcloud_data(self,ros_cloud):
        # Store pointcloud2 data
        self.cloud = ros_cloud

    def pcl_filter(self, polygon):
        # Run transformer function to get transform from the head_camera_depth_optical_frame
        # to the base_link
        trans,_ = self.transformer()
        # For loop to extract ros_cloud data into a list of x,y,z, and RGB (float)
        for data in pc2.read_points(self.cloud, skip_nans=True):
            self.pcl_data.append([trans[0] + data[2], trans[1] - data[0], trans[2] - data[1]])

        # Intialize region as a list.
        region = []

        # Run forloop to store polygon vertices in list named region.
        for vertices in polygon.polygon.points:
            region.append([vertices.x,vertices.y])

        for points in self.pcl_data:
            X = points[0]
            Y = points[1]
            line = geometry.LineString(region)
            point = geometry.Point(X, Y)
            polygon = geometry.Polygon(line)

            if polygon.contains(point) == True:
                self.filt_cloud.points.append(Point32(points[0],points[1],points[2]))

        # Publish filtered point_cloud data
        self.pointcloud_pub.publish(self.filt_cloud)

        # Reset filtered data points.
        del self.filt_cloud.points[:], self.pcl_data[:]


    def transformer(self):
        listener = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform("/base_link" ,"/head_camera_depth_optical_frame",  rospy.Time(0))
                return trans,rot
                if trans:
                    break
        # This will give you the coordinate of the child in the parent frame
            except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                pass

if __name__=="__main__":
    rospy.init_node('pcl_filter',anonymous=True)
    PointCloudFilter()
    rospy.spin()
