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

class pcl_filter:
    def __init__(self):
        # Initialize Subscribers
        self.pointcloud2_sub = rospy.Subscriber("/head_camera/depth_downsample/points"  , PointCloud2    ,self.pointcloud_data, queue_size=1)
        # self.pointcloud2_sub = rospy.Subscriber("/octomap_point_cloud_centers"  , PointCloud2    ,self.pointcloud_data, queue_size=1)
        self.plane_poly_sub  = rospy.Subscriber("/best_fit_plane_poly"                  , PolygonStamped ,self.pcl_filter, queue_size=1)

        # Initialize PointCloud Publisher
        self.pointcloud_pub = rospy.Publisher("/filtered_cloud", PointCloud, queue_size=1)

        # Initialize the camera PointCloud message type. This will be transfomed
        # to a the base_link frame_id
        self.camera_cloud = PointCloud()
        self.camera_cloud.header = Header()
        self.camera_cloud.header.stamp = rospy.Time.now()
        self.camera_cloud.header.frame_id = '/head_camera_depth_optical_frame'#/odom

        # Initialize filtered_cloud as a PointCloud message type
        self.filtered_cloud = PointCloud()
        self.filtered_cloud.header = Header()
        self.filtered_cloud.header.stamp = rospy.Time.now()
        self.filtered_cloud.header.frame_id = '/base_link'

        # Intialize pcl_data as an empty list
        self.pcl_data = []

        # Initialize self.cloud for data storage in pointcloud_data callback function
        self.cloud = None

    def pointcloud_data(self,ros_cloud):
        # Store pointcloud2 data
        self.cloud = ros_cloud
        # print("made it here")

    def pcl_filter(self,polygon):
        self.camera_cloud.points=[]
        # For loop to extract ros_cloud data into a list of x,y,z, and RGB (float)
        for data in pc2.read_points(self.cloud, skip_nans=True):
            self.camera_cloud.points.append(Point32(data[0],data[1],data[2]))

        transformed_cloud = self.transform_pointcloud(self.camera_cloud)
        # print(transformed_cloud)

        # Intialize region as a list
        region = []

        # Run forloop to store polygon vertices in list named region
        for vertices in polygon.polygon.points:
            region.append([vertices.x,vertices.y])

        # Set self.filtered_cloud.points as a filter
        self.filtered_cloud.points=[]
        # Use for loop to extract x and y points from the transformed cloud
        for points in transformed_cloud.points: #self.pcl_data:
            X = points.x
            Y = points.y

            # Check to see if the x and y points are in the polygon region. If so
            # then append the x,y, and z points to the filtered cloud.
            line = geometry.LineString(region)
            point = geometry.Point(X, Y)
            polygon = geometry.Polygon(line)
            if polygon.contains(point) == True:
                self.filtered_cloud.points.append(Point32(points.x,points.y,points.z))

        # Publish filtered point_cloud data
        self.pointcloud_pub.publish(self.filtered_cloud)


    def transform_pointcloud(self,point_cloud):
        listener = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                new_cloud = listener.transformPointCloud("/base_link" ,point_cloud)
                return new_cloud
                if new_cloud:
                    break
            except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                pass

if __name__=="__main__":
    rospy.init_node('pcl_filter',anonymous=True)
    pcl_filter()
    rospy.spin()
