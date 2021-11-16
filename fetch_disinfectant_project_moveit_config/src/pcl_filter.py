#!/usr/bin/env python3

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

"""
# Modified by Anshul Paigwar
# @email: p.anshul6@gmail.com
#
# Added new funtions:
# array_to_pointcloud2
# xyzrgb_array_to_pointcloud2
# .
# .
# .
Reference:
https://www.programcreek.com/python/example/99841/sensor_msgs.msg.PointCloud2
"""



# Import modules
import rospy
import pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint


class PointCloudFilter:
    def __init__(self):
        #Initialize Pointcloud Subscriber
        self.pointcloud2_sub = rospy.Subscriber("/realsense/depth/color/points",PointCloud2, self.ros_to_pcl, queue_size=1)

        self.flag = 0

    def ros_to_pcl(ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])

        pcl_data = pcl.PointCloud_PointXYZRGB()
        pcl_data.from_list(points_list)

        if self.flag == 0:
            print (pcl_data)
            self.flag = 1
        # return pcl_data



    def pointcloud2_to_array(cloud_msg, squeeze=True):
        ''' Converts a rospy PointCloud2 message to a numpy recordarray

        Reshapes the returned array to have shape (height, width), even if the height is 1.

        The reason for using np.fromstring rather than struct.unpack is speed... especially
        for large point clouds, this will be <much> faster.
        '''
        # construct a numpy record type equivalent to the point type of this cloud
        dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

        # parse the cloud into an array
        cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

        # remove the dummy fields that were added
        cloud_arr = cloud_arr[
            [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

        if squeeze and cloud_msg.height == 1:
            return np.reshape(cloud_arr, (cloud_msg.width,))
        else:
            return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))
