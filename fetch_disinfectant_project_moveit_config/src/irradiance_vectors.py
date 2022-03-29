#!/usr/bin/env python
from __future__ import division

import sys
import actionlib
import rospy
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from best_fit import fit
from scipy import spatial
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Pose, Vector3Stamped, PointStamped
from std_msgs.msg import Header, String, Int32, Float32
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class IrradianceVectors(object):

    def __init__(self):
        # Initialize Subscribers
        self.gui_input_sub = rospy.Subscriber('gui_input', String, self.interface_callback)
        self.duration_sub  = rospy.Subscriber('duration' , Float32 , self.duration_callback)

        # Initialize Publishers
        self.waypoints_marker_pub = rospy.Publisher('irradiance', Marker , queue_size=10)
        self.vector_array_pub = rospy.Publisher('vectors', numpy_msg(Floats), queue_size=10)

        # Initialize transform listener
        self.listener = tf.TransformListener()

        # Initialize self.cloud for data storage in pointcloud_data callback function
        self.cloud = None

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Initialize Point data
        self.Pt = PointStamped()
        self.Pt.header = Header()
        self.Pt.header.frame_id = "/ee_link"
        self.Pt.header.stamp = rospy.Time.now()

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

        self.model = fit(16, plotter = False)

        self.duration = 1

    def interface_callback(self,gui_input):
        #
        if gui_input.data == "1":
            self.distance_calculator()

    def duration_callback(self, duration):
        #
        self.duration = duration.data
        # print(self.duration)


    def distance_calculator(self):

        n = [1, 6, 12]
        r = [0.0, 0.025, 0.05]
        circles = self.circle_points(r,n)
        # print(len(circles))
        self.vectors = np.empty(shape=[len(circles),4])

        rate = rospy.Rate(5.0)
        start = rospy.get_time()
        iterations = 0
        while not rospy.is_shutdown():

            M = self.get_matrix(self.Pt)
            ee_trans, ee_rot = self.find_ee_pose()

            i = 0
            for e in circles:
                pt_coord = [e[0], e[1], e[2], 1]
                transformed_pt_coord = np.dot(M,pt_coord)

                self.vectors[i] = [transformed_pt_coord[0] - ee_trans[0],
                                   transformed_pt_coord[1] - ee_trans[1],
                                   transformed_pt_coord[2] - ee_trans[2],
                                   1]

                if i == 0:
                    self.vectors[i][3] = self.model(r[0]*100)

                elif i > 0 and i < 6:
                    self.vectors[i][3] = self.model(r[1]*100)

                elif i >= 6:
                    self.vectors[i][3] = self.model(r[2]*100)


                i+=1


            a = np.array([ee_trans[0],ee_trans[1],ee_trans[2]], dtype=np.float32)
            b = np.array(self.vectors.ravel(), dtype=np.float32)
            self.vector_array_pub.publish(np.concatenate((a,b)))

            # self.vector_array_pub.publish(self.vectors)

            if (rospy.get_time() - start) > self.duration:
                # print(self.vectors)
                # print(type(self.vectors))

                break

            rate.sleep()

    def find_ee_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform( '/base_link', '/ee_link',rospy.Time(0))
                return [trans,rot]
                if trans:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    def get_matrix(self, Pt):
        while not rospy.is_shutdown():
            try:
                Pts = Header()
                Pts.frame_id = "/ee_link"
                Pts.stamp = rospy.Time.now()
                new_Point = self.listener.asMatrix('/base_link', Pts)
                return new_Point
                if new_Point:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    def circle_points(self, r, n):
        circles = []
        for r, n in zip(r, n):
            t = np.linspace(0, 2*np.pi, n, endpoint=False)
            z = r * np.cos(t)
            y = r * np.sin(t)
            x = [0.3]*len(z)
            circles.append(np.c_[x, y, z])
            concatenate = np.concatenate( circles, axis=0 )
        return concatenate


if __name__=="__main__":
    rospy.init_node('irradiance_vectors',anonymous=True)
    IrradianceVectors()
    rospy.spin()






# # Delete previous ARROW markers and publish the empty data structure
# self.waypoints_marker.action = Marker.DELETEALL
# self.waypoints_marker_pub.publish(self.waypoints_marker)
#
# # Set marker action to add for new ARROW markers
# self.waypoints_marker.action = Marker.ADD

# poses = []
    # # Include characteristics of a pose
    # p = Pose()
    # p.position.x = transformed_pt_coord[0]
    # p.position.y = transformed_pt_coord[1]
    # p.position.z = transformed_pt_coord[2]
    # p.orientation.x = ee_rot[0]
    # p.orientation.y = ee_rot[1]
    # p.orientation.z = ee_rot[2]
    # p.orientation.w = ee_rot[3]
    # poses.append(p)
    #
    # # Create new marker id and pose to be published
    # self.waypoints_marker.id = i
    # self.waypoints_marker.pose = p
    # self.waypoints_marker_pub.publish(self.waypoints_marker)
