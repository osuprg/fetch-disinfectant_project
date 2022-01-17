#!/usr/bin/env python3

# Import what we need
import rospy
import random
import numpy as np

from scipy import spatial
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point, PolygonStamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from grid_based_sweep_coverage_path_planner import planning_animation,planning

class Waypoint_generator:
    def __init__(self):
        # Initialize Subscribers
        self.plane_poly_sub  = rospy.Subscriber("/best_fit_plane_poly"  , PolygonStamped ,self.polygon_callback, queue_size=1)
        # self.IM_array_sub    = rospy.Subscriber('IM_pose_array' ,numpy_msg(Floats)  ,self.best_fit_plane_callback   ,queue_size=1)
        self.pointcloud_sub  = rospy.Subscriber("filtered_cloud"          ,PointCloud         ,self.data_pointcloud           ,queue_size=1)


        # Initialize Publishers
        self.waypoints_pub        = rospy.Publisher('waypoints'       , PoseArray , queue_size=1)
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker    , queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Intialize the inverse matrix
        self.M_inv = None

        # Intialize the X and Y coordinates of the 2D data points
        self.X = None
        self.Y = None
        # self.Z = None

        # Set the waypoint resolution (distance between points)
        self.resolution = .05

        # Initialize waypoints as a PoseArray
        self.waypoints = PoseArray()
        self.waypoints.header = self.header

        # Initialize waypoint_markers and all of the other feature values
        self.waypoints_marker = Marker()
        self.waypoints_marker.header = self.header
        self.waypoints_marker.type = Marker.LINE_STRIP
        self.waypoints_marker.action = Marker.ADD
        self.waypoints_marker.id = 1
        self.waypoints_marker.scale.x = .01
        self.waypoints_marker.scale.y = .01
        self.waypoints_marker.color.a = 1
        self.waypoints_marker.color.r = 0
        self.waypoints_marker.color.g = 0
        self.waypoints_marker.color.b = 1.0

        # Initialize name for filtered point cloud data
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_z = []

        # offset_val
        self.offset = 0.3


    def best_fit_plane_callback(self,arr_msg):
        # Conditional statement to process arr_msg
        if len(arr_msg.data) < 9:
            pass

        else:
            self.X = []
            self.Y = []
            # Sepearately store the components (XYZ) of the IM's
            for i in range(int(len(arr_msg.data)/3)):
                # print(arr_msg.data[i*3 + 0])
                self.X.append(arr_msg.data[i*3 + 0])
                self.Y.append(arr_msg.data[i*3 + 1])


    def polygon_callback(self,polygon_msg):
        self.X = []
        self.Y = []

        for vertices in polygon_msg.polygon.points:
            self.X.append(vertices.x)
            self.Y.append(vertices.y)

        self.X.append(self.X[0])
        self.Y.append(self.Y[0])




            # print(arr_msg.data)
    def data_pointcloud(self,pointcloud_msg):
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_z = []

        for i in range(len(pointcloud_msg.points)):
            # self.filtered_cloud.append([pointcloud_msg.points[i].x, pointcloud_msg.points[i].y, pointcloud_msg.points[i].z])
            self.cloud_x.append(pointcloud_msg.points[i].x)
            self.cloud_y.append(pointcloud_msg.points[i].y)
            self.cloud_z.append(pointcloud_msg.points[i].z)

        self.waypoint_planner()


    def waypoint_planner(self):

        # Acquire the planned x an y values from the planning function
        px,py = planning(self.X, self.Y, self.resolution)


        # Create KD tree for filtered cloud
        tree = spatial.KDTree(np.c_[self.cloud_x,self.cloud_y])



        # Create lists and array of waypoints to publish
        poses = []
        marker_list = []

        # Begin dimension increase for 2D coordinates (px, py) and store new coordinates
        for i in range(len(px)):
            # waypoint = np.matmul(self.M_inv, np.array([px[i], py[i], 0, 1]))

            # poses append to poses (Pose Array)
            closest_points_ii = tree.query_ball_point([px[i],py[i]], 0.05)

            if len(closest_points_ii ) == 0:
                continue

            z = []
            for j in closest_points_ii:
                z.append(self.cloud_z[j])


            p = Pose()
            p.position.x = px[i]
            p.position.y = py[i]
            p.position.z = max(z) + self.offset
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0
            poses.append(p)

            # Append position values for the marker
            marker_list.append(Point(p.position.x, p.position.y, p.position.z))



        # assisn poses to the PoseArray, self,waypoints.
        self.waypoints.poses = poses

        # Publish poses for computeCartesianPath
        self.waypoints_pub.publish(self.waypoints)

        # Publish markers for waypoints
        self.waypoints_marker.points = marker_list
        self.waypoints_marker_pub.publish(self.waypoints_marker)

        del self.cloud_x[:], self.cloud_y[:], self.cloud_z[:]


if __name__=="__main__":
    # Initialize the node
    rospy.init_node('waypoint_generator')
    Waypoint_generator()
    rospy.spin()
