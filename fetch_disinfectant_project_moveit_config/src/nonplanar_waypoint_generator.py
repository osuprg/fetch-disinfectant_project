#!/usr/bin/env python3

# Import what we need
import rospy
import random
import numpy as np
import pyvista as pv

from scipy import spatial, stats
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point, PolygonStamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from grid_based_sweep_coverage_path_planner import planning_animation,planning
from scipy.spatial.transform import Rotation as R

class Nonplanar_waypoint_generator:
    def __init__(self):
        # Initialize Subscribers
        self.plane_poly_sub  = rospy.Subscriber("/best_fit_plane_poly"  ,PolygonStamped ,self.polygon_callback  ,queue_size=1)
        self.pointcloud_sub  = rospy.Subscriber("filtered_cloud"        ,PointCloud     ,self.data_pointcloud   ,queue_size=1)


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
        self.waypoints_marker.type = Marker.ARROW
        self.waypoints_marker.scale.x = 0.03
        self.waypoints_marker.scale.y = 0.01
        self.waypoints_marker.scale.z = 0.005
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


    def polygon_callback(self,polygon_msg):
        # Store the x and y positions of polygon message in separate lists
        self.X = []
        self.Y = []

        # For loop to extract x and y coordinates of the polygon
        for vertices in polygon_msg.polygon.points:
            self.X.append(vertices.x)
            self.Y.append(vertices.y)

        # Append first x and y coordinates to close the polygon loop
        self.X.append(self.X[0])
        self.Y.append(self.Y[0])


    def data_pointcloud(self,pointcloud_msg):
        # Store the x, y, and z coordinates of the filtered pointcloud in separate lists
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_z = []

        # For loop to extract x, y, and z coordinates of the pointcloud
        for i in range(len(pointcloud_msg.points)):
            self.cloud_x.append(pointcloud_msg.points[i].x)
            self.cloud_y.append(pointcloud_msg.points[i].y)
            self.cloud_z.append(pointcloud_msg.points[i].z)

        # Run waypoint planner for non-planar surfaces.
        self.waypoint_planner()


    def waypoint_planner(self):

        # Acquire the planned x an y values from the planning function in the
        # grid_based_sweep_coverage_path_planner python script.
        px,py = planning(self.X, self.Y, self.resolution)

        # Create space-partition data strucutre (KD tree) for filtered cloud
        tree = spatial.KDTree(np.c_[self.cloud_x,self.cloud_y])


        # Pass the pointcloud points to the PolyData constructor. Then run the
        # delaunay_2d triangulation function on the PolyData
        cloud = pv.PolyData(np.c_[self.cloud_x, self.cloud_y, self.cloud_z])
        surf = cloud.delaunay_2d()

        # Delete previous ARROW markers and publish the empty data structure
        self.waypoints_marker.action = Marker.DELETEALL
        self.waypoints_marker_pub.publish(self.waypoints_marker)

        # Set marker action to add for new ARROW markers
        self.waypoints_marker.action = Marker.ADD


        # Create lists for waypoints and waypoint markers that will be publish
        poses = []
        marker_list = []

########################## Test Case 1: Fixed height ###########################
        # z_index = self.cloud_z.index(max(self.self.cloud_z))
        # for i in range(len(px)):
        #     # Get index values closest neighbors of the planned x and y values
        #     # in the KDtree
        #     closest_points_ii = tree.query_ball_point([px[i],py[i]], 0.025)
        #
        #     # If there are no closest neighbors, then continue to next iteration
        #     # of the for loop
        #     if len(closest_points_ii ) == 0:
        #         continue
        #
        #     self.z_val = []
        #     # append the z values of the point cloud of the indexed values.
        #     for j in closest_points_ii:
        #         self.z_val.append(self.cloud_z[j])
        #
        #     # Remove any z outliers of the stored data.
        #     self.remove_outliers()
        #
        #     # Find the index of the max z value from the closest neighbors point cloud
        #     index = closest_points_ii[self.z_val.index(max(self.z_val))]
        #
        #     # Include characteristics of a pose
        #     p = Pose()
        #     p.position.x = self.cloud_x[index]
        #     p.position.y = self.cloud_y[index]
        #     p.position.z = self.cloud_z[z_index]
        #     p.orientation.x = 0
        #     p.orientation.y = 0
        #     p.orientation.z = 0
        #     p.orientation.w = 1
        #     poses.append(p)
        #
        #     # Create new marker id and pose to be published
        #     self.waypoints_marker.id = i
        #     self.waypoints_marker.pose = p
        #     self.waypoints_marker_pub.publish(self.waypoints_marker)

################### Test Case 2: Height Change in Z axis Only #################
        # for i in range(len(px)):
        #     # Get index values closest neighbors of the planned x and y values
        #     # in the KDtree
        #     closest_points_ii = tree.query_ball_point([px[i],py[i]], 0.025)
        #
        #     # If there are no closest neighbors, then continue to next iteration
        #     # of the for loop
        #     if len(closest_points_ii ) == 0:
        #         continue
        #
        #     self.z_val = []
        #     # append the z values of the point cloud of the indexed values.
        #     for j in closest_points_ii:
        #         self.z_val.append(self.cloud_z[j])
        #
        #     # Remove any z outliers of the stored data.
        #     self.remove_outliers()
        #
        #     # Find the index of the max z value from the closest neighbors point cloud
        #     index = closest_points_ii[self.z_val.index(max(self.z_val))]
        #
        #
        #     # Include characteristics of a pose
        #     p = Pose()
        #     p.position.x = self.cloud_x[index]
        #     p.position.y = self.cloud_y[index]
        #     p.position.z = self.cloud_z[index] + self.offset
        #     p.orientation.x = 0
        #     p.orientation.y = 0
        #     p.orientation.z = 0
        #     p.orientation.w = 1
        #     poses.append(p)
        #
        #     # Create new marker id and pose to be published
        #     self.waypoints_marker.id = i
        #     self.waypoints_marker.pose = p
        #     self.waypoints_marker_pub.publish(self.waypoints_marker)


################ Test Case 3: Offset distance from point normal ################
        for i in range(len(px)):
            # Get index values closest neighbors of the planned x and y values
            # in the KDtree
            closest_points_ii = tree.query_ball_point([px[i],py[i]], 0.025)

            # If there are no closest neighbors, then continue to next iteration
            # of the for loop
            if len(closest_points_ii ) == 0:
                continue

            self.z_val = []
            # append the z values of the point cloud of the indexed values.
            for j in closest_points_ii:
                self.z_val.append(self.cloud_z[j])

            # Remove any z outliers of the stored data.
            self.remove_outliers()

            # Find the index of the max z value from the closest neighbors point cloud
            index = closest_points_ii[self.z_val.index(max(self.z_val))]

            # compute the point_normal angles
            alpha = np.arccos(surf.point_normals[index][0])#-np.pi/2
            gamma = np.arccos(surf.point_normals[index][1])#+np.pi
            beta  = np.arccos(surf.point_normals[index][2])#+np.pi/2

            # Run rotation matrix of the three rotation angles
            mat = self.rotation_matrix(alpha,gamma,beta)
            # r = R.from_rotvec([[0    , 0   , alpha],
            #                    [0    , gamma, 0],
            #                    [0    , 0   , beta]   ])

            # Obtain Quaternion values from rotational matrix
            m = R.from_matrix(mat)
            q=m.as_quat()


            # project new point
            flipped_normal = [-surf.point_normals[index][0], -surf.point_normals[index][1], -surf.point_normals[index][2]]
            unit_vector = flipped_normal/np.linalg.norm(flipped_normal)
            offset = self.offset * unit_vector
            # print(unit_vector)

            # Include characteristics of a pose
            p = Pose()
            p.position.x = self.cloud_x[index] + offset[0]
            p.position.y = self.cloud_y[index] + offset[1]
            p.position.z = self.cloud_z[index] + offset[2]
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            poses.append(p)

            # Create new marker id and pose to be published
            self.waypoints_marker.id = i
            self.waypoints_marker.pose = p
            self.waypoints_marker.header.stamp = rospy.Time.now()
            self.waypoints_marker_pub.publish(self.waypoints_marker)

        # assisn poses to the PoseArray, self,waypoints.
        self.waypoints.poses = poses

        # Publish poses for computeCartesianPath
        self.waypoints_pub.publish(self.waypoints)

        # Clear out cloud data for new updated data
        del self.cloud_x[:], self.cloud_y[:], self.cloud_z[:]

    def rotation_matrix(self,alpha, gamma, beta):
        R_z = np.array([[ np.cos(alpha),-np.sin(alpha), 0],
                        [ np.sin(alpha), np.cos(alpha), 0],
                        [ 0            , 0            , 1]])

        R_y = np.array([[ np.cos(gamma), 0, -np.sin(gamma)],
                        [ 0           , 1,  0           ],
                        [ np.sin(gamma), 0,  np.cos(gamma)]])

        R_x = np.array([[ np.cos(beta), -np.sin(beta), 0],
                        [ np.sin(beta),  np.cos(beta), 0],
                        [ 0            ,  0          , 1]])

        # print(R_z)
        # print(R_y)
        # print(R_x)
        return np.linalg.multi_dot([R_z,R_y,R_x])

    def remove_outliers(self, m = 3):
            # Find z score of the z_val list
            z = np.abs(stats.zscore(self.z_val))
            # Locate the index of where z scores are larger than the m value (sigma)
            locs = np.where(z > m)
            reversed = locs[0][::-1]

            # Pop the large zscore values out.
            for ele in reversed:
                self.z_val.pop(ele)



if __name__=="__main__":
    # Initialize the node
    rospy.init_node('nonplanar_waypoint_generator')
    Nonplanar_waypoint_generator()
    rospy.spin()
