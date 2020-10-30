#!/usr/bin/env python
import rospy
import sys
import numpy as np
from visualization_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, Pose, PoseArray, Quaternion,Point
from shapely.geometry import Point as Point_2D
from shapely.geometry.polygon import Polygon
from interactive_markers.interactive_marker_server import Header
from planar import BoundingBox
from plane_fitting import Plane_fitting
from movegroupinterface import MoveGroupInteface, FollowTrajectoryClient

class Waypoint_generator:
    def __init__(self):
        rospy.init_node('waypoint_generator')
        self.best_fit_plane_sub = rospy.Subscriber('best_fit_plane', PolygonStamped, self.update_data)
        self.gui_input_sub = rospy.Subscriber('gui_input', String, self.callback )
        self.bounding_box_pub = rospy.Publisher('bounding_box',Marker, queue_size=10)
        self.waypoints_pub = rospy.Publisher('waypoints', PoseArray, queue_size = 10)
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker, queue_size = 10)

        #Initialize the "get_points" node, parameters, Subscriber, and publisher
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        self.bbox = Marker()
        self.bbox.header = self.header

        self.point_list = Marker()
        self.point_list.header = self.header

        self.solver = Plane_fitting()
        self.data = None

    def update_data(self,msg):
        self.data = msg

    def callback(self,gui_input):
        if gui_input.data == "0":
            if self.data != None:
                self.bounding_box()
            elif self.data == None:
                print('You need to select at least 3 points to generate waypoints or update one of the 3 interactive markers.')


    def bounding_box(self):
        data = self.data
        bbox_marker = self.bbox
        bbox_marker.type = Marker.LINE_STRIP
        bbox_marker.action = Marker.ADD
        bbox_marker.id = 1
        bbox_marker.scale.x = .03
        bbox_marker.scale.y = .03
        bbox_marker.scale.y = .03
        bbox_marker.color.a = 1
        bbox_marker.color.r = 1
        bbox_marker.color.g = 1
        bbox_marker.color.b = 0
        bbox_marker.pose.position.x = 0
        bbox_marker.pose.position.y = 0
        bbox_marker.pose.position.z = 0
        bbox_marker.pose.orientation.w = 1.0
        poly_points = []
        bbox_marker.points = []
        for i in range(len(data.polygon.points)):
            poly_points.append([data.polygon.points[i].x,data.polygon.points[i].y])

        poly_xy_limits = BoundingBox(poly_points)
        bbox_2D = poly_xy_limits.to_polygon()
        for j in range(len(bbox_2D)):
            z_value = self.solver.solve_for_z(bbox_2D[j][0],bbox_2D[j][1])
            bbox_marker.points.append(Point(bbox_2D[j][0],bbox_2D[j][1],z_value))
            if j == (len(bbox_2D)-1):
                z_value = self.solver.solve_for_z(bbox_2D[0][0],bbox_2D[0][1])
                bbox_marker.points.append(Point(bbox_2D[0][0],bbox_2D[0][1],z_value))
        self.bounding_box_pub.publish(bbox_marker)
        self.generate_waypoints(poly_xy_limits,Polygon(poly_points))

    def generate_waypoints(self,poly_xy_limits,poly_points):
        self.waypoints = PoseArray()
        self.waypoints.header = self.header
        dist_from_surface = 1
        margin = .1
        x_lim_min = poly_xy_limits.min_point[0] + margin
        x_lim_max = poly_xy_limits.max_point[0] - margin
        y_lim_min = poly_xy_limits.min_point[1] + margin
        y_lim_max = poly_xy_limits.max_point[1] - margin
        num_x = abs(round((x_lim_min-x_lim_max)/(margin)))
        num_y = abs(round((y_lim_min-y_lim_max)/(margin)))
        x = np.linspace(x_lim_min, x_lim_max,num = num_x)
        y = np.linspace(y_lim_max, y_lim_min,num = num_y)
        XX,YY = np.meshgrid(x,y)
        row, col = XX.shape
        list = []
        for m in range(row):
            for n in range(col):
                if poly_points.contains(Point_2D(XX[m][n],YY[m][n])) == True:
                    p = Pose()
                    p.position.x = XX[m][n]
                    p.position.y = YY[m][n]
                    p.position.z = np.float64(self.solver.solve_for_z(XX[m][n], YY[m][n])+.2)
                    list.append(Point(XX[m][n],YY[m][n],p.position.z))
                    p.orientation.w = 1.0
                    self.waypoints.poses.append(p)

        self.waypoints_pub.publish(self.waypoints)
        self.waypoint_marker(list)

    def waypoint_marker(self,list):
        marker = self.point_list
        poly_points_2D = []
        triangulation_points = []
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.id = 1
        marker.scale.x = .01
        marker.scale.y = .01
        marker.color.a = 1
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        marker.points = list
        self.waypoints_marker_pub.publish(marker)



if __name__=="__main__":
    Waypoint_generator()
    rospy.spin()




# flag = 0
# X = x_lim_min
# Y = y_lim_max
# incr = .1
# list = []
# for i in range(int(num_y)):
#     if flag == 0:
#         while True:
#             if poly_points.contains(Point_2D(X,Y)) == True:
#                 p = Pose()
#                 p.position.x = X
#                 p.position.y = Y
#                 p.position.z = np.float64(self.solver.solve_for_z(X, Y) + .2)
#                 p.orientation.w = 1.0
#                 list.append(Point(X, Y,p.position.z))
#                 self.waypoints.poses.append(p)
#             else:
#                 flag = 1
#                 Y -= incr
#                 break
#             X += incr
#
#     elif flag == 1:
#         while True:
#             print(Y)
#             if poly_points.contains(Point_2D(X,Y)) == True:
#                 p = Pose()
#                 p.position.x = X
#                 p.position.y = Y
#                 p.position.z = np.float64(self.solver.solve_for_z(X, Y) + .2)
#                 p.orientation.w = 1.0
#                 list.append(Point(X,Y,p.position.z))
#                 self.waypoints.poses.append(p)
#             else:
#                 flag = 0
#                 Y -= incr
#                 break
#             X -= incr
#
# print(i, num_y)







# print(self.waypoints.poses)
