#!/usr/bin/env python3

import rospy
import sys
import numpy as np
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from interactive_markers.interactive_marker_server import Header
from scipy.optimize import leastsq
from sympy import solve, Poly, Eq, Function, exp, Symbol
from scipy.spatial import ConvexHull, Delaunay
# import shapely.geometry as geometry
from planar import BoundingBox

class Plane_fitting:
    def __init__(self):
        self.convex_hull_sub = rospy.Subscriber('convex_hull', PolygonStamped, self.polygon_callback, queue_size=1 )
        self.best_fit_plane_pub = rospy.Publisher('best_fit_plane',PolygonStamped, queue_size=10)
        self.best_fit_plane_triangle_list_pub = rospy.Publisher('best_fit_plane_triangle_list', Marker, queue_size = 10)
        # self.bounding_box = rospy.Publisher('bouding_box', Marker, queue_size = 10)
        #Initialize the "get_points" node, parameters, Subscriber, and publisher

        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        self.poly = PolygonStamped()
        self.poly.header = self.header

        self.bbox = Marker()
        self.bbox.header = self.header

        self.plane_marker = Marker()
        self.plane_marker.header = self.header

        self.a = None
        self.b = None
        self.d = None

    def polygon_callback(self,msg):
        self.best_fit_equation(msg)

    def best_fit_equation(self,msg):
        X = []
        Y = []
        Z = []
        for i in range(len(msg.polygon.points)):
            X.append(msg.polygon.points[i].x)
            Y.append(msg.polygon.points[i].y)
            Z.append(msg.polygon.points[i].z)
        # coordinates (XYZ) of the interactive_markers
        XYZ = np.array([X,Y,Z])
        tmp_A = []
        tmp_b = []
        for i in range(len(X)):
            tmp_A.append([X[i], Y[i], 1])
            tmp_b.append(Z[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit
        residual = np.linalg.norm(errors)
        # print "solution:"
        # print "%f x + %f y + %f = z" % (fit[0], fit[1], fit[2])
        # print "errors:"
        # print errors
        # print "residual:"
        # print residual
        self.a = float(fit[0])
        self.b = float(fit[1])
        self.d = float(fit[2])
        fitted_plane = []
        for i in range(len(X)):
                z_value = self.solve_for_z(X[i],Y[i])
                fitted_plane.append(Point32(X[i],Y[i],z_value))
        self.poly.polygon.points = fitted_plane
        self.best_fit_plane_pub.publish(self.poly)
        self.triangulation_polygon()


    def solve_for_z(self,x_value,y_value):
        z = Symbol('z')
        z_value = solve(self.a*x_value + self.b*y_value - z + self.d, z)
        return float(z_value[0])

    def triangulation_polygon(self):
        poly = self.poly.polygon.points
        marker = self.plane_marker
        poly_points_2D = []
        triangulation_points = []
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.id = 1
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = .8
        marker.color.r = 1.0
        marker.color.g = 0
        marker.color.b = 0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        for i in range(len(poly)):
            poly_points_2D.append([poly[i].x,poly[i].y])

        tri = Delaunay(poly_points_2D)
        for j in range(len(tri.vertices)):
            for e in tri.vertices[j]:
                triangulation_points.append(Point(poly_points_2D[e][0],poly_points_2D[e][1], poly[e].z))

        marker.points = triangulation_points
        self.best_fit_plane_triangle_list_pub.publish(marker)


def f_min(X,p):
    plane_xyz = p[0:3]
    distance = (plane_xyz*X.T).sum(axis=1) + p[3]
    return distance / np.linalg.norm(plane_xyz)

def residuals(params, signal, X):
    return f_min(X, params)


if __name__=="__main__":
    rospy.init_node('plane_fitting')
    Plane_fitting()
    rospy.spin()
