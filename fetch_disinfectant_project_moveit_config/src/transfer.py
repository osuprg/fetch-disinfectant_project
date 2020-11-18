#!/usr/bin/env python

# Import what we need
import rospy
import sys
import math
import numpy as np

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from scipy.spatial import ConvexHull, Delaunay
from sympy import solve, Poly, Eq, Function, exp, Symbol


class Convex_hull:
    def __init__(self):
        # Initialize Subscribers
        self.clicked_point_sub = rospy.Subscriber('clicked_point', PointStamped, self.makeInteractiveMarker, queue_size = 1)

        # Initialize Publishers
        self.convex_hull_pub = rospy.Publisher('convex_hull',PolygonStamped, queue_size=1)

        # Setup header for Interactive Markers (IM)
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Setup PolygonStamped for convex hull
        self.poly = PolygonStamped()
        self.poly.header = self.header

        # create an IM server on the topic namespace interactive_marker
        self.server = InteractiveMarkerServer("interactive_marker")

        # Initialize a lists for the projected points of each IM.
        self.proj_x = []
        self.proj_y = []
        self.proj_z = []
        
        # Initialize id of IM's.
        self.id = 0

        # Set X,Y, and Z lists for the IM positions
        self.X = []
        self.Y = []
        self.Z = []

        # Set a, b and d values to none. Equation: ax+by+cz=d
        self.a = None
        self.b = None
        self.c = None

        # Set components of unit normal vector
        self.nx = None
        self.ny = None
        self.nz = None

    # processFeedback gets called when an IM's position is changed
    def processFeedback(self,feedback):
        # update the clicked_points_3D list and recalculate the convex_hull
        p = feedback.pose.position
        id = int(feedback.marker_name)
        self.X[id] = p.x
        self.Y[id] = p.y
        self.Z[id] = p.z
        if self.id > 1:
            self.best_fit_plane(self.X,self.Y,self.Z)

    def makeInteractiveMarker(self, msg):
        # store x,y, and z point values from subscribed clicked points.
        self.X.append(msg.point.x)
        self.Y.append(msg.point.y)
        self.Z.append(msg.point.z)

        # create an IM for our server
        int_marker = InteractiveMarker()
        int_marker.header = self.header
        int_marker.scale = 0.2
        int_marker.name = str(self.id)
        int_marker.pose.position = msg.point

        # create a white sphere marker
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = int_marker.scale*0.5
        sphere_marker.scale.y = int_marker.scale*0.5
        sphere_marker.scale.z = int_marker.scale*0.5
        sphere_marker.color.r = 1
        sphere_marker.color.g = 1
        sphere_marker.color.b = 1
        sphere_marker.color.a = 1

        # create an interactive control which contains the sphere and allows the
        # marker to move in any direction when you click and drag (MOVE_3D)
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere_control.markers.append(sphere_marker)

        # add the control to the IM
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the x direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 1
        sphere_control.orientation.y = 0
        sphere_control.orientation.z = 0
        sphere_control.name = "move_x"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the z direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 0
        sphere_control.orientation.y = 1
        sphere_control.orientation.z = 0
        sphere_control.name = "move_z"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # create interactiver control to move linearly in the y direction
        sphere_control = InteractiveMarkerControl()
        sphere_control.orientation.w = 1
        sphere_control.orientation.x = 0
        sphere_control.orientation.y = 0
        sphere_control.orientation.z = 1
        sphere_control.name = "move_y"
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(sphere_control)

        # Add the IM to our collection & tell the server to call processFeedback()
        # when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

        # Run a convex hull once there are 3 or more clicked points
        if self.id > 1:
            self.best_fit_plane(self.X,self.Y,self.Z)


        # update the IM id for next clicked point.
        self.id += 1

    def best_fit_plane(self,X,Y,Z):
        # coordinates (XYZ) of the interactive_markers
        XYZ = np.array([X,Y,Z])

        # Begin plane fit computation. Reference:https://stackoverflow.com/questions/12299540/
        tmp_A = []
        tmp_b = []
        for i in range(len(X)):
            tmp_A.append([X[i], Y[i], 1])
            tmp_b.append(Z[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit

        # Equation: fit[0]*x + fit[1]*y + fit[2] = z
        # assign a,b, and c values by rearranging equation.
        # Rearranged Eq. : (-fit[0]/fit[2])*x + (-fit[1]/fit[2])*y + (1/fit[2])*z = 1
        self.a = float(-fit[0]/fit[2])
        self.b = float(-fit[1]/fit[2])
        self.c = float(1/fit[2])
        #print("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
        #print("Normal vector: < {0}, {1}, {2}> ".format(self.a,self.b,self.c))

        self.project_on_plane()

    def project_on_plane(self):
        # Create an origin point by using x and y pose of the first IM and solve for z.
        orig_x = self.X[0]
        orig_y = self.Y[0]
        orig_z = self.solve_for_z(orig_x, orig_y)

        # Create unit normal vector
        mag = math.sqrt(self.a**2 + self.b**2 + self.c**2)
        self.nx = self.a / mag
        self.ny = self.b / mag
        self.nz = self.c / mag
        n = np.array([self.nx,self.ny,self.nz])

        # Create a list of the projected points
        # proj_points = []

        # forloop to compute the projections of each IM
        for i in range(len(self.X)):
            # Make a vector from the origin points to the IM's location.
            # v = Interactive_maker_location - orig (with respect to each dimension)
            vx = self.X[i] - orig_x
            vy = self.Y[i] - orig_y
            vz = self.Z[i] - orig_z
            v  = np.array([vx,vy,vz])

            # Take dot product of vector <vx,vy,vz> and unit normal vector <nx,ny,nz>
            # to get the scalar distance from the IM to the plane along the normal.
            dist =np.matmul(v,n)

            # Multiply the unit normal vector by the dist, and subtract from the IM point.
            # Append to appropriate lists.
            self.proj_x = self.X[i] - dist*self.nx
            self.proj_y = self.Y[i] - dist*self.ny
            self.proj_z = self.Z[i] - dist*self.nz



            # print('')
            # print(self.X[0],self.Y[0], self.Z[0])
            # print(proj_points)
            # print('')

        # self.transfer_matrix(proj_points, )

    def transfer_matrix(self):
        #s = np.array([[1,2,1,1], [1,1,1,2], [1,1,2,1], [1,1,1,1]])
        # ss = np.linalg.inv(s)
        #d = np.array([[0,1,0,0],[0,0,1,0], [0,0,0,1], [1,1,1,1]])
        #m = np.matmul(d,ss)
        #A = np.array([1,1,1,1])
        #

        return 0


    def solve_for_z(self,x_value,y_value):
        z = Symbol('z')
        z_value = solve(self.a*x_value + self.b*y_value + self.c*z - 1, z)
        return float(z_value[0])






if __name__=="__main__":
    # Initialize the node
    rospy.init_node('convex_hull')
    Convex_hull()
    rospy.spin()
