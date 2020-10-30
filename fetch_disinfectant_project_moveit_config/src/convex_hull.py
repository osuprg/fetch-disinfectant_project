#!/usr/bin/env python

# Import what we need
import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from scipy.spatial import ConvexHull, Delaunay

class Convex_hull:
    def __init__(self):
        # Initialize Subscribers
        self.sub = rospy.Subscriber('clicked_point', PointStamped, self.makeInteractiveMarker)

        self.convex_hull_pub = rospy.Publisher('convex_hull',PolygonStamped, queue_size=1)
        # self.triangle_list = rospy.Publisher('convex_hull_triangle_list', Marker, queue_size = 10)

        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        self.poly = PolygonStamped()
        self.poly.header = self.header

        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("interactive_marker")

        # self.triangulation_points = []
        self.clicked_points_3D = []
        self.id = 0



    def processFeedback(self,feedback):
        p = feedback.pose.position
        id = int(feedback.marker_name)
        self.clicked_points_3D[id][0] = p.x
        self.clicked_points_3D[id][1] = p.y
        self.clicked_points_3D[id][2] = p.z
        if len(self.clicked_points_3D) > 2:
            self.convex_hull_function()
        # elif len(self.clicked_points) > 3:
        #     self.server.clear()
        #     self.server.applyChanges()
        #     self.poly.polygon.points = []
        #     self.convex_hull_pub.publish(self.poly)



    def makeInteractiveMarker(self, msg):
        # s
        self.clicked_points_3D.append([msg.point.x,msg.point.y, msg.point.z])
        # print(len(self.convex_points_2D))
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.scale = 0.2
        int_marker.name = str(self.id)
        int_marker.pose.position = msg.point

        # create a grey box marker
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = int_marker.scale*0.45
        sphere_marker.scale.y = int_marker.scale*0.45
        sphere_marker.scale.z = int_marker.scale*0.45
        sphere_marker.color.r = 1
        sphere_marker.color.g = 1
        sphere_marker.color.b = 1
        sphere_marker.color.a = 1.0

        # create a interactive control which contains the box
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere_control.markers.append(sphere_marker)

        # add the control to the interactive marker
        int_marker.controls.append(sphere_control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()
        self.id += 1

        if len(self.clicked_points_3D) > 2:
            self.convex_hull_function()
            # self.server.clear()
            # self.server.applyChanges()


    def convex_hull_function(self):
        points_2D = []
        convex_points = []
        for i in range(len(self.clicked_points_3D)):
            points_2D.append([self.clicked_points_3D[i][0], self.clicked_points_3D[i][1]])

        hull = ConvexHull(points_2D)
        for e in hull.vertices:
            convex_points.append(Point32(points_2D[e][0],points_2D[e][1],self.clicked_points_3D[e][2]))

        self.poly.polygon.points = convex_points
        self.convex_hull_pub.publish(self.poly)

if __name__=="__main__":
    #Initialize the node
    rospy.init_node('convex_hull')

    Convex_hull()
    rospy.spin()




# self.triangulation_polygon()

#hit the plane to plan a path to do a trajectory fo the end effecot then execute that.
#do it on the robot.
#how to evaluate it.

# def triangulation_polygon(self):
#     poly = self.poly.polygon.points
#     poly_points_2D = []
#     triangulation_points = []
#     marker = Marker()
#     marker.header.frame_id = "/base_link"
#     marker.type = Marker.TRIANGLE_LIST
#     marker.action = Marker.ADD
#     marker.id = 1
#     marker.scale.x = 1
#     marker.scale.y = 1
#     marker.scale.z = 1
#     marker.color.a = .2
#     marker.color.r = 1.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
#     marker.pose.position.x = 0
#     marker.pose.position.y = 0
#     marker.pose.position.z = 0
#     marker.pose.orientation.w = 1.0
#
#     for i in range(len(poly)):
#         poly_points_2D.append([poly[i].x,poly[i].y])
#
#     tri = Delaunay(poly_points_2D)
#     for j in range(len(tri.vertices)):
#         for e in tri.vertices[j]:
#             triangulation_points.append(Point(poly_points_2D[e][0],poly_points_2D[e][1], poly[e].z))
#
#     marker.points = triangulation_points
#     self.triangle_list.publish(marker)
