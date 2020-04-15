#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_python import MoveGroupInterface, PlanningSceneInterface


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('lawn_mower_path_cartesian_movement',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "arm" #or arm_with_torso
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

 
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
   

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def plan_cartesian_path(self, scale=1):
    group = self.group

    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = [Pose(Point(0.55, 0.0, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.65, 0.0, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.75, 0.0, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.85, 0.0, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.85, -.2, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.75, -.2, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.65, -.2, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.55, -.2, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.55, -.4, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.65, -.4, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.75, -.4, 0.78),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.85, -.4, 0.78),Quaternion(0.000, 0.0, 0, 1))]

    for i in range(len(waypoints) -1, -1, -1):
	waypoints.append(waypoints[i])

    #moving 
    '''wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))'''

    # This creates objects in the planning scene that mimic the ground and other objects
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("front_box")
    planning_scene.addCube("front_box", .7, .65, 0.0, 0.3)
    #planning_scene.removeCollisionObject("my_front_ground")
    #planning_scene.removeCollisionObject("my_back_ground")
    #planning_scene.removeCollisionObject("my_right_ground")
    #planning_scene.removeCollisionObject("my_left_ground")
    #planning_scene.addCube("my_front_ground", 1, -1.2, 0.0, -1.0)
    #planning_scene.addCube("my_back_ground", 1, -1.2, 0.0, -1.0)
    #planning_scene.addCube("my_left_ground", 1, -1.2, 1.2, -1.0)
    #planning_scene.addCube("my_right_ground", 1, -1.2, -1.2, -1.0)

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def execute_plan(self, plan):
    group = self.group
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
  
def main():
  try:

    path = MoveGroupPythonIntefaceTutorial()
    print "============ Press `Enter` to plan and execute the Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = path.plan_cartesian_path()
    path.execute_plan(cartesian_plan)
   
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

