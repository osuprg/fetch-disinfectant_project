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
    #group_name = "arm_with_torso" #or arm_with_torso
    group1 = moveit_commander.MoveGroupCommander("arm")
    # group2 = moveit_commander.MoveGroupCommander("arm_with_torso")

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame1 = group1.get_planning_frame()
    # planning_frame2 = group2.get_planning_frame()

    #print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link1 = group1.get_end_effector_link()
    # eef_link2 = group2.get_end_effector_link()

    #print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""


    # Misc variables
    # self.box_name = ''
    # self.robot = robot
    # self.scene = scene
    self.group1 = group1
    # self.group2 = group2
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame1 = planning_frame1
    # self.planning_frame2 = planning_frame2
    self.eef_link1 = eef_link1
    # self.eef_link2 = eef_link2
    self.group_names = group_names

  def plan_cartesian_path_table(self, scale=1):
    group1 = self.group1

    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = [Pose(Point(0.55, 0.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.80, 0.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.85, 0.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.55, 0.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.55, 0.0, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.85, 0.0, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.85, -.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.55, -.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.55, -.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.80, -.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                 Pose(Point(0.55, 0.0, 0.98),Quaternion(0.000, 0.0, 0, 1))]

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group1.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:

    return plan, fraction

  def plan_cartesian_path_book_shelf(self, scale=1):
    group1 = self.group1


    waypoints = [Pose(Point(0.55, 0.4, 1),Quaternion(0, -0.7071068, 0, 0.7071068)),
                 Pose(Point(0.55, -.4, 1),Quaternion(0, -0.7071068, 0, 0.7071068)),
                 Pose(Point(0.55, -.4,.6),Quaternion(0.0, -0.7071068, 0, 0.7071068)),
                 Pose(Point(0.55, 0.4,.6),Quaternion(0.0, -0.7071068, 0, 0.7071068)),]

    (plan, fraction) = group1.compute_cartesian_path(
                                     waypoints,   # waypoints to follow
                                     0.01,        # eef_step
                                     0.0)         # jump_threshold
    return plan, fraction

  def plan_cartesian_path_book_cabinet(self, scale=1):
    group1 = self.group1


    waypoints = [Pose(Point(0.65, 0.0, 1),Quaternion(0, -0.7071068, 0, 0.7071068)),
                 Pose(Point(0.65, 0.0,.4),Quaternion(0.0, -0.7071068, 0, 0.7071068)),]

    (plan, fraction) = group1.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    return plan, fraction

  def execute_plan(self, plan):
    group1 = self.group1

    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    print "======== Press 'Enter' to start the lawn mower path. "
    raw_input()
    group1.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


  def init_pose(self):
    group1 = self.group1

    joint_goal = group1.get_current_joint_values()
    # joint_goal[0] = 0.0
    joint_goal[0] = 0.74#-pi/4
    joint_goal[1] = -0.3
    joint_goal[2] = -2.06#-pi/2
    joint_goal[3] = 2.02
    joint_goal[4] = -0.54#pi/3
    joint_goal[5] = -1.43
    joint_goal[6] = 1.86

    print(joint_goal)
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group1.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group1.stop()

def main():
  try:

    path = MoveGroupPythonIntefaceTutorial()
    # print "============ Press `Enter` to plan and execute the Cartesian path ..."
    # raw_input()
    path.init_pose()
    cartesian_plan, fraction = path.plan_cartesian_path_table()
    path.execute_plan(cartesian_plan)
    cartesian_plan, fraction = path.plan_cartesian_path_book_shelf()
    path.execute_plan(cartesian_plan)
    path.init_pose()
    cartesian_plan, fraction = path.plan_cartesian_path_table()
    path.execute_plan(cartesian_plan)
    cartesian_plan, fraction = path.plan_cartesian_path_book_cabinet()
    path.execute_plan(cartesian_plan)
    path.init_pose()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
