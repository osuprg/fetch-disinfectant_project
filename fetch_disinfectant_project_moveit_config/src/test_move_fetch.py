#!/usr/bin/env python3
from __future__ import print_function
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

'''JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
               'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
#Q1 = [ 1.32,1.40,-.20,1.70,0.00,1.66,0.00]
#Q2 = [ 2.40,1.40,-.20,1.70,0.00,1.66,0.00]
Q1 = [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]
Q2 = [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]
Q3 = [0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0]
Q4 = [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0]
Q5 = [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0]
Q6 = [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]
Q7 = [0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]'''


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint',
               'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'] #'torso_lift_joint'

#Q1 = [0.0, .74, -0.3, -2.06, 2.02, -0.54, -1.43, 1.86]
Q1 = [ 1.60, 0.3, -3.10, 2.25, -1.56, -1.52, 2.77]
#Q2 = [0.0,0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]
#Q3 = [0.1,0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0]
#Q4 = [1.0,-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0]
#Q5 = [1.0,-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0]
#Q6 = [1.0,0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]
#Q7 = [1.0, 0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]


client = None

def move_init():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0], time_from_start=rospy.Duration(5.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    d = 5.0
    g.trajectory.points = []
    for i in range(3):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0], time_from_start=rospy.Duration(d)))
        d += 8
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0], time_from_start=rospy.Duration(d)))
        d += 8
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0], time_from_start=rospy.Duration(d)))
        d += 8
	g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q4, velocities=[0], time_from_start=rospy.Duration(d)))
        d += 8
	g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q5, velocities=[0], time_from_start=rospy.Duration(d)))
        d += 8
	g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q6, velocities=[0], time_from_start=rospy.Duration(d)))
	d += 8
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q7, velocities=[0], time_from_start=rospy.Duration(d)))
        d += 8
	

    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=False, disable_signals=False)
        #client = actionlib.SimpleActionClient('/arm_with_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        raw_input('Move to initial position')
	move_init()
	#raw_input('Run sequence')
        #qqmove_repeated()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
