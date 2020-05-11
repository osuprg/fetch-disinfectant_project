#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Note: fetch_moveit_config move_group.launch must be running
# Safety!: Do NOT run this script near people or objects.
# Safety!: There is NO perception.
#          The ONLY objects the collision detection software is aware
#          of are itself & the floor.
if __name__ == '__main__':
    rospy.init_node("lawn_mower")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm", "base_link")

    # This is the wrist link not the gripper itself
    gripper_frame = 'wrist_roll_link'
    # Position and rotation of a sequence of wave points
    gripper_poses = [Pose(Point(0.55, 0.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.65, 0.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.75, 0.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.85, 0.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.85, 0.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.75, 0.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.65, 0.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.55, 0.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.55, 0.0, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.65, 0.0, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.75, 0.0, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.85, 0.0, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.85, -.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.75, -.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.65, -.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.55, -.2, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.55, -.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.65, -.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.75, -.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),]
                     Pose(Point(0.8, -.4, 0.98),Quaternion(0.000, 0.0, 0, 1)),]

    # Construct a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    rospy.sleep(2)


    for i in range(0,1):  #while not rospy.is_shutdown():
        for pose in gripper_poses:
            # Finish building the Pose_stamped message
            # If the message stamp is not current it could be ignored
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            # Set the message pose
            gripper_pose_stamped.pose = pose

            # Move gripper frame to the pose specified
            move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            result = move_group.get_move_action().get_result()

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()
