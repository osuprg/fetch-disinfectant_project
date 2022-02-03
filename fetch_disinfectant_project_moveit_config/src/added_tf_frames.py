#!/usr/bin/env python
import roslib

import rospy
import tf
if __name__ == '__main__':
    rospy.init_node('added_tf_frames')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.55, 0.7, 0.65),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        "table_corner",
        "base_link")

        br.sendTransform((0.1, -0.3, 0.1),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        "book",
        "table_corner")

        br.sendTransform((0.15, -0.5, 0.2),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        "ramp",
        "table_corner")

        br.sendTransform((0.1, -0.7, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        "surface",
        "table_corner")




        rate.sleep()
