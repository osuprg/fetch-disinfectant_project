#!/usr/bin/env python

import sys
import actionlib
import rospy
import numpy as np
import tf

from std_msgs.msg import String

class TF_distance_calculator(object):

  def __init__(self):
    # Initialize Subscribers
    self.gui_input_sub = rospy.Subscriber('gui_input', String, self.interface_callback)

    # Initialize transform listener
    self.listener = tf.TransformListener()

  def interface_callback(self,gui_input):
      # Conditional statement that runs the distance_calculator function
      if gui_input.data == "1":
          self.distance_calculator()


  def distance_calculator(self):
    rate = rospy.Rate(10.0)
    start = rospy.get_time()
    book_prev_mag = 10
    ramp_prev_mag = 10
    surface_prev_mag = 10

    while not rospy.is_shutdown():
        try:
            (trans,rot) = self.listener.lookupTransform('/ee_link', '/book', rospy.Time(0))
            book_mag = np.linalg.norm(trans)
            if np.linalg.norm(trans) < book_prev_mag:
                book_prev_mag = book_mag

            (trans,rot) = self.listener.lookupTransform('/ee_link', '/ramp', rospy.Time(0))
            ramp_mag = np.linalg.norm(trans)
            if np.linalg.norm(trans) < ramp_prev_mag:
                ramp_prev_mag = ramp_mag

            (trans,rot) = self.listener.lookupTransform('/ee_link', '/surface', rospy.Time(0))
            surface_mag = np.linalg.norm(trans)
            if np.linalg.norm(trans) < surface_prev_mag:
                surface_prev_mag = surface_mag


            if (rospy.get_time() - start) > 10:
                print(book_prev_mag, ramp_prev_mag, surface_prev_mag)
                break

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

if __name__=="__main__":
    rospy.init_node('tf_distance_calculator',anonymous=True)
    TF_distance_calculator()
    rospy.spin()
