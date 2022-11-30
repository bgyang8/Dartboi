#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import numpy as np
from geometry_msgs.msg import Twist, Vector3

from geometry_msgs.msg import Twist

#Define the method which contains the main functionality of the node.
def get_transform(frame1, frame2):
  """
  Controls a turtlebot whose position is denoted by frame1,
  to go to a position denoted by target_frame
  Inputs:
  - frame1: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  #Create a publisher and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
        # The transform of the target with respect to the head camera
      trans = tfBuffer.lookup_transform(frame2, frame1, rospy.Time())
      print(trans)
    #   # Process trans to get your state error
      x = trans.transform.translation.x
      y = trans.transform.translation.y
      z = trans.transform.translation.z

    # Planar distance in the x-z (horizontal) plane
      dist = np.sqrt(x**2 + z**2)

    # Want to the same y as the target
      y_to_go_to = y

    # Roll and pitch zero
      r = 0
      p = 0

    # yaw is ...
      y = ''


    #   # Generate a control command to send to the robot
    #   msg = Twist()

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

"""Get transformations between frame1 (reference) and frame2 (target)"""
def trans_node(name, frame1, frame2, anonymous=True):
    rospy.init_node(name)

    try:
        get_transform(frame1, frame2)
    except rospy.ROSInterruptException:
        pass

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
    frame1 = 'head_camera'
    frame2 = 'ar_marker_6'
    name = 'ar_transform'
    trans_node(name, frame1, frame2)
  