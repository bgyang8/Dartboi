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

def get_transform(frame1, frame2, traj_dist):
    """
    Controls a turtlebot whose position is denoted by frame1,
    to go to a position denoted by target_frame
    Inputs:
    - frame1: the tf frame of the AR tag on your turtlebot
    - frame2: the tf frame of the target AR tag
    """
  
    #Create a publisher and a tf buffer, which is primed with a tf listener
    pub = rospy.Publisher('launch_des', Twist, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    
    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    r = rospy.Rate(100) # 100hz
    
    num_samples = 750
    i = 0
    x_ref = 0
    y_ref = 0
    z_ref = 0
    while (not rospy.is_shutdown()) and (i < num_samples):
        try:
            trans = tfBuffer.lookup_transform(frame2, frame1, rospy.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            x_ref += x / num_samples
            y_ref += y / num_samples
            z_ref += z / num_samples
            
            if ~ (num_samples % 50):
                print(num_samples)

            i += 1
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
        
    epsilon = 0.1 
    r = rospy.Rate(10) # 10hz

    # Loop until the node is killed with Ctrl-C, traj_dist
    while not rospy.is_shutdown():
        try:
            # The transform of the target with respect to the head camera
            trans = tfBuffer.lookup_transform(frame2, frame1, rospy.Time())
            trans_ref = tfBuffer.lookup_transform('head_camera', 'base', rospy.Time())

            quat_ref = trans_ref.transform.rotation

            rref, pref, yref = quat_to_rpy(quat_ref)
            
            # Process trans to get your state error
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            within_eps = (abs(x_ref - x) < epsilon) \
                     and (abs(y_ref - y) < epsilon) \
                     and (abs(z_ref - z) < epsilon)
            
            if not within_eps:
                continue
            else:
                x_ref = x
                y_ref = y
                z_ref = z

            quat = trans.transform.rotation
    
            roll, pitch, yaw = quat_to_rpy(quat)
        
            # Testing direction
            # print('R: ', roll / np.pi * 180)
            # print('P: ', pitch / np.pi * 180)
            # print('Y: ', yaw / np.pi * 180)
    
            # Experimentally, PITCH is the horizontal "yaw" of the target
            # Flat is 0 deg, ccw is positive, cw is negative
            theta = roll
            
            print([x,y,z,theta])
            #print([roll, pitch, yaw])
            #print([rref, pref, yref])
            
            x_d = z - traj_dist #* np.cos(theta)
            y_d = x # - traj_dist * np.sin(theta)
            z_d = y

            ### OLD wrong coord transform ###
            # x_d = x - traj_dist * np.cos(theta)
            # y_d = y
            # z_d = z - traj_dist * np.sin(theta)
    
            r_des = 0
            p_des = 0
            y_des = theta
    
            # Generate a control command to send to the robot
            launch_twist = Twist()
            launch_twist.linear.x = x_d
            launch_twist.linear.y = y_d
            launch_twist.linear.z = z_d
            launch_twist.angular.x = r_des
            launch_twist.angular.y = p_des
            launch_twist.angular.z = y_des
            
            pub.publish(launch_twist)
            print(launch_twist)
  
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
  
  
""  "Get rpy from Quaternion"""
def quat_to_rpy(q):
    rpy = [0, 0, 0]

    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    rpy[0] = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    if np.abs(sinp) >= 1:
        rpy[1] = (np.pi / 2) * np.sign(sinp)
    else:
        rpy[1]= np.arcsin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    rpy[2] = np.arctan2(siny_cosp, cosy_cosp)

    return rpy


"""Get transformations between frame1 (reference) and frame2 (target)"""
def trans_node(name, frame1, frame2, traj_dist, anonymous=True):
    rospy.init_node(name)

    try:
        get_transform(frame1, frame2, traj_dist)
    except rospy.ROSInterruptException:
        pass

      
if __name__ == '__main__':
    # Check if the node has received a signal to shut down
    # If not, run the talker method
  
    #Run this program as a new node in the ROS computation graph 
    #called /turtlebot_controller.
    frame1 = 'base' #'head_camera'
    frame2 = 'ar_marker_0'
    traj_dist = 1  # [meters]
    name = 'ar_transform'
    trans_node(name, frame1, frame2, traj_dist)
  