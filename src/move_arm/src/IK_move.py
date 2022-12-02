#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
from geometry_msgs.msg import Twist, Vector3


def moveArm(target_twist):

    print(target_twist.linear)
    print(target_twist.angular)
    pos_xyz = [target_twist.linear.x, target_twist.linear.y, target_twist.linear.z]
    ori_xyzw = get_quaternion_from_euler(target_twist.angular.x, target_twist.angular.y, target_twist.angular.z)

    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    #rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    ############################# LEGACY #############################
    # # Calibrate the gripper (other commands won't work unless you do this first)
    # print('Calibrating...')
    # right_gripper.calibrate()
    # rospy.sleep(2.0)

    # # Open the right gripper
    # print('Open that gates!!!')
    # right_gripper.open()
    # rospy.sleep(1.0)service_query

    user_input = 'n'

    # while not rospy.is_shutdown():
    #input('Press [ Enter ]: ')

    # Construct the start position request
    start_request = GetPositionIKRequest()
    start_request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    start_request.ik_request.ik_link_name = link
    # start_request.ik_request.attempts = 20
    start_request.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    start_request.ik_request.pose_stamped.pose.position.x = pos_xyz[0]
    start_request.ik_request.pose_stamped.pose.position.y = pos_xyz[1]
    start_request.ik_request.pose_stamped.pose.position.z = pos_xyz[2]  
    start_request.ik_request.pose_stamped.pose.orientation.x = ori_xyzw[0]
    start_request.ik_request.pose_stamped.pose.orientation.y = ori_xyzw[1]
    start_request.ik_request.pose_stamped.pose.orientation.z = ori_xyzw[2]
    start_request.ik_request.pose_stamped.pose.orientation.w = ori_xyzw[3]

    # start_request.ik_request.pose_stamped.pose.position.x = 0.5
    # start_request.ik_request.pose_stamped.pose.position.y = 0.5
    # start_request.ik_request.pose_stamped.pose.position.z = 0.0        
    # start_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    # start_request.ik_request.pose_stamped.pose.orientation.y = 1.0
    # start_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    # start_request.ik_request.pose_stamped.pose.orientation.w = 0.0

    
    try: 
        # Send the request to the service
        response = compute_ik(start_request)
        
        # Print the response HERE
        print(response)
        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(start_request.ik_request.pose_stamped)

        # Setting just the position without specifying the orientation
        # group.set_position_target(pos_xyz)

        # Plan IK and execute
        plan = group.plan()
        user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
        
        # Execute IK if safe
        if user_input == 'y':
            group.execute(plan[1])
        
    except rospy.ServiceException as e:
        print("Start Position Service call failed: %s"%e)


    #### launcher code for later

    # user_input = "n"
    # while user_input != 'y':
    #     user_input = input("Enter 'y' if dart is ready to shoot")

    #     if user_input == 'y':
    #         # Close the right gripper
    #         print('Closing :(')
    #         right_gripper.close()
    #         rospy.sleep(2.0)

    user_input = "n"


def listener():
    rospy.Subscriber('launch_des', Twist, moveArm)
    # rospy.Subscriber('launch_des', Twist, callback)
    rospy.spin()

def callback(message):
    print(message)

def get_quaternion_from_euler(r, p, y):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param r: The roll (rotation around x-axis) angle in radians.
    :param p: The pitch (rotation around y-axis) angle in radians.
    :param y: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(r/2) * np.cos(p/2) * np.cos(y/2) - np.cos(r/2) * np.sin(p/2) * np.sin(y/2)
  qy = np.cos(r/2) * np.sin(p/2) * np.cos(y/2) + np.sin(r/2) * np.cos(p/2) * np.sin(y/2)
  qz = np.cos(r/2) * np.cos(p/2) * np.sin(y/2) - np.sin(r/2) * np.sin(p/2) * np.cos(y/2)
  qw = np.cos(r/2) * np.cos(p/2) * np.cos(y/2) + np.sin(r/2) * np.sin(p/2) * np.sin(y/2)
 
  return [qx, qy, qz, qw]

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    listener()

