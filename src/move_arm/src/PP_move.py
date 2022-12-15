#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Twist, Vector3


from intera_interface import gripper as robot_gripper

from path_planner import PathPlanner

try:
    from controller import Controller
except ImportError:
    pass
    
def moveArm(target_twist):

    print("callback called back")
    right_gripper = robot_gripper.Gripper('right_gripper')
    planner = PathPlanner("right_arm")

    Kp = 0.9 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.05 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.05 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    controller = Controller(Kp, Kd, Ki, Kw, Limb("right"))

    print(target_twist)
    pos_xyz = [target_twist.linear.x, target_twist.linear.y, target_twist.linear.z]
    ori_xyzw = get_quaternion_from_euler(target_twist.angular.x, target_twist.angular.y, target_twist.angular.z)

    print("angle to shoot from")
    print(target_twist.angular.y)

    # Add the obstacle to the planning scene here
    obs = PoseStamped()
    obs.header.frame_id = "base"

    #x, y, and z position
    obs.pose.position.x = 0.5
    obs.pose.position.y = 0.0
    obs.pose.position.z = -0.3

    #Orientation as a quaternion
    obs.pose.orientation.x = 0.0
    obs.pose.orientation.y = 0.0
    obs.pose.orientation.z = 0.0
    obs.pose.orientation.w = 1.0
    planner.add_box_obstacle(np.array([0.4,1.2,0.2]), "aero_andrew", obs)

    obs2 = PoseStamped()
    obs2.header.frame_id = "base"

    #x, y, and z position
    obs2.pose.position.x = -0.25
    obs2.pose.position.y = 0.0
    obs2.pose.position.z = 0.0

    #Orientation as a quaternion
    obs2.pose.orientation.x = 0.0
    obs2.pose.orientation.y = 0.0
    obs2.pose.orientation.z = 0.0
    obs2.pose.orientation.w = 1.0
    # planner.add_box_obstacle(np.array([0.1,1.2,1.2]), "big_bryan", obs2)

    user_input = 'n'

    if user_input == "n":
        return

    while not rospy.is_shutdown():
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"
            
            #x, y, and z position
            goal.pose.position.x = pos_xyz[0]
            goal.pose.position.y = pos_xyz[1]
            goal.pose.position.z = pos_xyz[2]

            goal.pose.orientation.x = ori_xyzw[0] + 0.000001
            goal.pose.orientation.y = ori_xyzw[1] + 0.000001
            goal.pose.orientation.z = ori_xyzw[2] + 0.000001
            goal.pose.orientation.w = ori_xyzw[3] + 0.000001

            plan = planner.plan_to_pose(goal, [])
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            if user_input == 'y':
                if not controller.execute_plan(plan[1]): 
                    raise Exception("Execution failed")

                user_input = "n"
                user_input = input("Enter 'y' if dart is ready to shoot")
                if user_input == 'y':
                    # open the right gripper
                    print('Opening and shooting')
                    right_gripper.open()
                    rospy.sleep(2.0)
                else:
                    user_input = "n"

            elif user_input == 'q':
                user_input = "n"
                break

        except Exception as e:
            print(e)
            traceback.print_exc()
        else:
            break

def listener():
    rospy.Subscriber('launch_des', Twist, moveArm)
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
