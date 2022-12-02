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

from path_planner import PathPlanner

try:
    from controller import Controller
except ImportError:
    pass
    
def moveArm(target_twist):
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

    controller = Controller(Kp, Kd, Ki, Kw, Limb("right"))

    print(target_twist.linear)
    print(target_twist.angular)
    pos_xyz = [target_twist.linear.x, target_twist.linear.y, target_twist.linear.z]
    ori_xyzw = get_quaternion_from_euler(target_twist.angular.x, target_twist.angular.y, target_twist.angular.z)

    # 
    # Add the obstacle to the planning scene here
    # obs = PoseStamped()
    # obs.header.frame_id = "base"

    # #x, y, and z position
    # obs.pose.position.x = 0.5
    # obs.pose.position.y = 0.0
    # obs.pose.position.z = 0.0

    # #Orientation as a quaternion
    # obs.pose.orientation.x = 0.0
    # obs.pose.orientation.y = 0.0
    # obs.pose.orientation.z = 0.0
    # obs.pose.orientation.w = 1.0
    # planner.add_box_obstacle(np.array([0.4,1.2,0.1]), "aero_andrew", obs)

    # obs2 = PoseStamped()
    # obs2.header.frame_id = "base"

    # #x, y, and z position
    # obs2.pose.position.x = -0.25
    # obs2.pose.position.y = 0.0
    # obs2.pose.position.z = 0.0

    # #Orientation as a quaternion
    # obs2.pose.orientation.x = 0.0
    # obs2.pose.orientation.y = 0.0
    # obs2.pose.orientation.z = 0.0
    # obs2.pose.orientation.w = 1.0
    # planner.add_box_obstacle(np.array([0.1,1.2,1.2]), "big_bryan", obs2)


    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper_tip";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.5;
    orien_const.absolute_y_axis_tolerance = 0.5;
    orien_const.absolute_z_axis_tolerance = 0.5;
    orien_const.weight = 1.0;

    user_input = 'n'

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "base"
                
                #x, y, and z position
                goal.pose.position.x = pos_xyz[0]
                goal.pose.position.y = pos_xyz[1]
                goal.pose.position.z = pos_xyz[2]

                # #Orientation as a quaternion
                goal.pose.orientation.x = ori_xyzw[0]
                goal.pose.orientation.y = ori_xyzw[1]
                goal.pose.orientation.z = ori_xyzw[2]
                goal.pose.orientation.w = ori_xyzw[3]

                # goal.pose.orientation.x = 0.0
                # goal.pose.orientation.y = 1.0
                # goal.pose.orientation.z = 0.0
                # goal.pose.orientation.w = 0.0


                # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal, [orien_const])
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
                if user_input == 'y':
                    if not controller.execute_plan(plan[1]): 
                        raise Exception("Execution failed")
            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break


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
