#!/bin/bash
cd /home/cc/ee106a/fa22/class/ee106a-abl/Dartboi

catkin_make
source devel/setup.bash

rosrun intera_interface enable_robot.py -e 
roslaunch intera_examples sawyer_tuck.launch &

rosrun intera_interface joint_trajectory_action_server.py &

roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true &

roslaunch tracking ar_track.launch &
python ./src/tracking/src/camera_display.py &
python ./src/tracking/src/transformation.py &
python ./src/move_arm/src/PP_move.py &

exec bash