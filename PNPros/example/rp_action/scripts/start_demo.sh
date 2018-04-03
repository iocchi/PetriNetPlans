#!/bin/bash
xterm -e roslaunch rp_action dis-B1.launch &
sleep 3
xterm -e roslaunch rp_action robot.launch robotname:=robot_0 &
sleep 3
xterm -e roslaunch rp_action robot_plan.launch robotname:=robot_0 &
# use_gui:=true 

# Launch second robot
#xterm -e roslaunch rp_action robot_1.launch &

# RVIZ
#xterm -e "rosrun rviz rviz -d ../config/rviz/$ROS_DISTRO/robot_0.rviz" &


