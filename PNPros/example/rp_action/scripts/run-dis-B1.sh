#!/bin/bash
xterm -e roscore &
sleep 3
xterm -e roslaunch rp_action dis-B1.launch &
sleep 3
xterm -e roslaunch rp_action robot_0.launch &
sleep 3
#xterm -e roslaunch rp_action robot_1.launch &
sleep 3
#xterm -e rosrun rviz rviz -d ../config/rviz/$ROS_DISTRO/robot_0.vcg &


