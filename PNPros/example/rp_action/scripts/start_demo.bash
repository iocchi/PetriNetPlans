#!/bin/bash
# Use: ./start_demo.bash [multirobot]

if [ "$#" == 0 ]; then

xterm -e roslaunch rp_action dis-B1.launch &
sleep 3
xterm -e roslaunch rp_action robot.launch &
sleep 3
xterm -e roslaunch rp_action robot_plan.launch &

else

# Launch multi-robot demo

ARG_MULTIROBOT="multirobot:=true"

xterm -e roslaunch rp_action dis-B1.launch $ARG_MULTIROBOT &
sleep 3
xterm -e roslaunch rp_action robot.launch robotname:=robot_0 $ARG_MULTIROBOT &
sleep 3
xterm -e roslaunch rp_action robot_plan.launch robotname:=robot_0 $ARG_MULTIROBOT &
sleep 3
xterm -e roslaunch rp_action robot.launch robotname:=robot_1 $ARG_MULTIROBOT &
sleep 3
xterm -e roslaunch rp_action robot_plan.launch robotname:=robot_1 $ARG_MULTIROBOT &

fi

# RVIZ
#xterm -e "rosrun rviz rviz -d ../config/rviz/$ROS_DISTRO/robot_0.rviz" &


