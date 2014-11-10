#!/bin/bash
xterm -e roscore &
sleep 3
xterm -e roslaunch rp_action dis-B1.launch &
sleep 3
xterm -e roslaunch rp_action robot.launch robotname:=robot_0 &
sleep 3
xterm -e roslaunch rp_action robot_plan.launch robotname:=robot_0 planname:=interrupt &
sleep 3
