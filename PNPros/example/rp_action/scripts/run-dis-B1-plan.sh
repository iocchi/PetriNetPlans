#!/bin/bash
xterm -e roscore &
sleep 3
xterm -e roslaunch rp_action dis-B1.launch &
sleep 3
xterm -e roslaunch rp_action robot_0_plan.launch &
sleep 3
#xterm -e roslaunch rp_actions robot_1.launch &
sleep 3



