#!/bin/bash
xterm -e roslaunch rosplan_demos pnp_conditional.launch &
sleep 3
xterm -e rosrun rosplan_to_pnp rosplan_to_pnp_node &
sleep 3
xterm -e rosservice call /kcl_rosplan/planning_server &
