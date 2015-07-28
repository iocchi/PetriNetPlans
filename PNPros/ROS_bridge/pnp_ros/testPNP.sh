#!/bin/bash

PLAN="sensing"

if [ -n "$1" ]; then PLAN="$1"; fi

echo "Executing plan $PLAN"

xterm -e roscore &
sleep 3
xterm -e scripts/PNPActionServer.py &
sleep 3
xterm -e rosrun pnp_ros pnp_node _current_plan:=$PLAN &
sleep 1
xterm -e rostopic echo /pnp_ros/currentActivePlaces &


