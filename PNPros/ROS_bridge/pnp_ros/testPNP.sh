#!/bin/sh
xterm -e roscore &
sleep 1
xterm -e scripts/PNPActionServer.py &
sleep 1
xterm -e rosrun pnp_ros pnp_node _current_plan:=$1 &
