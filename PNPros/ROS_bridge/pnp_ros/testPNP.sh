#!/bin/sh
xterm -e roscore &
sleep 1
xterm -e test/PNPActionServer.py &
sleep 1
xterm -e rosrun pnp_ros pnp_ros _current_plan:=$1 &
