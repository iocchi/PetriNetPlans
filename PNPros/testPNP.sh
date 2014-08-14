#!/bin/sh
xterm -e roscore &
sleep 1
xterm -e test/PNPActionServer.py &
sleep 1
xterm -e bin/pnp_ros _current_plan:=$1 &
