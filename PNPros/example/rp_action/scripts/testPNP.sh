#!/bin/bash
xterm -e roscore &
sleep 3
xterm -e rosrun rp_action mypnpas &
sleep 3
xterm -e rosrun pnp_ros pnp_node _plan_folder:=`rospack find pnp_ros`/plans/ _current_plan:=sequence_loop &
sleep 1

