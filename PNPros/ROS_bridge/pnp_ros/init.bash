#!/bin/bash

SESSION=pnpinit

tmux -2 new-session -d -s $SESSION

tmux rename-window -t $SESSION:0 'pnp_ros'
tmux send-keys -t $SESSION:0 "rosrun pnp_ros pnp_node name:=pnp_ros" C-m

tmux new-window -t $SESSION:1 -n 'pnp_AS'
tmux send-keys -t $SESSION:1 "rosrun pnp_ros pnp_as" C-m


while [ 1 ]; do
  sleep 10
done
