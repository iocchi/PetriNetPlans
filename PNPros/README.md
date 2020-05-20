# Introduction

This package contains PNP-ROS bridge.

ROS hydro/indigo/kinetic/melodic (catkin) version.

## Compiling 

Note: not needed if you are using the docker image.

Add links from the folders in PNPros to your catkin workspace.
For example

    ros/catkin_ws/src$ ln -s <path_to_PNP>/PNPros/ROS_bridge/pnp_ros .
    ros/catkin_ws/src$ ln -s <path_to_PNP>/PNPros/ROS_bridge/pnp_msgs .

and run `catkin_make` from your catkin workspace
to compile the ROS packages related to PNPros.

## Running examples

See  [rp_example](/PNPros/examples/rp_example)

