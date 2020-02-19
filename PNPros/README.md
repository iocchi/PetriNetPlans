## Introduction

This package contains PNP ROS bridge.

ROS hydro/indigo/kinetic/melodic (catkin) version.


## How to compile pnp_ros and examples (rp_action)


Add links from the folders in PNPros to your catkin workspace.
For example

    catkin_ws/src$ ln -s <path_to_PNP>/PNPros/ROS_bridge/pnp_ros .
    catkin_ws/src$ ln -s <path_to_PNP>/PNPros/ROS_bridge/pnp_msgs .

and run '''catkin_make''' from your catkin workspace
to compile the ROS packages related to PNPros
and some examples.

Note: If some errors occur during build like: 
'''fatal error: pnp_msgs/Action.h: No such file or directory'''

try to build the msgs first:

    catkin_make --pkg pnp_msgs


How to test pnp_ros
===================

Follow instructions in '''PNPros/example/rp_action/README.md'''


