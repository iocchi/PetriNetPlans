
This package contains PNP ROS bridge.

ROS hydro/indigo (catkin) version.


How to compile pnp_ros and examples (rp_action)
===============================================


Add a link from this folder to your catkin workspace
for example, 

  catkin_ws/src$ ln -s <path_to_PNP>/PNPros .

and run 'catkin_make' from your catkin workspace
to compile the ROS packages related to PNPros
and some examples.

Note: If some errors occur during build like: 
fatal error: pnp_msgs/Action.h: No such file or directory

try to build the msgs first:
catkin_make --pkg pnp_msgs


How to test pnp_ros
===================

Move to the PNPros/ROS_bridge/pnp_ros/ folder and run the test script

  pnp_ros$ ./testPNP.sh <planname>

Available plans are in the 'plans' directory. 
Plan name should not contain the '.pnml' suffix.

Example:

  pnp_ros$ ./testPNP.sh sequence_loop
 
You will see the plan in action according to the definition
of actions given in the actione server scripts/PNPActionServer.py
(actions are implemented with just print outputs).

To terminate the test, just close all the xterm windows,
or use 'killall -9 xterm' if you do not normally use xterm for
other purposes.


How to test rp_action
=====================

Move to the PNPros/example/rp_action folder

