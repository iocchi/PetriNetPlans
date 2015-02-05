
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

Note: Install 'xterm' if the system complains about it
(sudo apt-get install xterm)


How to test rp_action
=====================

From the PNPros/example/rp_action/scripts folder, run

  $ ./run-dis-B1-plan.sh

You should see a robot moving on stage following the plan
specified in run-dis-B1-plan.sh (sensing, by default)

If it does not work, try to run ./run-dis-B1.sh
and debug your ROS setting with rviz

You can change plan on-line by using:

  $ rostopic pub /robot_0/planToExec std_msgs/String "data: '<plan_name>'" -1 

Special plan_name 'stop' is used to stop/abort the current plan.


How to visualize the execution of the plan
==========================================

From the PNPros/example/rp_action/scripts folder, run

  $ ./run-dis-B1-plan-GUI.sh

Run the PNPjarp: from the Jarp folder, run

  $ ./jarp.sh

In PNPjarp load the plan that is in execution on the robot,
plans executed in rp_action are in the folder 'PNPros/ROS_bridge/pnp_ros/plans/'.

Then push Start on the GUI and specify the host where PNP is running 
(127.0.0.1 for local host).

You will see the tokens in PNP moving according to the execution
of the plan.

Note: at this moment the Stop button does not work and closing the 
Jarp program make the plan fail. Sorry...




