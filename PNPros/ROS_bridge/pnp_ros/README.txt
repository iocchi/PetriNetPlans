===================
How to test pnp_ros
===================

Just run the script

  $ ./testPNP.sh

It will start the execution of the default plan 'sensing'


Run the script with the name of a plan as an argument

  $ ./testPNP.sh interrupt
  
Make sure the plan is in the plans folder of pnp_ros and 
do not use the extension .pnml


To stop the plan, use:

  $ rostopic pub planToExec std_msgs/String "data: 'stop'" -1
  

To start another plan, use the same topic with the name of a plan

  $ rostopic pub planToExec std_msgs/String "data: 'fork_join'" -1
  

To test a single action:

  $ rostopic pub PNPActionCmd std_msgs/String "data: '<action_str> <command>'" --once

action_str = actionname_actionparams

command = start | end | interrupt

Example:

  $ rostopic pub PNPActionCmd std_msgs/String "data: 'goto_kitchen start'" --once


