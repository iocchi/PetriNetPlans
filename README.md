
Petri Net Plans
===============

This repository contains Petri Net Plans library, PNP ROS bridge and some sample applications.
More information on the PNP web site: pnp.dis.uniroma1.it

This master branch contains up-to-date development of main PNP library, other utilities for plan generation, bridges with ROS and NAOqi and examples.

PNP is licensed under GPL v3.

How to install and run
======================

Please follow this order.

1. Compile and install PNP (see PNP/README.txt)

2. Compile and test pnp_ros and rp_action (see PNPros/README.md)

These packages have been tested with Ubuntu 12.04 + ROS hydro, 
Ubuntu 14.04 + ROS indigo, Ubuntu 16.04 + ROS Kinetic, Ubuntu 18.04 + ROS Melodic.

If you encounter and problem, please contact one of the maintainers
(e.g., iocchi@diag.uniroma1.it)


How to use variables in PNP
======================

In PNP, there is the possibility to instantiate parametric plans (i.e., plans that contain variables).

In PNP variables are represented with a string starting with the @ character.

For example, it is possible to define a plan GoTo_@X_@Y_@Theta where the three variables @X, @Y, and @Theta are instatiated at run time. 

In order to instatiate variables, we need to create a transition with a condition containing the variables name (e.g., "start [GoTo_@X_@Y_@Theta]" where the condition is [GoTo_@X_@Y_@Theta]). 

At this point, every time PNP receives an event of the form GoTo_instance1_instance2_instance3, it will instatiate the variables with the instances provided (in this case X=instance1, Y=instance2, Theta=instance3). 

From this point on, the global variables can be used in the plan (for example in execution nodes, such as in a node called "GoTo_@X_@Y_@Theta.exec").

Finally, to modify the variables' value just reinstantiate them as previously explained.
