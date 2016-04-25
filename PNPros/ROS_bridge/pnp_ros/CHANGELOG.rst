^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pnp_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2016-04-25)
------------------
* made Marc Hanheide maintainer
* launch file added and plans installed
* pnp_ros added automatic timeout conditions 'timeout_actionname_value', becomes true value seconds after the start of the action
* updated pnp_ros cmakelists file
* added install targets
* added required dependencies to make the whole ws build
* LI fixed problem with variables and ROS params
* LI improved condition evaluation, using a cache
* LI Improved condition buffer with condition cache
* Merge branch 'master' of github.com:iocchi/PetriNetPlans
* rp_actions modifications
* .
* LI updated PNPgen
* Can register action functions as class members
* Timeout conditions set to 10 seconds (but it should be parametric, i.e. decided by the client who is publishing it)
* Improved switch of plans
* Merged GUI visualization of plan execution; modified PNPjarp.jar
* fixed crash bug when plan does not exist; added autorestart flag to restart a plan after the goal (default is FALSE)
* Fixed activePlaces bug when sending a new plan; added README in pnp_ros
* LI last action of old plan terminated when new plan is received from the topic
* LI added topic to run a plan on-line (NOTE: it does not interrupt the action in execution of the old plan); 'stop' is a fake plan name to stop any plan
* Fixed compilation error that required to first run the command catkin_make --pkg pnp_msgs. Now everything can be compiled directly through catkin_make
* pnp_ros: added virtual methods for easier action start, end and interrupt handling
* small changes
* better implementation of rp_action examples; some fixes in pnp_ros
* Possible pnp v1.1
* GG
* change rosbridge to use a parameter if it is connected to the gui
  default is not to be connected
* fixed some porting bugs
* realligned PNPros with the groovy version
* Add cmake target dependencies of the executable/library to ensure that
  the required messages/services/actions are generated previously.
* added sockets for visualizing in mod java gui
* Adjusted pnp_ros scripts
* sensing plan corrected
* Sensing plan adjusted
* Other updates on cmake files
* Adjustments to pnp_ros
* deleted qtcreator dirs
* added porting to hydro/indigo of the ros_bridge
* Contributors: Alexander Buchegger, Clemens Mühlbacher, Fabio Previtali, Julia Nitsch, Luca Iocchi, Marc Hanheide, corot, g-gemignani
