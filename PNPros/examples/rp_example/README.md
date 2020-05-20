# rp_action examples

This folder contains a full example of PNP application using ROS actions.

## Link and compile rp_action packages

Note: this step is not needed if you are using the PNP docker image

Make sure PNP and PNPros have been installed and build ROS packages in this folder.

Example:

        cd ~/ros/catkin_ws/src
        ln -s $HOME/src/PetriNetPlans/PNPros/examples/rp_example/rp_action . 
        ln -s $HOME/src/PetriNetPlans/PNPros/examples/rp_example/rp_action_msgs . 
        ln -s $HOME/src/PetriNetPlans/PNPros/examples/rp_example/rp_pnp . 
        ln -s $HOME/src/PetriNetPlans/PNPros/examples/rp_example/rp_demo .
        cd ..
        catkin_make


## Single robot execution

Start the demo

        roscd rp_demo/scripts
        ./start_demo.bash 

Run a plan

        roscd rp_pnp/scripts
        ./runplan.bash sequence_loop

Other plans to test available in `rp_pnp/plans` folder.

Stop the current plan

        roscd rp_pnp/scripts
        ./runplan.bash stop


## Multi-robot execution

NOTE: Multi-robot configuration not working yet on ROS Melodic (Ubuntu 18.04)!

Start the demo

        roscd rp_demo/scripts
        ./start_demo.bash  multirobot

Run a plan for each robot

        roscd rp_pnp/scripts
        ./runplan.bash robot_0 sequence_loop
        ./runplan.bash robot_1 sequence_loop

Stop the current plan for each robot

        roscd rp_pnp/scripts
        ./runplan.bash robot_0 stop
        ./runplan.bash robot_1 stop


## Plans available


Plans available in `rp_pnp\plans` folder: `sequence_loop`, `sensing`, `interrupt`, `fork_join`.

Notes: 1) the `obstacle` condition can be activated by placing (drag with mouse) the red box in the simulated environment in front of the robot; 2) the `wave` action outputs some text on the screen.


## Quit the simulation

        rosnode kill -a


## Implementation details

The devlopment of a PNP-based application requires two components:
* domain-independent plan execution monitor `pnp_node` ROS node
* a plan to be executed `<plan>.pnml`
* a domain-specific PNP Action Server implementing actions and conditions 
that will be used by the PNP plan execution monitor

The PNP Action Server is a ROS node inheriting from the domain-independent
class `PNPActionServer` defined in `PNPros` package 
([see PNPActionServer.h](/PNPros/ROS_bridge/pnp_ros/include/pnp_ros/PNPActionServer.h)).

Example:
        
        // definition of action functions

        void action_fn(string params, bool *run) {
          ...
          while ( ... && (*run)) {
            ...
          }
        }

        // definition of condition functions

        int condition_fn(string params) {
          int r = ...; // 0: false, 1: true, -1: unknown
          return r;
        }
        

        // PNP Action server

        class MyPNPActionServer : public PNPActionServer {

          public:

            MyPNPActionServer() : PNPActionServer()  { 
	        
              register_action("action",&action_fn);
              ...
              register_condition("condition",&condition_fn);
              ...
            }

        };





