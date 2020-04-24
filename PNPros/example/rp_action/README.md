# rp_action examples

This folder contains a full example of PNP application using ROS actions.

## Link and compile rp_action package

Build this package in your ROS environment.

        cd ~/ros/catkin_ws/src
        ln -s ~/src/PetriNetPlans/PNPros/example/rp_action .
        ln -s ~/src/PetriNetPlans/PNPros/example/rp_action_msgs .
        cd ..
        catkin_make


## Single robot execution

Start the demo

        cd ~src/PetriNetPlans/PNPros/example/rp_action/scripts
        ./start_demo.bash 

Run a plan

        ./runplan.bash <planname>

Stop the current plan

        ./runplan.bash stop


## Multi-robot execution

NOTE: Multi-robot configuration not working yet on ROS Melodic!

Start the demo

        cd ~src/PetriNetPlans/PNPros/example/rp_action/scripts
        ./start_demo.bash  multirobot

Run a plan

        ./runplan.bash <robotname> <planname>

Stop the current plan

        ./runplan.bash <robotname> stop


## Plans available


Plans available in ```ros_pnp\plans``` folder: ```sequence_loop```, ```sensing```, ```interrupt```, ```fork_join```.

Notes: 1) the ```obstacle``` condition can be activated by placing (drag with mouse) the red box in the simulated environment in front of the robot; 2) the ```wave``` action outputs some text on the screen.



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
([see PNPActionServer.h](PNPros/ROS_bridge/pnp_ros/include/pnp_ros/PNPActionServer.h)).

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





