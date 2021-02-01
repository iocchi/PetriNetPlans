# pnp_ros 

`pnp_ros` is a ROS node monitoring the execution of a PNP by orchestrating execution
of actions and checking of conditions.

## Client/server connection

### Centralized action server

`pnp_ros` communicates with a central `PNPActionServer` through `PNP` action defined as follows.

    #Goal
    uint32 id
    string robotname
    string name
    string params
    string function
    ---
    #Result
    string result
    ---
    #Feedback
    string feedback


`PNPActionServer` is responsible for dispatching actual execution of all actions.

### Distributed action server

`pnp_ros` publishes a String topic named `pnp/action_str` in the form 

        [<robotname>#]<actionname>[_<params>].<command>

`<command>` is either `start`, `interrupt`, `end`.

Action executors should listen to this message and execute the corresponding action. Programmer is responsible for ensuring that only one executor would take care of execution of an action.

Example of String values for topic `pnp/action_str`:

        goto_kitchen.start
        goto_kictchen.interrupt
        goto_kitchen.end

        robot1#goto_kitchen.start
        robot1#goto_kictchen.interrupt
        robot1#goto_kitchen.end

`pnp_ros` is listening for action commands through the topic `pnp/actionCmd`. Action executors have to notify when an action is terminated publishing a String message in the form

        [<robotname>#]<actionname>_<params>.end

Example of String values of topic `pnp/actionCmd`:

    robot#goto_kitchen.end


## Examples

Run a demo (see [rp_example](/PNPros/examples/rp_example))


To run a plan, use:

    rostopic pub planToExec std_msgs/String "data: '<plan_name>'" -1

To stop the plan, use:

    rostopic pub planToExec std_msgs/String "data: 'stop'" -1
  

 
To test a single action of the `PNPActionServer`:

    rostopic pub PNPActionCmd std_msgs/String "data: '<action_str> <command>'" --once

    <action_str> = actionname_actionparams

    <command> = start | end | interrupt

Examples:

    rostopic pub pnp/actionCmd std_msgs/String "data: 'goto_kitchen.start'" --once
    rostopic pub pnp/actionCmd std_msgs/String "data: 'wait_5.start'" --once

To listen action execution commands

    rostopic echo pnp/action_str

Status of actions under execution

    rosparam list pnp/actionStatus/

    rosparam get pnp/actionStatus/<actionname>


