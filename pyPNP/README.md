# pyPNP module

```pyPNP``` is a Python front-end for PNP execution monitoring.
With ```pyPNP``` it is possible to write Python controllers including PNP action and plan execution and condition evaluation.

pyPNP is independent from the middleware used. At this moment ROS and Naoqi frameworks are supported. pyPNP scripts can run with both ROS and Naoqi PNP engines, including using both at the same time (e.g. for a Pepper with ROS Naoqi driver).

## Install

```pyPNP``` is included in PetriNetPlans package. It uses PNP-ROS interface ```PetriNetPlans/PNPros/ROS_bridge/pnp_ros/py/pnp_cmd_ros.py``` and PNP-Naoqi interface ```PetriNetPlans/PNPnaoqi/py/pnp_cmd_naoqi.py```

## Use

Before running pyPNP scripts, you must start an action server (ROS or Naoqi or both).

### Start an action server

ROS: Start a PNPActionServer

Naoqi: Start the script ```init_actions.py``` in the action folder

### pyPNP script

Main functions available to manage plan and action executions are described through the example below.

```
import pnp_cmd_ros
from pnp_cmd_ros import *

import pnp_cmd_naoqi
from pnp_cmd_naoqi import *

p = pnp_cmd_ros.PNPCmd() # ROS actions and conditions
q = pnp_cmd_naoqi.PNPCmd() # Naoqi actions and conditions

p.begin()
q.begin()

# execute action / blocking function
p.exec_action('action', 'param1_param2')

# execute action with interrupt / blocking function
p.exec_action('action', 'params', interrupt='condition', recovery='waitfor_not_condition;restart_action') 

# start action / non-blocking function
p.action_cmd('action', 'params', 'start')

# stop action / non-blocking function
p.action_cmd('action', 'params', 'stop')

# start plan / non-blocking function
p.plan_cmd('cocktail_party','start') 

# stop plan / non-blocking function
p.plan_cmd('cocktail_party','stop') 

# read conditions
r = p.get_condition('condition')

p.end()
q.end()
```

