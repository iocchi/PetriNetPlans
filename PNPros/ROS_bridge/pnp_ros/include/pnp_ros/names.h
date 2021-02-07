#pragma once

// Topics

// PNPAS sub 
#define TOPIC_PNPCONDITION "pnp/conditionEvent" // PNP Conditions as events

// PNPAS pub
#define TOPIC_PLANTOEXEC "pnp/planToExec" // executes the plan 

// pnp_ros pub
#define TOPIC_PNPACTION "pnp/action" // action proxy publisher 
#define TOPIC_PNPACTION_STR "pnp/action_str" // action proxy publisher 
                                             // string format <actionname>[_<params>].<command>

// pnp_ros sub
#define TOPIC_PNPACTIONTERMINATION "pnp/actionTermination"  // notifies action is terminated

// pnp_ros sub
#define TOPIC_PNPACTIONCMD "pnp/actionCmd" // received action command to execute
// Message syntax: <actionname>[_<params>].<start|end|interrupt>
// TODO Uniform with naoqi
// Message syntax: [start|end|interrupt] <actionname> [<params>]

// pnp_ros pub, PNPAS sub
#define TOPIC_PNPACTIVEPLACES "pnp/currentActivePlaces" // names of current active places


// Services (PNPAS)

#define SRV_PNPCONDITIONEVAL "pnp/conditionEval"
#define SRV_PNPGETEVENT      "pnp/getEventStartingWith"
#define SRV_PNPCLEARBUFFER   "pnp/clearBuffer"
#define SRV_PNPGETVAR        "pnp/getVariableValue"
#define SRV_PNPSETVAR        "pnp/setVariableValue"

// Params

#define PARAM_PNPCONDITIONBUFFER "pnp/conditionsBuffer/" // PNP Conditions as param values

#define PARAM_PNPACTIONSTATUS "pnp/actionStatus/" // PNP action status

#define PARAM_PNP_CURRENT_PLAN "pnp/currentPlan" // current plan


