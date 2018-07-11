#pragma once

// Topics

#define TOPIC_PNPACTIVEPLACES "pnp/currentActivePlaces" // names of current active places

#define TOPIC_PNPCONDITION "PNPConditionEvent" // PNP Conditions as events

#define TOPIC_PLANTOEXEC "planToExec" // executes the plan

#define TOPIC_PNPACTION "pnp_action" // action proxy publisher when using messages 
// (not used when PNP uses actionlib protocol - default)

#define TOPIC_PNPACTIONCMD "PNPActionCmd" // executes an action command. 
// Message syntax: <actionname>[_<params>] <start|end|interrupt>
// TODO Uniform with naoqi
// Message syntax: [start|end|interrupt] <actionname> [<params>]


#define TOPIC_PNPACTIONTERMINATION "pnp_action_termination"  // notifies action is terminated


// Services

#define SRV_PNPCONDITIONEVAL "PNPConditionEval"
#define SRV_PNPGETEVENT      "PNPGetEventStartingWith"
#define SRV_PNPCLEARBUFFER   "PNPClearBuffer"
#define SRV_PNPGETVAR        "PNPGetVariableValue"
#define SRV_PNPSETVAR        "PNPSetVariableValue"


// Params
#define PARAM_PNPCONDITIONBUFFER "PNPconditionsBuffer/" // PNP Conditions as param values

#define PARAM_PNPACTIONSTATUS "PNPActionStatus/" // PNP action status

#define PARAM_PNP_CURRENT_PLAN "PNPCurrentPlan" // current plan


