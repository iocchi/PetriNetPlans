#!/usr/bin/python
# -*- coding: utf-8 -*-


import threading
import sys
import os
import roslib
import rospy
import actionlib
sys.path.append(os.path.join(os.path.dirname(__file__), '../actions'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../conditions'))

from importlib import import_module
from AbstractAction import AbstractAction
from ConditionManager import ConditionManager
from pnp_msgs.msg import PNPActionFeedback, PNPResult, PNPAction
from pnp_msgs.srv import PNPCondition, PNPConditionResponse
from std_msgs.msg import String

roslib.load_manifest('pnp_ros')
PKG = 'pnp_ros'
NODE = 'pnpactionserver'
action_instances = {}
conditionManager = None

# topic to publish action start/interrupt
TOPIC_PNPACTION_STR = "pnp/action_str"

# topic to subscriber for end
TOPIC_PNPACTIONCMD = "pnp/actionCmd"

SRV_PNPCONDITIONEVAL = "pnp/conditionEval"

pub_actioncmd = None

#TODO adjust for multiple actions, need a map <string, goalhandler>
#     to manage end of individual actions
current_goalhandler = None

## find the action implementation
def find_action_implementation(action_name):
    try:
        action_class = getattr(import_module(action_name), action_name)
    except (ImportError, AttributeError):
        rospy.logwarn("action " + action_name + " not implemented")
        return None
    else:
        if issubclass(action_class, AbstractAction):
            return action_class
        else:
            rospy.logwarn("class " + action_class + " must inherit from AbstractAction")
            return None

pnpas = None

def actionproxy_cb(data):
    global current_goalhandler

    sdata = data.data
    #print(sdata)

    robot = None
    action = None
    params = None
    command = None
    v = sdata.split('#')
    if len(v)>1:
        robot = v[0]
        sdata = v[1]
    v = sdata.split('.')
    if len(v)!=2:
        raise Exception("ActionProxy: wrong format in %s [%s]" %(TOPIC_PNPACTIONPROXY_STR, sdata))
    command = v[1]
    k = v[0].find('_')
    if (k<0):
        action = v[0]
    else:
        action = v[0][0:k]
        params = v[0][k+1:]

    #print("robot: %s action: %s params: %s command: %s" \
    #    %(robot, action, params, command))
    if command=='end':
        current_goalhandler.set_succeeded()
    else:
        print("ActionProxy: wrong command %s" %(command))


def startAction(goalhandler):
    global pub_actioncmd, current_goalhandler

    current_goalhandler = goalhandler

    goal = goalhandler.get_goal()
    rospy.loginfo("Starting action: %s %s" %(goal.name,goal.params))

    # search for an implementation of the action
    action = find_action_implementation(goal.name)

    # accept the goal
    goalhandler.set_accepted()

    if action:
        # Instantiate the action
        action_instance = action(goalhandler, goal.params)

        # add action instance to the dict
        action_instances.update({
            goal.id : action_instance
        })

        # start the action
        action_instances[goal.id].start_action()

    # use messages to manage actions
    cmd = 'start'
    data = goal.name+"_"+goal.params+"."+cmd
    pub_actioncmd.publish(data)


def cancelAction(goalhandler):
    global pub_actioncmd
    goal = goalhandler.get_goal()
    print "Terminating " + goal.name + " " + goal.params
    # accept the goal
    goalhandler.set_accepted()

    # stop the action
    if goal.id in action_instances:
        action_instances[goal.id].stop_action()

    goalhandler.set_succeeded()

class PNPActionServer(object):
    #  create messages that are used to publish feedback/result
    _feedback = PNPActionFeedback()
    _result = PNPResult()

    def __init__(self, name):
        self._action_server_name = name
        self._as = actionlib.ActionServer(self._action_server_name,
                                          PNPAction,
                                          self.execute_cb,
                                          auto_start=False)
        self._as.start()
        rospy.loginfo('%s: Action Server started' % self._action_server_name)

    def execute_cb(self, goalhandler):
        r = rospy.Rate(4)
        # init running
        self._feedback.feedback = 'running...'
        goalhandler.publish_feedback(self._feedback)
        goal = goalhandler.get_goal()

        # publish info to the console for the user
        rospy.loginfo('%s: Starting action %s %s' %
                      (self._action_server_name, goal.name, goal.params))
        if goal.function == 'start':
            # start executing the action
            startAction(goalhandler)
        elif goal.function == 'interrupt':
            #  print '### Interrupt ',goal.name
            cancelAction(goalhandler)
        elif goal.function == 'end':
            #  print '### End ',goal.name
            cancelAction(goalhandler)

def handle_PNPConditionEval(req):
    cond_elems = req.cond.split("_")
    cond = cond_elems[0]
    params = cond_elems[1:]

    rospy.loginfo('Eval condition: ' + cond + ' ' + ' '.join(params))

    # evaluate through the condition manager
    cond_value = conditionManager.evaluate(cond, params)

    rospy.loginfo('Condition ' + cond + ' value: ' + str(cond_value))
    return PNPConditionResponse(cond_value)

if __name__ == '__main__':
    rospy.init_node(NODE)

    conditionManager = ConditionManager()

    pnpas = PNPActionServer("PNP")
    rospy.Service(SRV_PNPCONDITIONEVAL,
                  PNPCondition,
                  handle_PNPConditionEval)

    pub_actioncmd = rospy.Publisher(TOPIC_PNPACTION_STR, String, queue_size=10)
    print("Action execution publisher: %s" %TOPIC_PNPACTION_STR)

    sub_actioncmd = rospy.Subscriber(TOPIC_PNPACTIONCMD, String, actionproxy_cb)
    print("Action execution subscriber: %s" %TOPIC_PNPACTIONCMD)

    rospy.spin()

