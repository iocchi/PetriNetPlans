#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import datetime
import time
import threading

PKG = 'pnp_ros'
NODE = 'pnpactionserver'

import roslib; roslib.load_manifest('pnp_ros')
import rospy
import actionlib

import std_msgs
from std_msgs.msg import String

import pnp_msgs.msg
import pnp_msgs.srv



global condvalue
condvalue=0

def generateConditions():
  global condvalue
  r = rospy.Rate(0.1)
  while not rospy.is_shutdown():
    condvalue = 1 - condvalue
    print 'Condition value = ',condvalue
    r.sleep()

def action_callback(data):
  global canmove
  global pub_af
  print 'Action server received: Action ',data
  if data.function=='start':
    if data.name=='goto':
      turnAndGotoStr(data.params)
      af = ActionFinished()
      af.id = data.id
      af.name = data.name
      af.params = data.params
      af.result="OK"
      pub_af.publish(af)
  elif data.function=='end':
    print 'Stop action: ',data
    canmove=0

valid_goals = []

def is_goal_valid(id):
    for i in range(len(valid_goals)):
        if (valid_goals[i]==id):
            return True
    return False

def executeAction(goalhandler):
  global valid_goals
  r = rospy.Rate(1)
  goal = goalhandler.get_goal()
  feedback = pnp_msgs.msg.PNPActionFeedback()
  result   = pnp_msgs.msg.PNPResult()
  valid_goals.append(goal.id)
  cnt=0
  while (cnt<5 and is_goal_valid(goal.id)): 
    print 'Executing ['+str(goal.id)+'] ',goal.name,' ',cnt
    feedback.feedback = 'running '+str(cnt)+' ...'
    goalhandler.publish_feedback(feedback)
    r.sleep()
    cnt = cnt+1
  result.result = 'OK'
  goalhandler.set_succeeded(result, 'OK');
  rospy.loginfo('Action Succeeded: %s %s' % (goal.name, goal.params))


def startAction(goalhandler):
  goal = goalhandler.get_goal()
  print "Starting "+goal.name+" "+goal.params
  # accept the goal
  goalhandler.set_accepted()
  # start a thread for action execution
  th = threading.Thread(None, executeAction,args=(goalhandler, ));
  th.start();


def cancelAction(goalhandler):
  global valid_goals  
  goal = goalhandler.get_goal()
  print "Terminating "+goal.name+" "+goal.params
  # accept the goal
  goalhandler.set_accepted()
  if goal.id in valid_goals:
    valid_goals.remove(goal.id)
  



class PNPActionServer(object):
  # create messages that are used to publish feedback/result
  _feedback = pnp_msgs.msg.PNPActionFeedback()
  _result   = pnp_msgs.msg.PNPResult()

  def __init__(self, name):
    self._action_server_name = name
    #self._as = actionlib.SimpleActionServer(self._action_name, 
	#           pnp_msgs.msg.PNPAction, execute_cb=self.execute_cb, 
	#           auto_start=False)
    self._as = actionlib.ActionServer(self._action_server_name,
	           pnp_msgs.msg.PNPAction, self.execute_cb, 
	           auto_start=False)
    self._as.start()
    rospy.loginfo('%s: Action Server started' % self._action_server_name)
    
  def execute_cb(self, goalhandler):
    # init running
    self._feedback.feedback = 'running...'
    goalhandler.publish_feedback(self._feedback)    
    goal = goalhandler.get_goal()
    # publish info to the console for the user
    rospy.loginfo('%s: Starting action %s %s' % (self._action_server_name, goal.name, goal.params))
    if goal.function=='start':
        # start executing the action
        startAction(goalhandler)
    elif goal.function=='interrupt':
        #print '### Interrupt ',goal.name
        cancelAction(goalhandler)
    elif goal.function=='end':
        #print '### End ',goal.name
        cancelAction(goalhandler)

def handle_PNPConditionEval(req):
  global condvalue
  cond = req.cond
  print 'Eval condition: ',cond
  res=0
  if (cond=='hello'):
     res=1
  elif (cond=='true'): 
      res = 1
  elif (cond=='false'):
      res = 0
  else: 
      res = condvalue
  print 'Result: ',res
  return pnp_msgs.srv.PNPConditionResponse(res)


if __name__ == '__main__':
  rospy.init_node(NODE) 
  rospy.set_param('robot_name','dummy')

  thC = threading.Thread(None, generateConditions);
  thC.start();

  PNPActionServer("PNP")
  rospy.Service('PNPConditionEval', pnp_msgs.srv.PNPCondition, handle_PNPConditionEval)
  rospy.spin()


