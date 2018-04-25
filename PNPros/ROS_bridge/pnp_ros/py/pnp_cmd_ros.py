#!/usr/bin/python
# -*- coding: utf-8 -*-

# ROS action_cmd

import threading,os
import roslib, rospy

import pnp_msgs.msg, pnp_msgs.srv

import std_msgs.msg

# from PetriNetPlans/pyPNP
import pnp_cmd_base
from pnp_cmd_base import *


roslib.load_manifest('pnp_ros')
PKG = 'pnp_ros'
NODE = 'pnp_cmd'

# ROS names (see pnp_ros/include/pnp_ros/names.h)

TOPIC_PLANTOEXEC = "planToExec"
TOPIC_PNPACTIONCMD = "PNPActionCmd"
SRV_PNPCONDITIONEVAL = "PNPConditionEval"
PARAM_PNPACTIONSTATUS = "PNPActionStatus/"
PARAM_PNPCONDITIONBUFFER = "PNPconditionsBuffer/"

PNPPLANFOLDER = "pnp/plan_folder"


def get_robot_key(name):
    key = name
    if rospy.has_param('robotname'):
        key = "/"+rospy.get_param('robotname')+"/"+key
    return key


class PNPCmd(PNPCmd_Base):

    def __init__(self):
        PNPCmd_Base.__init__(self)
        self.pub_actioncmd = None
        self.pub_plantoexec = None

    def init(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("-a", type=str, default="",
                            help="action name")
        parser.add_argument("-p", type=str, default="",
                            help="params")
        parser.add_argument("-c", type=str, default="",
                            help="command")
        args = parser.parse_args()
        action = args.a
        params = args.p
        cmd = args.c

        return [action, params, cmd]

    def begin(self):
        rospy.init_node(NODE)

        self.rate = rospy.Rate(2) # hz
        self.rate.sleep()

        key = get_robot_key(TOPIC_PNPACTIONCMD)
        self.pub_actioncmd = rospy.Publisher(key, std_msgs.msg.String, queue_size=10)
        print("Publisher %s" %key)
        self.rate.sleep()

        key = get_robot_key(TOPIC_PLANTOEXEC)
        self.pub_plantoexec = rospy.Publisher(key, std_msgs.msg.String, queue_size=10)
        print("Publisher %s" %key)
        self.rate.sleep()

        key = get_robot_key(SRV_PNPCONDITIONEVAL)
        print("Waiting for service %s ..." %key)
        rospy.wait_for_service(key)
        print("Service %s OK" %key)


        self.plan_folder = rospy.get_param(get_robot_key(PNPPLANFOLDER))

        # wait for connections on action_cmd topic
        conn = self.pub_actioncmd.get_num_connections()
        #rospy.loginfo('Connections: %d', conn)
        while conn==0:
            self.rate.sleep()
            conn = self.pub_actioncmd.get_num_connections()
            #rospy.loginfo('Connections: %d', conn)

    def end(self):
        pass

    def action_cmd(self,action,params,cmd):
        if (cmd=='stop'):
            cmd = 'interrupt'
        data = action+"_"+params+" "+cmd
        self.pub_actioncmd.publish(data)
        self.rate.sleep()

    def action_status(self, action):
        key = get_robot_key(PARAM_PNPACTIONSTATUS)+action
        try:
            r = rospy.get_param(key)
            #print('Action %s status %s' %(action,r))
        except KeyError, e:
            print "action %s status error: %s" %(action,e)
            r = ''
        return r

    def get_condition(self, cond):
        try:
            service = rospy.ServiceProxy(get_robot_key(SRV_PNPCONDITIONEVAL), pnp_msgs.srv.PNPCondition)
            r = service(cond)
            return r.truth_value!=0
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def plan_cmd(self, planname, cmd): # non-blocking
        if (cmd=='start'):
            self.plan_gen(planname)
            self.pub_plantoexec.publish(planname)
        elif (cmd=='stop'):
            self.pub_plantoexec.publish('stop')
        else:
            print("ERROR: plan cmd %s %s undefined!" %(planname,cmd))
        self.rate.sleep()


def main():
    a = PNPCmd()
    [action, params, cmd] = a.init()
    a.begin()
    a.action_cmd(action, params, cmd)
    a.end()


if __name__ == "__main__":
    main()

