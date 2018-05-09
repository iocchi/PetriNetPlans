#!/usr/bin/env python

# Naoqi action_cmd

import qi
import argparse
import os, sys
import time
import threading

try:
    sys.path.insert(0, os.getenv('PNP_HOME')+'/PNPnaoqi/actions')
    sys.path.insert(0, os.getenv('PNP_HOME')+'/pyPNP')
except Exception as e:
    print "Please set PNP_HOME environment variable to PetriNetPlans folder."
    print e
    sys.exit(1)

import action_base
from action_base import *
from conditions import *

# from PetriNetPlans/pyPNP
import pnp_cmd_base
from pnp_cmd_base import *

key_actioncmd = "PNP/ActionCmd"
key_currentaction = "PNP/CurrentAction"
key_actionresult = "PNP/ActionResult/"

class PNPCmd(PNPCmd_Base):

    def __init__(self):
        PNPCmd_Base.__init__(self)
        self.memory_service = None
        self.pip=os.environ['PEPPER_IP']
        self.pport=9559


    def init(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                            help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
        parser.add_argument("--pport", type=int, default=9559,
                            help="Naoqi port number")
        parser.add_argument("-a", type=str, default="",
                            help="action name")
        parser.add_argument("-p", type=str, default="",
                            help="params")
        parser.add_argument("-c", type=str, default="",
                            help="command")
        args = parser.parse_args()
        self.pip = args.pip
        self.pport = args.pport
        action = args.a
        params = args.p
        cmd = args.c

        return [action, params, cmd]


    def begin(self):
        #Starting application
        try:
            connection_url = "tcp://" + self.pip + ":" + str(self.pport)
            print "Connecting to ",    connection_url
            app = qi.Application(["Conditions", "--qi-url=" + connection_url ])
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
                   "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)

        app.start()
        self.memory_service  = app.session.service("ALMemory")


    def end(self):
        pass

    def action_cmd(self,action,params,cmd):
        v = cmd+" "+action+" "+params
        print('Writing on key: %s value: %s\n' %(key_actioncmd, v))
        self.memory_service.raiseEvent(key_actioncmd, v);

    def action_status(self, action):
        key = "PNP_action_result_"+action
        try:
            r = self.memory_service.getData(key)
        except:
            r = 'unknown'
        return r

    def get_condition(self, cond):
        return get_condition(self.memory_service, cond)


def main():
    a = PNPCmd()
    [action, params, cmd] = a.init()
    a.begin()
    a.action_cmd(action, params, cmd)
    time.sleep(1)    
    a.end()

if __name__ == "__main__":
    main()

