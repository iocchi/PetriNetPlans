import qi
import argparse
import sys
import time
import threading

import action_base
from action_base import *


actionName = "B"



class ActionB(NAOqiAction_Base):
    
    # self.session, self.memory_session, self.do_run

    def actionThread_exec (self, params):
        # self.memory_service 
        print "Action "+self.actionName+" started with params "+params
        if params=='':
            params = '3' # default
        # action init
        dt = 0.25
        count = int(float(params) / dt)
        # action init
        while (self.do_run and count>0): 
            print "Action "+self.actionName+" "+params+" exec..."
            #print "DEBUG:: Action thread ",t," run ",getattr(t, "do_run", True)
            # action exec
            count = count-1
            # action exec
            time.sleep(dt)

        # action end
        count = 0
        # action end
        action_termination(self.actionName,params)
        #action_success(actionName,params)
        #action_failure(actionName,params)




action = None

def init(session):
    global action
    action = ActionB(actionName,session)

def quit():
    global action
    action.stop()


# Used to test this action
if __name__ == "__main__":

    print('Starting action server for action %s (CTRL-C to quit)' %actionName)

    app = action_base.initApp()

    action = ActionB(actionName, app.session)

    #Program stays at this point until we stop it
    app.run()

    action.stop()

