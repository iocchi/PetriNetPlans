import qi
import argparse
import sys
import time
import threading
import json

import action_base
from action_base import *

actionName = "moveTo"

def getGoal(memory_service, params):
    '''
    TODO: Create external list with coordinates
    '''
    x = 0.0
    y = 0.0
    return x, y

def actionThread_exec (params):
    t = threading.currentThread()
    memory_service = getattr(t, "mem_serv", None)
    navigation_service = getattr(t, "session", None).service("ALNavigation")
    x, y = getGoal(memory_service, params)
    navigation_service.navigateToInMap([x, y, 0.0])

    while (getattr(t, "do_run", True) and count>0): 
        print "Action "+actionName+" "+params+" exec..."
        # action exec
        count = count - 1
        # action exec
        time.sleep(0.1)

    print "Action "+actionName+" "+params+" terminated"
    # action end

    # action end
    memory_service.raiseEvent("PNP_action_result_"+actionName,"success");

def init(session):
    print actionName + " init"
    action_base.init(session, actionName, actionThread_exec)

def quit():
    print actionName + " quit"
    actionThread_exec.do_run = False

if __name__ == "__main__":
    app = action_base.initApp(actionName)
    
    init(app.session)

    app.run()
    quit()
