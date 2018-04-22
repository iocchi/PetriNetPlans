import qi
import argparse
import sys
import time
import threading

import action_base
from action_base import *


actionName = "B"


def actionThread_exec (params):
    t = threading.currentThread()
    memory_service = getattr(t, "mem_serv", None)
    print "Action "+actionName+" started with params "+params
    if params=='':
        params = '3' # default
    # action init
    dt = 0.25
    count = int(float(params) / dt)
    # action init
    while (getattr(t, "do_run", True) and count>0): 
        print "Action "+actionName+" "+params+" exec..."
        # action exec
        count = count-1
        # action exec
        time.sleep(dt)
    print "Action "+actionName+" "+params+" terminated"
    # action end
    count = 0
    # action end
    memory_service.raiseEvent("PNP_action_result_"+actionName,"success");


def init(session):
    print actionName+" init"
    action_base.init(session, actionName, actionThread_exec)


def quit():
    print actionName+" quit"
    actionThread_exec.do_run = False


if __name__ == "__main__":

    app = action_base.initApp(actionName)
        
    init(app.session)

    #Program stays at this point until we stop it
    app.run()

    quit()

