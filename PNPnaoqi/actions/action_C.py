import qi
import argparse
import sys
import time
import threading

import action_base
from action_base import *

# Example of instantaneous action

actionName = "C"


def actionThread_exec (params):
    t = threading.currentThread()
    print "Action "+actionName+" started with params "+params
    action_termination(actionName,params)


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

