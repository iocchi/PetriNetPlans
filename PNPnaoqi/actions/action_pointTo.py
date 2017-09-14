import qi
import argparse
import sys
import time
import threading
import json

import action_base
from action_base import *

actionName = "pointTo"

def getDirection(memory_service, params):
    x = 0.0
    y = 0.0
    z = 0.0
    return x,y,z

def actionThread_exec (params):
    t = threading.currentThread()
    memory_service = getattr(t, "mem_serv", None)
    motion_service = getattr(t, "session", None),service("ALMotion")
    # Maybe this will not even be used
    # x,y,z = getDirection(memory_service, params)
    if (params=="left"):
        names = ["LElbowRoll","LElbowYaw","LShoulderPitch","LShoulderRoll","LWristYaw"]
        angles = [-57.0*almath.TO_RAD,-116.2*almath.TO_RAD,33.6*almath.TO_RAD,88.9 * almath.TO_RAD,-103.0*almath.TO_RAD]
        timeLists = [0.8,0.8,0.8,0.8,0.8,0.8]
        motion_service.angleInterpolation(names, angles, timeLists, True)
    elif (params=="right"):
        names = ["RElbowRoll","RElbowYaw","RShoulderPitch","RShoulderRoll","RWristYaw"]
        angles = [16.4*almath.TO_RAD,96.7*almath.TO_RAD,58.0*almath.TO_RAD,-66.5 * almath.TO_RAD,64.8*almath.TO_RAD]
        timeLists = [0.8,0.8,0.8,0.8,0.8,0.8]
        motion_service.angleInterpolation(names, angles, timeLists,True)
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
