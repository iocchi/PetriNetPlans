#!/usr/bin/env python

import qi
import argparse
import sys
import time
import threading

import action_base
from action_base import *
from conditions import *


class tcol:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# globals
app = None
memory_service = None
pip=os.environ['PEPPER_IP']
pport=9559
execlevel = 0


def begin():
    global app, memory_service, pip, pport

    #Starting application
    try:
        connection_url = "tcp://" + pip + ":" + str(pport)
        print "Connecting to ",    connection_url
        app = qi.Application(["Conditions", "--qi-url=" + connection_url ])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    app.start()
    memory_service  = app.session.service("ALMemory")



def init():
    global app, memory_service, pip, pport
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
    pip = args.pip
    pport = args.pport
    action = args.a
    params = args.p
    cmd = args.c

    return [action, params, cmd]


def end():
    pass


def start_action(actionName, params):
    global memory_service
    memory_service.raiseEvent(action_base.key_actioncmd,"start "+actionName+" "+params);


def end_action(actionName):
    global memory_service
    memory_service.raiseEvent(action_base.key_actioncmd,"end "+actionName);

def interrupt_action(actionName):
    global memory_service
    memory_service.raiseEvent(action_base.key_actioncmd,"interrupt "+actionName);


def PNPaction_cmd(action, params, cmd):

    printindent()
    print("  %s %s -> %s" %(action,params,cmd))

    if (cmd=='start'):
        start_action(action,params)
    elif (cmd=='end' or cmd=='stop'):
        end_action(action)
    elif (cmd=='interrupt'):
        interrupt_action(action)


def action_params_split(astr):
    #TODO
    astr = astr.strip()
    c = astr.find('_')
    action = astr[0:c]
    params = astr[c+1:]
    return [action, params]


def execRecovery(recovery):
    v = recovery.split(';')
    for i in range(len(v)):
        ap = v[i].strip()
        printindent()
        print('%s-- recovery %s%s' %(tcol.WARNING,ap,tcol.ENDC))
        if (ap=='restart_action'):
            return ap
        elif (ap=='skip_action'):
            return ap
        elif (ap=='restart_plan'):
            return ap
        elif (ap=='fail_plan'):
            return ap
        else:
            [action, params] = action_params_split(ap)
            execPNPaction(action, params)


def printindent():
    global execlevel
    for i in range(execlevel):
        print '    ',

def execPNPaction(action, params, interrupt='', recovery=''):
    global memory_service, execlevel

    printindent()
    print("%sExec: %s %s %s" %(tcol.OKGREEN,action,params,tcol.ENDC))

    run = True

    while (run): # interrupt may restart this action

        PNPaction_cmd(action, params, 'start')
        time.sleep(0.1)

        # wait for action to terminate
        key = "PNP_action_result_"+action
        r = ''
        c = False
        while (r!='success' and not c):
            r = memory_service.getData(key)
            c = get_condition(memory_service, interrupt)
            #print "   -- action: %s, interrupt condition: %r" %(r,c)
            time.sleep(0.1)
        run = False  # exit 
        if (not c): # normal termination
            printindent()
            print("  %s %s -> %s" %(action,params,r))
        else: # interrupt
            printindent()
            print("  %s%s %s -> %s%s" %(tcol.WARNING,action,params,'interrupt',tcol.ENDC))
            execlevel += 1
            r = execRecovery(recovery)
            execlevel -= 1
            if r=='restart_action':
                run = True


def PNPplan_cmd(planname, cmd): # non-blocking
    # TODO
    pass


def execPNP(planname):  # blocking
    PNPplan_cmd(planname, 'start')
    # wait for plan to terminate
    # TODO


def main():

    [action, params, cmd] = init()

    begin()

    PNPaction_cmd(action, params, cmd)

    time.sleep(1)    

    end()

if __name__ == "__main__":
    main()

