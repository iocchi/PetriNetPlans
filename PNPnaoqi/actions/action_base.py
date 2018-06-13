import qi
import argparse
import sys
import time
import threading
import os

class tcol:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


G_actionThread_exec = {}  # action thread execution functions
G_actionThreads = {}  # action threads functions
G_memory_service = None 
G_session = None

G_actions_running = [] # list of action currently running

key_actioncmd = "PNP/ActionCmd"
key_actioncmd2 = "PNP_action"
#key_currentaction = "PNP/CurrentAction"  # should not be used: key_runningactions lists the current running actions
key_actionresult = "PNP/ActionResult/"
key_actionstatus = "PNP/ActionStatus/"
key_action_starttime = "PNP/ActionStarttime/"
key_runningactions = "PNP/RunningActions/"
key_quitrunningactions = "PNP/QuitRunningActions/"

acb = None # action callback
acb2 = None # action callback
qacb = None # quit actions callback

# action callback
# key = 'PNP/ActionCmd'
# value = '<command> <actionname> <params>'
def action_cb(value):
    doActionCmd(value)

# action callback
# key = 'PNP_action'
# value = '<command> <actionname> <params>'
def action_cb2(value):
    doActionCmd(value)

# quit_all_actions callback
def quitactions_cb(value):
    print "*********************************************"
    print " Received signal to quit all running actions "
    print "*********************************************"
    quit_all_running_actions()


# value = '<command> <actionname> <params>'
def doActionCmd(value):
    global G_actionThreads, G_actionThread_exec, G_memory_service, G_session
    global G_actions_running 
    print "action_cb value ",value
    v = value.split()
    if (v[0]=='start'):
        actionName = v[1]
        params=""
        if (len(v)>2):
            params=v[2]
        
        if (actionName in G_actionThread_exec):
            G_actionThreads[actionName] = threading.Thread(target = G_actionThread_exec[v[1]], args=(params,))
            G_actionThreads[actionName].mem_serv = G_memory_service
            G_actionThreads[actionName].session = G_session
            #G_memory_service.raiseEvent(key_currentaction,actionName+"_"+params)
            G_memory_service.raiseEvent(key_actionstatus+actionName,"run");
            # start time
            starttime = time.time() # seconds
            G_actionThreads[actionName].starttime = starttime
            G_memory_service.insertData(key_action_starttime+actionName,str(starttime));
            #print('action_base setting starttime %s : %f' %(actionName,starttime))
            G_actionThreads[actionName].do_run = True  # execution thread associated to actionName
            G_actionThreads[actionName].start()
            time.sleep(0.1)
            G_actions_running.append(actionName)
            G_memory_service.insertData(key_runningactions, G_actions_running)
        else:
            print("%sERROR: Action %s not found !!!%s" %(tcol.FAIL,v[1],tcol.ENDC))
    elif (v[0]=='end' or v[0]=='stop' or v[0]=='interrupt'):
        try:
            actionName = v[1]
            G_actionThreads[actionName].do_run = False  # execution thread associated to actionName
            #update_quit_action_status(actionName)
        except:
            print("%sERROR: Action %s not started !!!%s" %(tcol.FAIL,v[1],tcol.ENDC))

# actions quit from pyPNP
def quit_all_running_actions():
    global G_actions_running, G_memory_service
    l = list(G_actions_running) # use a copy because the list may be modified by action threads
    for a in l:
        G_memory_service.raiseEvent(key_actioncmd, 'interrupt '+a)


def update_quit_action_status(actionName):
    global G_actions_running, G_memory_service

    try:
        G_actions_running.remove(actionName)
        G_memory_service.insertData(key_runningactions, G_actions_running)
        G_memory_service.removeData(key_action_starttime+actionName)
        G_memory_service.raiseEvent(key_actionstatus+actionName,"finished");
        #G_memory_service.raiseEvent(key_currentaction,"")
        # print "DEBUG: action ",actionName," ended.  Thread ",G_actionThreads[actionName]
    except:
        print("%sERROR: Action %s not started !!!%s" %(tcol.FAIL,v[1],tcol.ENDC))



# action terminated by the action thread
def action_termination(actionName,params):
    global  G_memory_service, G_actionThreads
    if (not actionName in G_actionThreads):
        print("Action %s not in action threads" %actionName)
        return 

    if G_actionThreads[actionName].do_run:
        status = 'success'
        colstatus = tcol.OKGREEN
    else:
        status = 'failure'
        colstatus = tcol.FAIL
    G_memory_service.raiseEvent("PNP_action_result_"+actionName,status) # feedback to pnp_naoqi
    update_quit_action_status(actionName)
    print "%sAction %s %s terminated - %s%s" %(colstatus,actionName,params,status,tcol.ENDC)


# action terminated by the action thread
def action_success(actionName,params):
    global  G_memory_service
    G_memory_service.raiseEvent("PNP_action_result_"+actionName,"success") # feedback to pnp_naoqi
    update_quit_action_status(actionName)
    status = "success"
    colstatus = tcol.OKGREEN
    print "%sAction %s %s terminated - %s%s" %(colstatus,actionName,params,status,tcol.ENDC)

# action terminated by the action thread
def action_failed(actionName,params):
    global  G_memory_service
    G_memory_service.raiseEvent("PNP_action_result_"+actionName,"failure") # feedback to pnp_naoqi
    update_quit_action_status(actionName)
    status = "failure"
    colstatus = tcol.FAIL
    print "%sAction %s %s terminated - %s%s" %(colstatus,actionName,params,status,tcol.ENDC)


def action_failure(actionName,params):
    action_failed(actionName,params)


def initApp(actionName):
    
	parser = argparse.ArgumentParser()
	parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
	parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
	args = parser.parse_args()
	pip = args.pip
	pport = args.pport

	#Starting application
	try:
		connection_url = "tcp://" + pip + ":" + str(pport)
		print "Connecting to ",	connection_url
		app = qi.Application(["Action_"+actionName, "--qi-url=" + connection_url ])
	except RuntimeError:
		print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
		sys.exit(1)

	app.start()
	return app


def init(session, actionName, actionThread_exec):
    global G_actionThread_exec, G_memory_service, G_session, acb, acb2, qacb
    G_actionThread_exec[actionName] = actionThread_exec # execution thread function associated to actionName
    G_session = session
    G_memory_service  = session.service("ALMemory")

    #subscribe to PNP action event
    if (acb==None):
        print('Naoqi subscriber to %s' %(key_actioncmd))
        acb = G_memory_service.subscriber(key_actioncmd)
        acb.signal.connect(action_cb)
        print('Naoqi subscriber to %s' %(key_actioncmd2))
        acb2 = G_memory_service.subscriber(key_actioncmd2)
        acb2.signal.connect(action_cb2)
    
    if (qacb==None):
        print('Naoqi subscriber to %s' %(key_quitrunningactions))
        qacb = G_memory_service.subscriber(key_quitrunningactions)
        qacb.signal.connect(quitactions_cb)

    G_memory_service.declareEvent(key_actionresult+actionName);
    #G_memory_service.declareEvent(key_currentaction);

    print "Naoqi Action server "+actionName+" running..."


