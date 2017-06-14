import qi
import argparse
import sys
import time
import threading
import os

G_actionThread_exec = None


def action_cb(value):
	global G_actionThread_exec, G_memory_service, G_session
	v = value.split()
	print "action_cb value ",value
	if (v[0]=='start'):
		params=""
		if (len(v)>1):
			params=v[1]
		actionThread = threading.Thread(target = G_actionThread_exec, args=(params,))
		actionThread.mem_serv = memory_service
		actionThread.session = session
		actionThread.start()
	elif (v[0]=='end'):
		actionThread.do_run = False
	elif (v[0]=='interrupt'):
		actionThread.do_run = False


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
    global G_actionThread_exec, G_memory_service, G_session
    G_actionThread_exec = actionThread_exec 
    G_session = session
    G_memory_service  = session.service("ALMemory")
    G_memory_service.declareEvent("PNP_action_result_"+actionName);

    #subscribe to PNP action event
    acb = G_memory_service.subscriber("PNP_action_"+actionName)
    acb.signal.connect(action_cb)

    print "Naoqi Action server "+actionName+" running..."



#def main(actionName, actionThread_exec):
#	global G_actionThread_exec, memory_service, session
#	G_actionThread_exec = actionThread_exec 

#	app = initApp(actionName)

	#Starting services
#	session = app.session

#    init(session,actionName)

	#Program stays at this point until we stop it
#	app.run()

#    quit(session,actionName)



