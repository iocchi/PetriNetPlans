import qi
import argparse
import sys
import time
import threading
import os

G_actionThread_exec = None


def action_cb(value):
	global G_actionThread_exec, memory_service
	print "action_cb value ",value
	if (value=='start'):
		actionThread = threading.Thread(target = G_actionThread_exec)
		actionThread.mem_serv = memory_service
		actionThread.start()
	elif (value=='end'):
		actionThread.do_run = False
	elif (value=='interrupt'):
		actionThread.do_run = False


def init(actionName):
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
		app = qi.Application(["Action "+actionName, "--qi-url=" + connection_url ])
	except RuntimeError:
		print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
		sys.exit(1)

	app.start()

	return app


def main(actionName, actionThread_exec):
	global G_actionThread_exec, memory_service
	G_actionThread_exec = actionThread_exec 

	app = init(actionName)

	#Starting services
	memory_service  = app.session.service("ALMemory")
	memory_service.declareEvent("PNP_action_result_"+actionName);

	#subscribe to PNP action event
	acb = memory_service.subscriber("PNP_action_"+actionName)
	acb.signal.connect(action_cb)

	print "Naoqi Action server "+actionName+" running..."
	#Program stays at this point until we stop it
	app.run()
	print "Naoqi Action server "+actionName+" quit"

