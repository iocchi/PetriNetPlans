#!/usr/bin/env python

import qi
import argparse
import sys
import time
import threading

import action_base
from action_base import *


def init():
	global app, memory_service
	parser = argparse.ArgumentParser()
	parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
	parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
	parser.add_argument("-a", type=str, default="",
                        help="action name")
	parser.add_argument("-p", type=str, default="",
                        help="params")
	args = parser.parse_args()
	pip = args.pip
	pport = args.pport
	action = args.a
	params = args.p

	#Starting application
	try:
		connection_url = "tcp://" + pip + ":" + str(pport)
		print "Connecting to ",	connection_url
		app = qi.Application(["Conditions --qi-url=" + connection_url ])
	except RuntimeError:
		print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
		sys.exit(1)

	app.start()

	return [app, action, params]



def start_action(memory_service, actionName, params):
	key = "PNP_action_"+actionName
	memory_service.raiseEvent(key,"start "+params);


def end_action(memory_service, actionName, params):
	key = "PNP_action_"+actionName
	memory_service.raiseEvent(key,"end");


def main():

	[app, action, params] = init()
	memory_service  = app.session.service("ALMemory")

	print "Exec: ",action,params

	start_action(memory_service,action,params)

	time.sleep(1)	


if __name__ == "__main__":
	main()



