#!/usr/bin/env python

import qi
import argparse
import sys
import time
import threading
import os

import conditions

def init():
	global app, memory_service
	parser = argparse.ArgumentParser()
	parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
	parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
	parser.add_argument("-c", type=str, default="",
                        help="condition")
	parser.add_argument("-v", type=str, default="true",
                        help="value")
	args = parser.parse_args()
	pip = args.pip
	pport = args.pport
	cond = args.c
	val = args.v

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

	return [app, cond, val]




def main():

	[app, cond, val] = init()
	memory_service  = app.session.service("ALMemory")

	conditions.set_condition(memory_service,cond,val)

	time.sleep(0.5)	

	print "Condition: ",cond," = ",conditions.get_condition(memory_service,cond)



if __name__ == "__main__":
	main()



