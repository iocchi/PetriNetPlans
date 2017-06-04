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
	args = parser.parse_args()
	pip = args.pip
	pport = args.pport

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

	return app



def set_condition(memory_service, atom, value):
	key = "PNP_cond_"+atom
	memory_service.insertData(key,value);


def get_condition(memory_service, atom):
	key = "PNP_cond_"+atom
	return memory_service.getData(key);


def main():

	app = init()
	memory_service  = app.session.service("ALMemory")

	set_condition(memory_service,"phi","false")

	time.sleep(1)	

	print "Condition: phi = ",get_condition(memory_service,"phi")

	time.sleep(1)	
#	app.run()


if __name__ == "__main__":
	main()



