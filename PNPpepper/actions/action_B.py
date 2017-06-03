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
	# action init
	count = 20
	# action init
	while (getattr(t, "do_run", True) and count>0): 
		print "Action "+actionName+" "+params+" exec..."
		# action exec
		count = count-1
		# action exec
		time.sleep(0.1)
	print "Action "+actionName+" "+params+" terminated"
	# action end
	count = 0
	# action end
	memory_service.raiseEvent("PNP_action_result_"+actionName,"success");



if __name__ == "__main__":
    main(actionName, actionThread_exec)

