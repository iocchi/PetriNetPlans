import qi
import argparse
import sys
import time
import threading

import action_base
from action_base import *


actionName = "C"


def actionThread_exec ():
	t = threading.currentThread()
	memory_service = getattr(t, "mem_serv", None)
	print "Action "+actionName+" started"
	# action init
	count = 30
	# action init
	while (getattr(t, "do_run", True) and count>0): 
		print "Action "+actionName+" exec..."
		# action exec
		count = count-1
		# action exec
		time.sleep(0.1)
	print "Action "+actionName+" terminated"
	# action end
	count = 0
	# action end
	memory_service.raiseEvent("PNP_action_result_"+actionName,"success");



if __name__ == "__main__":
    main(actionName, actionThread_exec)

