import os
import sys

try:
    sys.path.insert(0, os.getenv('PNP_HOME')+'/PNPnaoqi/py')
except:
    print "Please set PNP_HOME environment variable to PetriNetPlans folder."
    sys.exit(1)

import pnp_cmd_naoqi
from pnp_cmd_naoqi import *

p = PNPCmd()

p.begin()

p.exec_action('A', '3') # blocking

# start actions (non-blocking)
p.start_action('A', '15')
p.start_action('B', '5')
p.start_action('C', '1')

time.sleep(1)

# get a list of all the running actions
l = p.running_actions()
print('Running actions: %r' %l)

time.sleep(1)

p.wait_for_termination('B')

l = p.running_actions()
print('Running actions: %r' %l)

# quit all the running actions
p.quit_running_actions()

time.sleep(1)

l = p.running_actions()
print('Running actions: %r' %l)

p.end()

