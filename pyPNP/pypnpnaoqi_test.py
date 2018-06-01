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

p.exec_action('say', 'hello')

while (not p.get_condition('screentouched')):
    time.sleep(1)

p.exec_action('say', 'starting')

i=0

#remove comment if you want the robot to move
#p.exec_action('turn', '-90', interrupt='screentouched', recovery='skip_action')

while (i<5 and not p.get_condition('screentouched')):
    
    y = -40 + 20*i
    p.exec_action('headpose', '%d_20' %y)
    time.sleep(0.5)
    i += 1

p.exec_action('headpose', '0_-10')

p.exec_action('say', 'goodbye')

p.end()

