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

p.exec_action('A', '')

for i in range(4):
    p.exec_action('turn', '90', interrupt='c1', recovery='waitfor_not_c1; restart_action')
    p.exec_action('say', 'hello')

p.end()

