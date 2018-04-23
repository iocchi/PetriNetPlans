import os
import sys

try:
    sys.path.insert(0, os.getenv('PNP_HOME')+'/PNPros/ROS_bridge/pnp_ros/py')
except:
    print "Please set PNP_HOME environment variable to PetriNetPlans folder."
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *

p = PNPCmd()

p.begin()

p.exec_action('A', '')

#for i in range(4):
#    p.exec_action('turn', '90', interrupt='c1', recovery='waitfor_not_c1; restart_action')
#    p.exec_action('say', 'hello')

for i in range(30):
    print(p.get_condition('personhere'))
    time.sleep(0.5)

#p.plan_cmd('cocktail_party','start')

#time.sleep(20)

#p.plan_cmd('cocktail_party','stop')

p.end()

