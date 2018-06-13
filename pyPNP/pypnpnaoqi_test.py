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

p.exec_action('waitfor', 'screentouched', interrupt='timeout_2.5')

p.exec_action('say', 'hello')

to = 3
while (not p.get_condition('screentouched') and to>0):
    time.sleep(1)
    to -= 1

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

p.plan_cmd('hello', 'start')

print "here"

to = 3
while (not 'goal' in p.plan_status() and to>0):
    print p.plan_name(), p.plan_status()
    time.sleep(0.5)
    to -= 0.5

print p.plan_name(), p.plan_status()

p.plan_cmd('hello', 'stop')

# start actions (non-blocking)
p.action_cmd('A', '10', 'start')
p.action_cmd('B', '10', 'start')
p.action_cmd('C', '10', 'start')

time.sleep(1)

# get a list of all the running actions
l = p.running_actions()
print l

time.sleep(1)

# quit all the running actions
p.quit_running_actions()

time.sleep(1)

l = p.running_actions()
print l

p.end()

