import os
import sys
sys.path.insert(0, os.getenv('PNP_HOME')+'/PNPnaoqi/actions')

import action_cmd
from action_cmd import *

a = ActionCmd()

a.begin()

a.execPNPaction('A', '')


for i in range(3):
    a.execPNPaction('turn', '45', interrupt='c1', recovery='waitfor_not_c1; restart_action')
    a.execPNPaction('say', 'hello')


a.end()

