import qi
import argparse
import sys
import time
import threading

import action_base
from action_base import *





def set_condition(memory_service, atom, value):
	key = "PNP_cond_"+atom
	memory_service.insertData(key,value);


# Input: either 'cond' or 'not_cond'
# Output: boolean or 'Unknown'
def get_condition(memory_service, literal):
    v = literal.split('_')
    neg = False
    cond = literal
    if (len(v)==2):
        cond = v[1]
        neg=True
    try:
        key = "PNP_cond_"+cond
        cval = memory_service.getData(key);
        if (not neg):
            val = (cval.lower()=='true') or (cval=='1')
        else:
            val = (cval.lower()=='false') or (cval=='0')
    except:
        cval = 'except'
        val = False # 'Unknown'

    #print "Evaluating ", literal, " cval ", cval, " neg: ", neg, " -> ", val
    return val
	

