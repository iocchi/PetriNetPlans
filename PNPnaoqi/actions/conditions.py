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


def get_condition(memory_service, atom):
	key = "PNP_cond_"+atom
	return memory_service.getData(key);






