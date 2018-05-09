# pyPNP
# Module pnp_cmd_base
#
# Luca Iocchi 2018

import argparse
import sys
import os
import time
import threading

class tcol:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class PNPCmd_Base(object):

    def __init__(self):
        self.execlevel = 0 # pretty print

    def action_cmd_base(self, action, params, cmd):
        self.printindent()
        print("  %s %s -> %s" %(action,params,cmd))
        self.action_cmd(action,params,cmd)


    def action_params_split(self, astr):
        astr = astr.strip()
        c = astr.find('_')
        action = astr[0:c]
        params = astr[c+1:]
        return [action, params]


    def execRecovery(self, recovery):
        v = recovery.split(';')
        for i in range(len(v)):
            ap = v[i].strip()
            self.printindent()
            print('%s-- recovery %s%s' %(tcol.WARNING,ap,tcol.ENDC))
            if (ap=='restart_action'):
                return ap
            elif (ap=='skip_action'):
                return ap
            elif (ap=='restart_plan'):
                return ap
            elif (ap=='fail_plan'):
                return ap
            else:
                [action, params] = self.action_params_split(ap)
                self.exec_action(action, params)

    def printindent(self):
        for i in range(self.execlevel):
            print '    ',

    def exec_action(self, action, params, interrupt='', recovery=''):
        self.printindent()
        print("%sExec: %s %s %s" %(tcol.OKGREEN,action,params,tcol.ENDC))

        run = True

        while (run): # interrupt may restart this action

            self.action_cmd_base(action, params, 'start')
            time.sleep(0.1)

            # wait for action to terminate

            r = 'run'
            c = False
            while (r=='run' and not c):
                r = self.action_status(action)
                c = self.get_condition(interrupt)
                #print "   -- action status: %s, interrupt condition: %r" %(r,c)
                time.sleep(0.1)
            #print "   -- action status: %s, interrupt condition: %r" %(r,c)
            run = False  # exit 
            if (not c): # normal termination
                self.printindent()
                print("  %s %s -> %s" %(action,params,r))
            else: # interrupt
                self.printindent()
                print("  %s%s %s -> %s%s" %(tcol.WARNING,action,params,'interrupt',tcol.ENDC))
                self.action_cmd_base(action, params, 'interrupt')
                while self.action_status(action)=='run':
                    time.sleep(0.1)
                self.execlevel += 1
                rec = self.execRecovery(recovery)
                self.execlevel -= 1
                if rec=='restart_action':
                    run = True

    def plan_gen(self, planname):
        oscmd = 'cd %s; ./genplan.sh %s.plan %s.er' % (self.plan_folder, planname, planname)
        print(oscmd)
        os.system(oscmd)

    # TO BE IMPLEMENTED BY SUBCLASSES

    def begin(self):
        pass

    def end(self):
        pass

    def action_cmd(self,action,params,cmd): # non-blocking
        pass

    def action_status(self, action):
        return ''

    def get_condition(self, cond):
        return False

    def plan_cmd(self, planname, cmd): # non-blocking
        pass






