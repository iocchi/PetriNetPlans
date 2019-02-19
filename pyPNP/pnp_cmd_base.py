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
        if (cmd=='start'):
            print("  %s%s %s -> %s%s" %(tcol.OKGREEN,action,params,cmd,tcol.ENDC))
        elif (cmd=='interrupt'):
            print("  %s%s %s -> %s%s" %(tcol.WARNING,action,params,cmd,tcol.ENDC))
        elif (cmd=='end' or cmd=='success'):
            print("  %s%s %s -> %s%s" %(tcol.OKGREEN,action,params,cmd,tcol.ENDC))
        elif (cmd=='failure'):
            print("  %s%s %s -> %s%s" %(tcol.FAIL,action,params,cmd,tcol.ENDC))
        else:
            print("  %s %s -> %s" %(action,params,cmd))
        self.action_cmd(action,params,cmd)


    def action_params_split(self, astr):
        astr = astr.strip()
        c = astr.find('_')
        action = astr[0:c]
        params = astr[c+1:]
        return [action, params]


    def execRecovery(self, recovery):
        if recovery=='':
            return
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
                # check for interrupt condition
                if (interrupt[0:7].lower()=='timeout'):
                    now = time.time() 
                    st = self.action_starttime(action)
                    v = interrupt.split('_')
                    #print 'timeout %f %f diff: %f > %f' %(now, st, now-st, float(v[1]) )
                    c = (now - st) > float(v[1])
                else:
                    c = self.get_condition(interrupt)
                #print "   -- action status: %s, interrupt condition: %r" %(r,c)
                time.sleep(0.1)
            #print "   -- action status: %s, interrupt condition: %r" %(r,c)
            run = False  # exit 
            if (not c): # normal termination
                self.printindent()
                if (r=='success'):
                    print("  %s%s %s -> %s%s" %(tcol.OKGREEN,action,params,r,tcol.ENDC))
                else:
                    print("  %s%s %s -> %s%s" %(tcol.FAIL,action,params,r,tcol.ENDC))
            else: # interrupt
                r = 'interrupt'
                self.action_cmd_base(action, params, 'interrupt')
                while self.action_status(action)=='run':
                    time.sleep(0.1)
                self.execlevel += 1
                rec = self.execRecovery(recovery)
                self.execlevel -= 1
                if rec=='restart_action':
                    run = True
        return r



    def start_action(self, action, params):
        self.printindent()
        print("%sStart: %s %s %s" %(tcol.OKGREEN,action,params,tcol.ENDC))

        self.action_cmd_base(action, params, 'start')
        time.sleep(0.1)


    def wait_for_termination(self, action):
        # wait for action to terminate, returns termination status
        r = 'run'
        while (r=='run'):
            r = self.action_status(action)
            time.sleep(0.1)

        self.printindent()
        if (r=='success'):
            print("  %s%s .. -> %s%s" %(tcol.OKGREEN,action,r,tcol.ENDC))
        else:
            print("  %s%s .. -> %s%s" %(tcol.FAIL,action,r,tcol.ENDC))
        return r



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

    def set_condition(self, cond, value):
        return

    def plan_cmd(self, planname, cmd): # non-blocking
        pass

    def plan_name(self):
        return 'no plan'

    def plan_status(self):
        return 'no status'



