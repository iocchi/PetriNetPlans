import qi
import argparse
import sys
import time
import threading
import json

import action_base
from action_base import *

'''
Accepts keywords as parameters and uses ALAnimatedSpeech
'''

actionName = "say"

def phraseToSay(memory_service,params):
    if (params=='hello'):
        return "Hallo! Ich bin Pepper."
    elif (params=='greetperson'):
        tosay = "Hallo person!"
        try:
            pid = memory_service.getData('Actions/personhere/PersonID')
            pinfo = memory_service.getData('DialogueVesponse')
            pdict = json.loads(pinfo)
            tosay = "Hello "+pdict['Name']+" !"
        except Exception as e:
            print str(e)

        return tosay
    elif (params=='ok'):
        return "OK, dann mach ich das."
    elif (params=='offerTour'):
        return "Ich koennte euch etwas ueber das Mittelalter in Bielefeld erzaehlen. Habt ihr lust dazu?"
    elif (params=='personnotfound'):
        return "Wo seid ihr denn?"
    elif (params=='goodbye'):
        return "Auf Wiedersehen."
    return "Nothing to say."

def actionThread_exec (params):
    t = threading.currentThread()
    memory_service = getattr(t, "mem_serv", None)
    tts_service = getattr(t, "session", None).service("ALAnimatedSpeech")
    print "Action "+actionName+" started with params "+params
    # action init
    count = 1
    tosay = phraseToSay(memory_service,params)
    tts_service.say(tosay)
    print "  -- Say: "+tosay
    # action init
    while (getattr(t, "do_run", True) and count>0): 
        print "Action "+actionName+" "+params+" exec..."
        # action exec
        count = count - 1
        # action exec
        time.sleep(0.1)

    print "Action "+actionName+" "+params+" terminated"
    # action end

    # action end
    memory_service.raiseEvent("PNP_action_result_"+actionName,"success");


def init(session):
    print actionName+" init"
    action_base.init(session, actionName, actionThread_exec)


def quit():
    print actionName+" quit"
    actionThread_exec.do_run = False



if __name__ == "__main__":

    app = action_base.initApp(actionName)
        
    init(app.session)

    #Program stays at this point until we stop it
    app.run()

    quit()


