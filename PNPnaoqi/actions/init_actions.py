#!/usr/bin/env python

import qi
import argparse
import sys
import os
import time

import action_base
from action_base import *

import action_A, action_B, action_C, wait, action_displayImage, action_moveTo, action_posture, action_say

def init(session):
    action_A.init(session)
    action_B.init(session)
    action_C.init(session)
    action_displayImage.init(session)
    action_moveTo.init(session)
    action_posture.init(session)
    action_say.init(session)
    wait.init(session)

def quit():
    action_A.quit()
    action_B.quit()
    action_C.quit()
    action_displayImage.quit()
    action_moveTo.quit()
    action_posture.quit()
    action_say.quit()
    wait.quit()

def main():
    global memory_service
    parser = argparse.ArgumentParser()
    parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
    args = parser.parse_args()
    pip = args.pip
    pport = args.pport

    #Starting application
    try:
        connection_url = "tcp://" + pip + ":" + str(pport)
        print "Connecting to ",    connection_url
        app = qi.Application(["StartActions", "--qi-url=" + connection_url ])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    app.start()
    session = app.session

    init(session)

    app.run()    

    quit()

    time.sleep(1)
    sys.exit(0)

if __name__ == "__main__":
    main()

