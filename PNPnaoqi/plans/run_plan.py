#!/usr/bin/env python

import qi
import argparse
import sys
import time
import threading
import os


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--plan", type=str, default="stop",
                        help="Plan to execute")
    args = parser.parse_args()
    pip = args.pip
    pport = args.pport
    plan = args.plan

    #Starting application
    try:
        connection_url = "tcp://" + pip + ":" + str(pport)
        print "Connecting to ",    connection_url
        app = qi.Application(["Run plan", "--qi-url=" + connection_url ])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    app.start()
    session = app.session

    #Starting services
    memory_service  = session.service("ALMemory")

    print "Starting plan ", plan

    key = "PNP_planToExec"
    memory_service.insertData(key,plan)



if __name__ == "__main__":
    main()

