#!/usr/bin/env python

import qi
import argparse
import sys
import os
import time

from naoqi import ALProxy

# colored prints
class tcol:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# list all files in here you don't want autoloaded
# (without .py at the end, just the name)
# blacklisted_actions = ['personhere']
blacklisted_actions = []

# this is a list of tuples (Proxy, Subscriber) to 
# initialise services at the beginning
behaviours = [
#    ('ALFaceDetection', 'Face_Behavior'),
#    ('ALFaceCharacteristics', 'Char_Behavior'),
#    ('ALPeoplePerception', 'People_Behavior'),
#    ('ALSittingPeopleDetection', 'Sitting_Behavior'),
#    ('ALSoundLocalization', 'Sound_Behavior'),
#    ('ALMotion', 'Motion_Behavior'),
#    ('ALWavingDetection', 'Waving_Behavior'),
#    ('ALAnimationPlayer', None)
]

actions_running = []


def find_and_import():
    """
    find all python files in the same directory as
    this file and loads them dynamically
    """
    import pkgutil

    p = os.path.realpath(__file__)
    print p
    modules = []
    failed_loads = []
    for loader, name, is_pkg in pkgutil.walk_packages(p):
        if name == "init_actions" or name in blacklisted_actions:
            continue
        try:
            modules.append(loader.find_module(name).load_module(name))
        except Exception as e:
            print("*** exception importing module %s: %s ***"
                  % (name, str(e)))
            failed_loads.append(name)
    if len(failed_loads) > 0:
        print "*** failed loads: \n  ! %s" % '\n  ! '.join(failed_loads)

    return modules


def init_actions(modules, session=None):
    """
    initialises all modules by calling the init functions.
    Important: Crashes gracefully should one action not be initialised,
    but summarises the failed ones at the end
    """
    # Later in the code when processing is required:
    failed_inits = []
    for module in modules:
        # if it has an init function, call it:
        if "init" in module.__dict__:
            try:
                module.init(session)
                actions_running.append(module.__name__)
            except Exception as e:
                print("*** exception quitting action %s: %s ***"
                      % (module.__name__, str(e)))
                failed_inits.append(module.__name__)
    if len(actions_running) > 0:
        print("%s+++ actions running:: \n  + %s%s" %
            (tcol.OKGREEN, '\n  + '.join(actions_running), tcol.ENDC))
    if len(failed_inits) > 0:
        print("%s*** failed initialisations: \n  ! %s%s" %
            (tcol.FAIL, '\n  ! '.join(failed_inits), tcol.ENDC))
    return failed_inits


def quit_actions(modules):
    """
    quit all actions, crash gracefully if some don't work
    """
    # Later in the code when processing is required:
    for module in modules:
        # if it has an init function, call it:
        if "quit" in module.__dict__:
            try:
                module.quit()
            except Exception as e:
                print("*** exception initialising action %s: %s ***"
                      % (module.__name__, str(e)))


def start_behaviors():
    global pip, pport

    print "==================================="
    print "   Starting background behaviors   "
    print "==================================="

    for b in behaviours:
        proxy_name = b[0]
        subscriber_name = b[1]
        try:
            proxy = ALProxy(proxy_name, pip, pport)
            if subscriber_name is not None:
                proxy.subscribe(subscriber_name, 500, 0.0)
        except Exception as e:
            print("*** exception starting behaviour %s: %s ***"
                  % (proxy_name, str(e)))


def quit_behaviors():
    global pip, pport

    print "==================================="
    print "   Quitting background behaviors   "
    print "==================================="

    for b in behaviours:
        proxy_name = b[0]
        subscriber_name = b[1]
        try:
            proxy = ALProxy(proxy_name, pip, pport)
            if subscriber_name is not None:
                proxy.unsubscribe(subscriber_name)
        except Exception as e:
            print("*** exception starting behaviour %s: %s ***"
                  % (proxy_name, str(e)))


def init(session, modules):
    start_behaviors()
    init_actions(modules, session)


def quit(modules):
    quit_actions(modules)
    quit_behaviors()


def main():
    global memory_service, pip, pport
    parser = argparse.ArgumentParser()
    parser.add_argument("--pip", type=str, default=os.environ['PEPPER_IP'],
                        help="Robot IP address.  On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--pport", type=int, default=9559,
                        help="Naoqi port number")
    args = parser.parse_args()
    pip = args.pip
    pport = args.pport

    # Starting application
    try:
        connection_url = "tcp://" + pip + ":" + str(pport)
        print "Connecting to ", connection_url
        app = qi.Application(["StartActions", "--qi-url=" + connection_url ])
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + pip + "\" on port " + str(pport) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    app.start()
    session = app.session
    modules = find_and_import()
    time.sleep(1)
    init(session, modules)

    app.run()

    quit(modules)

    time.sleep(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
