#include <sstream>
#include <string>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include "NaoqiActionProxy.h"

using namespace std;

// set<string> ActionProxy::activeActions;
unsigned long long ActionProxy::maxID;

extern qi::SessionPtr session;




ActionProxy::ActionProxy(const string& nm)
{
    maxID++;
    iid = maxID;
    stringstream ss; ss << maxID;
    id = ss.str();
    robotname = "";
    string nms = nm;

    cout << "Creating action " << nm << endl;

    size_t k = nm.find('#');
    if (k != string::npos) {
        robotname = nm.substr(0,k);
        nms = nm.substr(k+1);
    }

    k = nms.find('_');
    if (k == string::npos)
        name = nms;
    else {
        name = nms.substr(0,k);
        params = nms.substr(k+1);
    }

    active=false;

	memProxy = session->service("ALMemory");

	string event = "PNP_action";
	memProxy.call<void>("declareEvent", event);

	acb_signal = memProxy.call<qi::AnyObject>("subscriber","PNP_action_result_"+name);

	// memProxy.call<void>("subscribeToEvent", "PNP_action_result_"+name, serviceName, "actionTerminationCallback");

	//acb = memory_service.subscriber("PNP_action_"+actionName)
	//idacb = 
	//acb.connect("PNP_action_result_"+name, actionTerminationCallback);


	signalID = acb_signal.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&ActionProxy::actionTerminationCallback, this))));

}

ActionProxy::~ActionProxy() {
	// terminate this action    
	cout << "ActionProxy: terminating action " << name << endl;  
    // memProxy.call<void>("unsubscribeToEvent", "actionTerminationCallback", "PNP_ActionProxy");

	acb_signal.disconnect(signalID);

    end();
}



void ActionProxy::start()
{
	cout << "ActionProxy: starting action " << name << endl; 
	// send message to start this action 
	qi::AnyObject memProxy = session->service("ALMemory");

	string event = "PNP_action_" + name;
	string value = "start";
	memProxy.call<void>("raiseEvent",event,value);

    active=true;
}

void ActionProxy::end()
{
    if (!active) return;

	// send message to end this action
	string event = "PNP_action_" + name;
	string value = "end";
	memProxy.call<void>("raiseEvent",event,value);

    active=false;
}

void ActionProxy::interrupt()
{
    if (!active) return;

	// send message to interrupt this action
	string event = "PNP_action_" + name;
	string value = "interrupt";
	memProxy.call<void>("raiseEvent",event,value);

    active=false;
}

bool ActionProxy::finished()
{
	return !active;
}

void ActionProxy::actionTerminationCallback()
{
	active=false;
}

