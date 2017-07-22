#include <sstream>
#include <string>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include "NaoqiActionProxy.h"

using namespace std;

// set<string> ActionProxy::;
unsigned long long ActionProxy::maxID;

extern qi::SessionPtr session;

static double global_time0 = -1;

double getCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    double v = (static_cast<double>(tv.tv_sec)+1e-6*static_cast<double>(tv.tv_usec));
    if (global_time0<0) {
        global_time0=v;
    }
    return (v-global_time0);
}


ActionProxy::ActionProxy(const string& nm)
{
    maxID++;
    iid = maxID;
    stringstream ss; ss << maxID;
    id = ss.str();
    robotname = "";
    string nms = nm;

    cout << "ActionProxy:: ++ new action " << nm << endl;

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

	signalID = acb_signal.connect("signal", (boost::function<void(qi::AnyValue)>(boost::bind(&ActionProxy::actionTerminationCallback, this))));

}

ActionProxy::~ActionProxy() {
	// terminate this action    
	cout << "ActionProxy:: -- delete action " << name << " " << params << endl;  
    // memProxy.call<void>("unsubscribeToEvent", "actionTerminationCallback", "PNP_ActionProxy");

	acb_signal.disconnect(signalID);

    end();

    activeActions.erase(this);
}




void ActionProxy::start()
{
	// cout << "ActionProxy:: starting action " << name << " " << params << endl; 
	// send message to start this action 
	qi::AnyObject memProxy = session->service("ALMemory");

	string event = "PNP_action" ;
	string value = "start "+ name+" "+params;
	memProxy.call<void>("raiseEvent",event,value);

    string action_timestart_key = "NAOqiAction/"+name+"/startTime";
    float timeValue = getCurrentTime();
	memProxy.call<void>("insertData",action_timestart_key,timeValue);

    cout << "ActionProxy::     " << name << "_" << params << " started at time " << timeValue << endl;

    activeActions.insert(this);

    active=true;
}

void ActionProxy::end()
{
    if (!active) return;

	// send message to end this action
	string event = "PNP_action";
	string value = "end "+name;
	memProxy.call<void>("raiseEvent",event,value);

    cout << "ActionProxy::     " << name << "_" << params << " ended at time " << getCurrentTime() << endl;

    active=false;
}

void ActionProxy::interrupt()
{
    if (!active) return;

	// send message to interrupt this action
	string event = "PNP_action";
	string value = "interrupt " + name;
	memProxy.call<void>("raiseEvent",event,value);

    cout << "ActionProxy::     " << name << "_" << params << " interrupted at time " << getCurrentTime() << endl;

    active=false;
}

bool ActionProxy::finished()
{
	return !active;
}

void ActionProxy::actionTerminationCallback()
{
    cout << "ActionProxy::     " << name << "_" << params << " completed at time " << getCurrentTime() << endl;
	active=false;
}


void ActionProxy_endAllActions() 
{
    std::set<ActionProxy *>::iterator it = activeActions.begin();
    while (it != activeActions.end()) {
        ActionProxy *a_ptr = *it;
        a_ptr->end();
        it++;
    }
    
}


