#include <sstream>

#include "ActionProxy.h"

using namespace std;


set<string> ActionProxy::activeActions;
unsigned long long ActionProxy::maxID;

ActionProxy::ActionProxy(const string& nm)
{
    maxID++;
    iid = maxID;
    stringstream ss; ss << maxID;
    id = ss.str();
    robotname = "";
    string nms = nm;

    // cerr << "Parsing: " << nm << " " << nm.find('_') << endl;

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

}

ActionProxy::~ActionProxy() {
	// terminate this action    
	cout << "ActionProxy: terminating action " << name << endl;    
    end();
}

/*
void ActionProxy::actionTerminationCallback()
{
	//cout << "Deleting " << message->id << endl;
	//activeActions.erase(message->id);
}

void ActionProxy::transitionCb(actionlib::ClientGoalHandle<pnp_msgs::PNPAction> gh)
{
	// ROS_INFO("Goal transition...");
	// TerminalState ts =	gh.getTerminalState ();
	// if (ts ==  actionlib::TerminalState::SUCCEEDED)
	//    activeActions.erase(id);
}

// Called every time feedback is received for the goal
void ActionProxy::feedbackCb(actionlib::ClientGoalHandle<pnp_msgs::PNPAction> gh, const pnp_msgs::PNPFeedbackConstPtr& feedback)
{
	//cerr << "Feedback: " << feedback->feedback << endl;
}
*/


void ActionProxy::start()
{

    active=true;

}

void ActionProxy::end()
{
    if (!active) return;
    active=false;

}

void ActionProxy::interrupt()
{
}

bool ActionProxy::finished()
{
	return false;
}

