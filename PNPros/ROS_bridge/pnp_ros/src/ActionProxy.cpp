#include <sstream>
#include <ros/ros.h>
#include <pnp_ros/names.h>
#include <pnp_ros/ActionProxy.h>

using namespace std;


namespace pnpros
{
    static actionlib::ActionClient<pnp_msgs::PNPAction>* pnpac = NULL;

    ros::Publisher ActionProxy::publisher;
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

        ROS_INFO_STREAM("Creating ActionProxy " << nm);
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

        //I think robot's name should be in plan name but is NOT!!
        if (robotname == "") {
            ros::NodeHandle nh;
            if(nh.hasParam("robotname")) {
              nh.getParam("robotname", robotname);
              ROS_DEBUG_STREAM("Exec: " << robotname << "#" << name << " " << params );
            }
            else ROS_WARN_STREAM("[PNPros]: No robot name defined. You should probably define it by setting the ros patameter \"robotname\"");
          }

          stringstream ssbuf;
          ssbuf << PARAM_PNPACTIONSTATUS << name;
          ros::param::set(ssbuf.str(),"init");

          active=false;

          if (pnpac == NULL) pnpac = new actionlib::ActionClient<pnp_msgs::PNPAction>("PNP");

          while (!pnpac->waitForActionServerToStart(ros::Duration(5.0))) {
              ROS_INFO("Waiting for the PNP action server to come up.");
          }
    }

	ActionProxy::~ActionProxy() {
	    ROS_DEBUG_STREAM("ActionProxy: terminating action " << name);
	    // terminate this action
	    end();
	}
	
    // TODO - not used -> remove
    // static function
/*
    void ActionProxy::actionTerminationCallback(const pnp_msgs::ActionFinished::ConstPtr& message)
	{
		ROS_INFO_STREAM("Deleting " << message->id);
		activeActions.erase(message->id); // activeActions is static
	}
*/
	
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

	void ActionProxy::start()
	{
        ROS_INFO_STREAM("Start: "+robotname+" "+ name + " " + params + " - ID: " + id);

#if USE_MESSAGES	
		activeActions.insert(id);  // ???
		
		pnp_msgs::Action action;
		
		action.id = id;
		action.robotname = robotname;
		action.name = name;
		action.params = params;
		action.function = "start";
		
		publisher.publish(action);
#endif

#if USE_ACTIONLIB
		
		if (!pnpac->waitForActionServerToStart(ros::Duration(0.1)))	{
			cout << "PNP: cannote execute action " << name << endl;
			return;
		}

        pnp_msgs::PNPGoal goal;
        ROS_DEBUG_STREAM("ActionProxy Robotname "<<robotname);
        goal.id = iid;
		goal.robotname = robotname;
		goal.name = name;
		goal.params = params;
		goal.function = "start";
		
		goalhandler = pnpac->sendGoal(goal,boost::bind(&ActionProxy::transitionCb, this,  _1),boost::bind(&ActionProxy::feedbackCb, this, _1, _2));
		ROS_DEBUG_STREAM("ActionGoal sent " << name << "_" << params);
		ros::spinOnce();
		usleep(100e3);

        stringstream ssbuf;
        ssbuf << PARAM_PNPACTIONSTATUS << name;
        ros::param::set(ssbuf.str(),"run");
/*
int 	ACTIVE = 2
int 	DONE = 7
int 	PENDING = 1
int 	PREEMPTING = 6
int 	RECALLING = 5
int 	WAITING_FOR_CANCEL_ACK = 4
int 	WAITING_FOR_GOAL_ACK = 0
int 	WAITING_FOR_RESULT = 3
*/

		while (goalhandler.getCommState() == actionlib::CommState::WAITING_FOR_GOAL_ACK) {
            cout << "### In start function: gh state = " << goalhandler.getCommState().toString() << endl;
  		    goalhandler = pnpac->sendGoal(goal,boost::bind(&ActionProxy::transitionCb, this,  _1),boost::bind(&ActionProxy::feedbackCb, this, _1, _2));
			ros::spinOnce();
			usleep(100e3);
		}

		while (goalhandler.getCommState() == actionlib::CommState::PENDING) {
            cout << "### In start function: gh state = " << goalhandler.getCommState().toString() << endl;
			ros::spinOnce();
			usleep(100e3);
		}

        //cout << "### In start function: gh state = " <<  << endl;

		ROS_DEBUG_STREAM("ActionGoal " << name << "_" << params << " state : " << goalhandler.getCommState().toString());

#endif

        active=true;

    }

    void ActionProxy::end()
    {

        // cerr << "###DEBUG### ActionProxy::end "+robotname+" "+ name + " " + params + " - ID: " + id << endl;


        if (!active) return;

#if USE_MESSAGES
        pnp_msgs::Action action;

        action.id = id;
        action.robotname = robotname;
        action.name = name;
        action.params = params;
        action.function = "end";

        publisher.publish(action);
#endif

#if USE_ACTIONLIB

        if (!pnpac->waitForActionServerToStart(ros::Duration(0.1))) {
            cout << "PNP: cannote terminate action " << name << endl;
            return;
        }

        pnp_msgs::PNPGoal goal;

        goal.id = iid;
        goal.robotname = robotname;
        goal.name = name;
        goal.params = params;
        goal.function = "end";

        goalhandler = pnpac->sendGoal(goal,boost::bind(&ActionProxy::transitionCb, this,  _1),boost::bind(&ActionProxy::feedbackCb, this, _1, _2));

        while (ros::ok() && ((goalhandler.getCommState() == actionlib::CommState::WAITING_FOR_GOAL_ACK) ||
                (goalhandler.getCommState() == actionlib::CommState::PENDING)) //||
                //(goalhandler.getCommState() == actionlib::CommState::ACTIVE)
              )
        {
            // cout << "###DEBUG### end function - inside while: gh state = " << goalhandler.getCommState().toString() << endl;
            usleep(100e3);
        }
        // cout << "###DEBUG### end function - outside while: gh state = " << goalhandler.getCommState().toString() << endl;
#endif

        stringstream ssbuf;
        ssbuf << PARAM_PNPACTIONSTATUS << name;
        ros::param::set(ssbuf.str(),"end");

        ROS_INFO_STREAM("End: "+robotname+" "+ name + " " + params + " - ID: " + id);

        active=false;

    }

    void ActionProxy::interrupt()
    {

        ROS_INFO_STREAM("Interrupt: "+robotname+" "+ name + " " + params + " - ID: " + id);

#if USE_MESSAGES
        pnp_msgs::Action action;

        action.id = id;
        action.robotname = robotname;
		action.name = name;
		action.params = params;
		action.function = "interrupt";

		publisher.publish(action);
#endif

#if USE_ACTIONLIB
        if (pnpac == NULL) pnpac = new actionlib::ActionClient<pnp_msgs::PNPAction>("PNP");

		while (!pnpac->waitForActionServerToStart(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the PNP action server to come up.");
		}

		if (!pnpac->waitForActionServerToStart(ros::Duration(0.1)))
		{
			cout << "PNP: cannote execute action " << name << endl;

			return;
		}

        pnp_msgs::PNPGoal goal;

        goal.id = iid;
        goal.robotname = robotname;
		goal.name = name;
		goal.params = params;
		goal.function = "interrupt";

		goalhandler = pnpac->sendGoal(goal,boost::bind(&ActionProxy::transitionCb, this,  _1),boost::bind(&ActionProxy::feedbackCb, this, _1, _2));
		
		while ((goalhandler.getCommState() == actionlib::CommState::WAITING_FOR_GOAL_ACK) ||
			   (goalhandler.getCommState() == actionlib::CommState::PENDING) // ||
			   // (goalhandler.getCommState() == actionlib::CommState::ACTIVE)
              )
		{
            //cout << "### In interrupt function: gh state = " << goalhandler.getCommState().toString() << endl;
			usleep(100e3);
		}
#endif

        stringstream ssbuf;
        ssbuf << PARAM_PNPACTIONSTATUS << name;
        ros::param::set(ssbuf.str(),"interrupt");

        ROS_INFO_STREAM("Interrupt: "+robotname+" "+ name + " " + params + " - ID: " + id);
	}

	bool ActionProxy::finished()
	{
		actionlib::CommState cs = goalhandler.getCommState();
		
		//cout << "### Finishing " << name << " ? " << cs.toString()  << endl;
		
		if (cs == actionlib::CommState::DONE)
		{
			actionlib::TerminalState ts = goalhandler.getTerminalState();
			
            ROS_DEBUG_STREAM("Finish: " << robotname << " " << name << " " << params << " - ID: " << id << " - " << ts.toString() << " " << ts.getText());
							
			if (ts == actionlib::TerminalState::SUCCEEDED)  return true;
		}
		
		return false;
	}
}

