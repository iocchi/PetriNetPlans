#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pnp/basic_plan/basic_plan.h>
#include <pnp/learning_plan/learnPlan.h>
#include <pnp/pnp_executer.h>
#include <pnp_msgs/Action.h>
#include <pnp_ros/ActionProxy.h>
#include <pnp_ros/ROSConds.h>
#include <pnp_ros/ROSInst.h>
#include <pnp_ros/LearnPNP/ROSLearnInstantiator.h>
#include <pnp_ros/LearnPNP/ROSReward.h>
#include <pnp_ros/LearnPNP/World/World.h>
#include <pnp_ros/names.h>

#include <pnp/connection_observer.h>

using namespace std;
using namespace PetriNetPlans;
using namespace pnpros;
using namespace pnpros::LearnPNP;
using std_msgs::String;


string planFolder = "plans/";


// Global variables
string robot_name = "NONAME";
string planToExec = "";
actionlib::ActionClient<pnp_msgs::PNPAction> *pnpac = NULL;



// rostopic pub /robot_0/planToExec std_msgs/String "data: 'stop'" --once
void planToExecuteCallback(const std_msgs::String::ConstPtr& msg)
{
  planToExec = msg->data;
  ros::param::param<std::string>("~plan_folder",planFolder,string("plans/"));
  ROS_INFO("Plan received from topic %s. Executing plan %s from folder %s ", TOPIC_PLANTOEXEC, planToExec.c_str(), planFolder.c_str());
  
}

void spinThread()
{
	ros::NodeHandle nh;
	ros::spin();
}


void action_cmd_callback(const std_msgs::String::ConstPtr& msg)
{
	stringstream ss(msg->data);
	string action, actionname, actionparams, actioncmd;
	ss >> action;
	ss >> actioncmd;

	// Split action name from parameters
	int k = action.find('_');
    if (k == string::npos) {
        actionname = action;
		actionparams = "";
	}
    else {
        actionname = action.substr(0,k);
        actionparams = action.substr(k+1);
    }
    
    ROS_INFO_STREAM("Action cmd: " << actioncmd << " " << actionname << " " << actionparams);

    
	if (actioncmd=="start" || actioncmd=="end" || actioncmd=="interrupt") {

		pnp_msgs::PNPGoal goal;

        goal.id = 101;
		goal.robotname = robot_name;
		goal.name = actionname;
		goal.params = actionparams;
		goal.function = actioncmd;

        if (pnpac == NULL) pnpac = new actionlib::ActionClient<pnp_msgs::PNPAction>("PNP");

		int cnt=5;
        while (!pnpac->waitForActionServerToStart(ros::Duration(5.0)) && (cnt-->0)) {
            ROS_INFO("pnp_ros actionCmd:: Waiting for the PNP action server to come up.");
        }

		if (pnpac->waitForActionServerToStart(ros::Duration(1.0)))
			pnpac->sendGoal(goal);
		else
			ROS_INFO("pnp_ros actionCmd:: Cannot connect to PNP action server!!!");
	}
	else {
		ROS_WARN_STREAM("UNKNOWN Action cmd: " << actioncmd << " " << actionname);
	}

}


void publish_activePlaces(PnpExecuter<PnpPlan> *executor, ros::Publisher &currentActivePlacesPublisher) {

    String activePlaces;

    vector<string> nepForTest = executor->getNonEmptyPlaces();

    activePlaces.data = "";

    for (vector<string>::const_iterator it = nepForTest.begin(); it != nepForTest.end(); ++it)
    {
        activePlaces.data += *it;
    }

    // also used to notify PNPAS that a PNP step is just over
    currentActivePlacesPublisher.publish(activePlaces);

}


int main(int argc, char** argv) 
{
	ros::init(argc,argv,"pnp_ros");
	
	// Needed by actionclient.
	boost::thread spin_thread(&spinThread);
		
	ros::NodeHandle n, np("~");

    robot_name = "NONAME";
	if (!n.getParam("robot_name",robot_name))
	    if (!n.getParam("robotname",robot_name))
		    n.getParam("tf_prefix", robot_name);

	
    ros::Subscriber planToExecSub = n.subscribe(TOPIC_PLANTOEXEC, 1, planToExecuteCallback);
	
	ExternalConditionChecker* conditionChecker;
    string planName = "stop", currentPlanName="stop";
	int episodes, epochs, learningPeriod, samples;

	bool learning = false, logPlaces = false, autorestart = false;
	bool use_java_connection = false;
	
    np.param(PARAM_PNP_CURRENT_PLAN,planName,string("stop"));
	np.param("plan_folder",planFolder,string("plans/"));
	np.param("learning",learning,false);
	np.param("autorestart",autorestart,false);
	np.param("use_java_connection",use_java_connection,false);
	
	cerr << "\033[22;31;1mCurrent plan: \033[0m\033[22;32;1m" << planName << "\033[0m" << endl;
	cerr << "\033[22;31;1mPlan folder: \033[0m\033[22;32;1m" << planFolder << "\033[0m" << endl;
	cerr << "\033[22;31;1mLearning: \033[0m\033[22;32;1m" << (learning ? "Enabled" : "Disabled") << "\033[0m" << endl;
	
	if (learning)
	{
		np.param("log_places",logPlaces,false);
		np.param("number_of_epochs",epochs,20);
		np.param("number_of_episodes",episodes,10000);
		np.param("number_of_samples",samples,100);
		np.param("learning_period",learningPeriod,200);
		
		cerr << "\033[22;31;1mLogging places: \033[0m\033[22;37;1m" << (logPlaces ? "Enabled" : "Disabled") << "\033[0m" << endl;
		cerr << "\033[22;31;1mNumber of epochs: \033[0m\033[22;37;1m" << epochs << "\033[0m" << endl;
		cerr << "\033[22;31;1mNumber of episodes: \033[0m\033[22;37;1m" << episodes << "\033[0m" << endl;
		cerr << "\033[22;31;1mNumber of samples: \033[0m\033[22;37;1m" << samples << "\033[0m" << endl;
		cerr << "\033[22;31;1mLearning period: \033[0m\033[22;37;1m" << learningPeriod << "\033[0m" << endl;
		
		struct stat temp;
		
		if (stat((planFolder + string("/../log")).c_str(),&temp) == -1)
		{
			mkdir((planFolder + string("/../log")).c_str(),0700);
		}
	}
	
	ActionProxy::publisher = n.advertise<pnp_msgs::Action>(TOPIC_PNPACTION,1);
	ros::Publisher currentActivePlacesPublisher = 
		n.advertise<String>(TOPIC_PNPACTIVEPLACES,1);
	ros::Subscriber sub = 
		n.subscribe(TOPIC_PNPACTIONTERMINATION, 10, &ActionProxy::actionTerminationCallback);
	
	// Wait for the other modules to subscribe.
	ros::Duration(3).sleep();

	// Subscriber for PNP action cmd topic
	ros::Subscriber	action_cmd_sub = 
		n.subscribe(TOPIC_PNPACTIONCMD, 10, &action_cmd_callback);


	// Testing Connection to PNP action server

	ROS_INFO("pnp_ros connecting to action server...");

	if (pnpac==NULL)
	    pnpac = new actionlib::ActionClient<pnp_msgs::PNPAction>("PNP");

	int cnt=5;
    while (!pnpac->waitForActionServerToStart(ros::Duration(5.0)) && (cnt-->0)) {
        ROS_INFO("pnp_ros:: Waiting for the PNP action server to come up.");
    }

	if (pnpac->waitForActionServerToStart(ros::Duration(1.0)))
		ROS_INFO("pnp_ros:: OK. Connected to PNP action server");
	else
		ROS_INFO("pnp_ros:: Cannot connect to PNP action server!!!");


	if (learning) conditionChecker = new ROSReward();
	else conditionChecker = new ROSConds();
	
    double refreshRate = 50.0; // Hz
	
	ros::Rate rate(refreshRate);
	
	srand(time(0));
	
	if (learning)
	{
		// The executor owns the instantiator.
		PnpExecuter<learnpnp::LearnPlan> executor(new ROSLearnInstantiator(conditionChecker,planFolder,logPlaces));
		
		World::w->learningPeriod = learningPeriod;
		World::w->samples = samples;
		
		int totalExperiments = episodes + (episodes / (float) learningPeriod * samples + samples);
		
		for (int epoch = 1; epoch <= epochs; ++epoch)
		{
			ostringstream fileNameStream;
			
			fileNameStream << planFolder << "/../log/reward-" << time(0) << "_" << epoch << ".txt";
			
			ofstream file(fileNameStream.str().c_str());
			
			cerr << "\033[22;31;1mStarting epoch: \033[0m\033[22;37;1m" << epoch << "\033[0m" << endl;
			
			ifstream fileTemp;
			
			fileTemp.open((planFolder + string("/../log/") + planName + string("_.txt")).c_str(),ifstream::in);
			fileTemp.close();
			
			if (!fileTemp.fail())
			{
				// To avoid compilation warning.
				if (system((("rm ") + planFolder + string("/../log/") + planName + string("_.txt")).c_str())) {;}
			}
			
			// Destroy the learner before creating another one.
			executor.setMainPlan("fakeplan");
			
			for (int i = 0; i < totalExperiments; ++i)
			{
				//if (i % 100 == 0)
				{
					cerr << "\033[22;37;1mExecuting plan: " << planName << "\033[0m" << endl;
					cerr << "\033[22;33;1mExperiment " << (i + 1) << " out of " << totalExperiments << "\033[0m" << endl;
				}
				
				srand(time(0));
				
				executor.setMainPlan(planName);
				
				while (!executor.goalReached() && !executor.failReached() && ros::ok())
				{

					String activePlaces;
					
					vector<string> nepForTest = executor.getNonEmptyPlaces();
					
					activePlaces.data = "";
					
					for (vector<string>::const_iterator it = nepForTest.begin(); it != nepForTest.end(); ++it)
					{
						activePlaces.data += *it;
					}
					
					currentActivePlacesPublisher.publish(activePlaces);	


					executor.execMainPlanStep();
					
					rate.sleep();
				}
				
				if (!World::w->isLearning()) file << World::w->totalActions()  << "\t" << World::w->getReward() << endl;
				
				World::w->endEpisode();
			}
			
			World::w->endEpoch();
			
			file.close();
		}
	} // if learning
	else
	{		
        while (ros::ok())
		{
			if (planToExec!="") {
                if (planToExec=="<currentplan>")
                    planName = currentPlanName;
                else
                    planName = planToExec;
			  planToExec = "";
			}			

            if (planName=="stop") {
			  cerr << "\033[22;31;1mWaiting for a plan...\033[0m" << endl;

			  while (planToExec=="" && ros::ok()) {
			      rate.sleep();
			  }

			}
#if 0
		// The executor owns the instantiator.
        ExecutableInstantiator* instantiator = new ROSInst(conditionChecker,planFolder);
        PnpExecuter<PnpPlan> executor(instantiator);
        
        ConnectionObserver observer(planName, use_java_connection);
        PlanObserver* new_observer = &observer;

        executor.setMainPlan(planName);
        executor.setObserver(new_observer);
#endif			
            else {

                currentPlanName = planName;
			  
                cerr << "\033[22;31;1mExecuting plan: " << planName << "\033[0m  autorestart: " << autorestart <<
                " use_java_connection: " << use_java_connection << endl;

                PnpExecuter<PnpPlan> *executor = NULL;

                // The executor owns the instantiator.
                try {
                ExecutableInstantiator* i = new ROSInst(conditionChecker,planFolder);
                if (i!=NULL)
                  executor = new PnpExecuter<PnpPlan>(i);
                }
                catch(int e) {
                    cerr << "No plan found!!!" << endl;
                    planToExec="stop"; continue;
                }

                if (executor!=NULL) {

                    if (use_java_connection)
                        cout << "Using GUI execution monitoring\nWaiting for a client to connect on port 47996" << endl;

                    ConnectionObserver observer(planName, use_java_connection);
                    executor->setMainPlan(planName);

                    PlanObserver* new_observer = &observer;
                    executor->setObserver(new_observer);

                    if (executor->getMainPlanName()=="") {
                        planToExec="stop";
                    }
                    else {
                        np.setParam(PARAM_PNP_CURRENT_PLAN,planName);
                        cout << "Starting " << executor->getMainPlanName() << endl;

                        String activePlaces;
                        activePlaces.data = "init";
                        currentActivePlacesPublisher.publish(activePlaces);

                        while (!executor->goalReached() && !executor->failReached() && ros::ok() && planToExec=="")
                        {

                            executor->execMainPlanStep();


                            publish_activePlaces(executor, currentActivePlacesPublisher);

/*
                            String activePlaces;

                            vector<string> nepForTest = executor->getNonEmptyPlaces();

                            activePlaces.data = "";

                            for (vector<string>::const_iterator it = nepForTest.begin(); it != nepForTest.end(); ++it)
                            {
                                activePlaces.data += *it;
                            }

                            // also used to notify PNPAS that a PNP step is just over
                            currentActivePlacesPublisher.publish(activePlaces);
*/

                            rate.sleep();
                        } // while

                        if (executor->goalReached()) {
                            cout << "GOAL NODE REACHED!!!" << endl;
                            String activePlaces;
                            activePlaces.data = "goal";
                            currentActivePlacesPublisher.publish(activePlaces);
                            if (!autorestart)
                              planToExec="stop";
                        }
                        else if (executor->failReached()) {
                            cout << "FAIL NODE REACHED!!!" << endl;
                            String activePlaces;
                            activePlaces.data = "fail";
                            currentActivePlacesPublisher.publish(activePlaces);
                            if (!autorestart)
                              planToExec="stop";
                        }
                        else {
                            cout << "PLAN STOPPED OR CHANGED!!!" << endl;
                            String activePlaces;
                            activePlaces.data = "abort";
                            currentActivePlacesPublisher.publish(activePlaces);
                        }

                    } // if executor getMainPlanName ...

                    delete executor;

                } // if executor!=NULL

            } // else
		} // while
	}
	
	// Cleanup.
	delete conditionChecker;
	
	return 0;
}

