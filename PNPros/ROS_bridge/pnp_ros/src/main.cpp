#include <pnp_ros/ActionProxy.h>
#include <pnp_ros/ROSConds.h>
#include <pnp_ros/ROSInst.h>
#include <pnp_ros/LearnPNP/ROSLearnInstantiator.h>
#include <pnp_ros/LearnPNP/ROSReward.h>
#include <pnp_ros/LearnPNP/World/World.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pnp/basic_plan/basic_plan.h>
#include <pnp/learning_plan/learnPlan.h>
#include <pnp/pnp_executer.h>
#include <pnp_msgs/Action.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>

#include <pnp_ros/connection_observer.h>

using namespace std;
using namespace PetriNetPlans;
using namespace pnpros;
using namespace pnpros::LearnPNP;
using std_msgs::String;

void spinThread()
{
	ros::NodeHandle nh;
	ros::spin();
}

int main(int argc, char** argv) 
{
	ros::init(argc,argv,"pnp_ros");
	
	// Needed by actionclient.
	boost::thread spin_thread(&spinThread);
	
	ros::NodeHandle n, np("~");
	ExternalConditionChecker* conditionChecker;
	string planName = "test1", planFolder = "plans/";
	int episodes, epochs, learningPeriod, samples;
	bool learning = false, logPlaces = false;
	
	np.param("current_plan",planName,string("test1"));
	np.param("plan_folder",planFolder,string("plans/"));
	np.param("learning",learning,false);
	
	cerr << "\033[22;31;1mCurrent plan: \033[0m\033[22;37;1m" << planName << "\033[0m" << endl;
	cerr << "\033[22;31;1mPlan folder: \033[0m\033[22;37;1m" << planFolder << "\033[0m" << endl;
	cerr << "\033[22;31;1mLearning: \033[0m\033[22;37;1m" << (learning ? "Enabled" : "Disabled") << "\033[0m" << endl;
	
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
	
    ActionProxy::publisher = n.advertise<pnp_msgs::Action>("pnp_action",1);
	ros::Publisher currentActivePlacesPublisher = np.advertise<String>("currentActivePlaces",1);
	ros::Subscriber sub = n.subscribe("pnp_action_termination",100,&ActionProxy::actionTerminationCallback);
	
	// Wait for the other modules to subscribe.
	ros::Duration(1).sleep();
	
	if (learning) conditionChecker = new ROSReward();
	else conditionChecker = new ROSConds();
	
	double refreshRate = 10.0; // Hz
	
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
				
				while (!executor.goalReached() && ros::ok())
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
	}
	else
	{
		// The executor owns the instantiator.
        ExecutableInstantiator* instantiator = new ROSInst(conditionChecker,planFolder);

        PnpExecuter<PnpPlan> executor(instantiator);
        ConnectionObserver observer(planName);
        PlanObserver* new_observer = &observer;

        executor.setMainPlan(planName);
        executor.setObserver(new_observer);



		while (ros::ok())
		{
			cerr << "\033[22;37;1mExecuting plan: " << planName << "\033[0m" << endl;
			

			
			while (!executor.goalReached() && ros::ok())
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
		}
	}
	
	// Cleanup.
	delete conditionChecker;
	
	return 0;
}
