#include <pnp_ros/LearnPNP/ROSLearnInstantiator.h>
#include <pnp_ros/LearnPNP/ROSReward.h>
#include <pnp_ros/LearnPNP/World/World.h>
#include <pnp_ros/ActionProxy.h>
#include <pnp/learning_plan/learnPlan.h>
#include <pnp/learning_plan/algo/TDParams.h>

//exploration
#include <pnp/learning_plan/exp/EGreedy.h>
#include <pnp/learning_plan/exp/Exploit.h>

//learning
#include <pnp/learning_plan/algo/TDLambda.h>
#include <pnp/learning_plan/algo/BasicController.h>

using namespace std;
using namespace PetriNetPlans;
using namespace learnpnp;
using namespace pnpros;

namespace pnpros
{
	namespace LearnPNP
	{
		ROSLearnInstantiator::ROSLearnInstantiator(ExternalConditionChecker* conditionChecker,const string& planFolder, bool logPlaces)
							: ROSInst(conditionChecker,planFolder), reward(new ROSReward()), network(0), logPlaces(logPlaces) {;}
		
		ROSLearnInstantiator::~ROSLearnInstantiator()
		{
			if (reward != 0) delete reward;
			if (network != 0) delete network;
		}
		
		Controller* ROSLearnInstantiator::createController(const string& VfunFilePath)
		{
			TDLParams params;
			
			params.alpha = 0.1;
			params.gamma = 1;
			params.lambda = 0.9;
			params.initialValue = 3;
			
			Learner* actualLearner = new TDLambda(VfunFilePath,params);
			
			ExpPolicy* policy;
			
			if (World::w->isLearning()) policy = new EGreedy(0.2,false);
			else
			{
				// When not learning, that is when evaluating, follow the current best policy and do not explore.
				policy = new Exploit();
			}
			
			LoggingController* learnLog = new BasicController(actualLearner,policy);
			
			if (World::w->isLearning() && logPlaces) setLogger(learnLog);
			
			learnLog->setLearning(World::w->isLearning());
			
			return learnLog;
		}
		
		PnpExecutable* ROSLearnInstantiator::createExecutable(const string& name) throw(runtime_error)
		{
			if (name == "fakeplan") return new PnpPlan(this,0,"fake");
			
			string actionName, path;
			ifstream file;
			
			unsigned long index = name.find('#');
			
			actionName = name;
			
			if (index != string::npos)
			{
				// Removing robot information to check if the action is a sub-plan.
				actionName = actionName.substr(index + 1);
			}
			
			path = planFolder + "/" + actionName + ".pnml";
			
			file.open(path.c_str(),ifstream::in);
			file.close();
			
			if (file.fail())
			{
				file.clear(ios::failbit);
				
				World::w->increaseActionValue();
				
				// Try an action.
				return new ActionProxy(name);
			}
			
			string txt = planFolder + "/../log/" + actionName + "_.txt";
			
			Controller* controller = createController(txt);
			
			delete network;
			
			try
			{
				network = new PnpPlan(this,reward);
				
				planLoader.loadFromPNML(path,network);
			}
			catch (const runtime_error&)
			{
				string errorString ("No action nor plan with name: ");
				errorString += name;
				
				throw runtime_error(errorString);
			}
			
			return new LearnPlan(*network,controller);
		}
		
		void ROSLearnInstantiator::setLogger(learnpnp::LoggingController* logController)
		{
			set<string> interestingPlaces;
			
			/// Demo BeeSAFE-LearnPNP with 2 blocks of 2 paths each.
			/*interestingPlaces.insert("p1");
			interestingPlaces.insert("p5");
			interestingPlaces.insert("p6");
			interestingPlaces.insert("p8");
			interestingPlaces.insert("p9");*/
			
			/// Demo BeeSAFE-LearnPNP with 2 blocks of 5 and 4 paths respectively.
			interestingPlaces.insert("p1");
			interestingPlaces.insert("p10");
			interestingPlaces.insert("p5");
			interestingPlaces.insert("p11");
			interestingPlaces.insert("p6");
			interestingPlaces.insert("p12");
			interestingPlaces.insert("p8");
			interestingPlaces.insert("p13");
			interestingPlaces.insert("p14");
			interestingPlaces.insert("p9");
			
			ostringstream fileNameStream;
			
			fileNameStream << planFolder << "/../log/places_" << World::w->currentEpoch() << ".txt";
			
			PlaceLogger logger(fileNameStream.str(),interestingPlaces);
			
			if (World::w->currentEpisode() == 1) logger.initialize();
			
			logController->addLogger(logger);
		}
	}
}
