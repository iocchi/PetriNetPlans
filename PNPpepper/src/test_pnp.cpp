#include <iostream>
#include <fstream>
#include <unistd.h>

#include <pnp/basic_plan/basic_plan.h>
#include <pnp/pnp_executer.h>
#include <pnp/basic_plan/xml_plan_instantiator.h>
#include <pnp/connection_observer.h>

#include "ActionProxy.h"

using namespace PetriNetPlans;
using namespace std;

class Inst : public PetriNetPlans::ExecutableInstantiator
{
	private:
		// Does not own.
		PetriNetPlans::ExternalConditionChecker* checker;
		
	protected:
		const std::string& planFolder;
		
	public:
		explicit Inst(PetriNetPlans::ExternalConditionChecker*,const std::string& planFolder);
		
		virtual ~Inst();
		
		virtual PetriNetPlans::PnpExecutable* createExecutable(const std::string& name) throw(std::runtime_error);
};



Inst::Inst(ExternalConditionChecker* checker, const string& planFolder) : checker(checker), planFolder(planFolder) 
{}
	
Inst::~Inst() 
{}

PnpExecutable* Inst::createExecutable(const string& name) throw (runtime_error)
{
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
		// cout << "  -- No plan found. Trying with an action ..." << endl;
		return new ActionProxy(name);
	}
	
	PnpPlan *plan = new PnpPlan(this,checker);
	
	try
	{
		XMLPnpPlanInstantiator planLoader;
		planLoader.loadFromPNML(path,plan);
		// cout << "  -- Plan found ... " << plan << endl;
	}
	catch(const runtime_error&)
	{
		string errorString("  -- No action nor plan with name: ");
		
		errorString += actionName;
		
		throw runtime_error(errorString);
	}
	
	return plan;
}



class Conds : public PetriNetPlans::ExternalConditionChecker
{
	private:
        // map<string,int> ConditionCache; - defined in ExternalConditionChecker
		
	public:
		Conds();
		
		bool evaluateAtomicExternalCondition(const std::string& atom);

};


Conds::Conds()
{}

bool Conds::evaluateAtomicExternalCondition(const string& atom)
{

    int r=-1; bool result=false;

    //cout <<  "    evaluateAtomicExternalCondition: " << atom << " begin ... " << endl;

    // This is necessary because multiple calls to the same condition can happen
    if (ConditionCache.find(atom) != ConditionCache.end()) {
        result = ConditionCache[atom];
        //cout <<  "    evaluateAtomicExternalCondition: " << atom << " CACHED result = " << result << " ... end" << endl;
        return result;
    }

    if (atom.find('@') == std::string::npos) {
        // Try to read condition from parameters
    }

    if (r==-1) {
        // Call the service

        // TODO !!!
        bool sr = false;

        if (sr)
        {
            // printf("Cond: %s value: %d ", atom.c_str(), srv.response.truth_value);
            r = true;
        }
        else
        {
            result = false;
        }
    }

    if (r!=-1) {
        result = (r==1);
        ConditionCache[atom]=result;
    }
    else
        result = false;


    //cout <<  "    evaluateAtomicExternalCondition: " << atom << " r = " << r << " - result = " << result << " ... end" << endl;

    return result;
}



int main()
{
	std::cout << "Hello, world! This is a test of PNP execution!" << std::endl;

	string planFolder=".", planToExec="test";
	bool use_java_connection = false, autorestart = false;

	PnpExecuter<PnpPlan> *executor = NULL;

	ExternalConditionChecker *conditionChecker = new Conds();

	cout << "\033[0mCurrent plan: \033[0m\033[22;32;1m" << planToExec << "\033[0m" << endl;
	cout << "\033[0mPlan folder: \033[0m\033[22;32;1m" << planFolder << "\033[0m" << endl;

	// cout << "Loading plan " << planToExec << " from folder " << planFolder << endl;

	cout << "Creating executable for the plan" << endl;

	try {		
	    ExecutableInstantiator* i = new Inst(conditionChecker,planFolder);
    	if (i!=NULL) {
      		executor = new PnpExecuter<PnpPlan>(i);
		}
   	}
    catch(int e) {
    	cout << "ERROR No Instantiator available." << endl;
    	planToExec="stop"; 
    }

	string planName = planToExec;

	if (executor!=NULL) {

        if (use_java_connection)
            cout << "Using GUI execution monitoring\nWaiting for a client to connect on port 47996" << endl;

        //ConnectionObserver observer(planName, use_java_connection);
        //PlanObserver* new_observer = &observer;

        executor->setMainPlan(planName);
        //executor->setObserver(new_observer);

        if (executor->getMainPlanName()!="") {

            cout << "Starting plan " << executor->getMainPlanName() << endl;
            //cout << "   goal: " << executor->goalReached() << endl;
            //cout << "   fail: " << executor->failReached() << endl;

            while (!executor->goalReached() && !executor->failReached())
            {

                
                string str_activePlaces;

                vector<string> nepForTest = executor->getNonEmptyPlaces();

                str_activePlaces = "";

                for (vector<string>::const_iterator it = nepForTest.begin(); it != nepForTest.end(); ++it)
                {
                    str_activePlaces += *it;
                }
				if (str_activePlaces == "") 
					str_activePlaces = "init;";

				cout << "-- active places: " << str_activePlaces << endl;
                // also used to notify PNPAS that a PNP step is just over
                // currentActivePlacesPublisher.publish(activePlaces);

				executor->execMainPlanStep();

                usleep(100000);
            }

            if (executor->goalReached()) {
                cout << "GOAL NODE REACHED!!!" << endl;
                string activePlaces;
                activePlaces = "goal";
                //currentActivePlacesPublisher.publish(activePlaces);
                if (!autorestart)
                  planToExec="stop";
            }
            else if (executor->failReached()) {
                cout << "FAIL NODE REACHED!!!" << endl;
                string activePlaces;
                activePlaces = "fail";
                //currentActivePlacesPublisher.publish(activePlaces);
                if (!autorestart)
                  planToExec="stop";
            }
            else {
                cout << "PLAN STOPPED OR CHANGED!!!" << endl;
                string activePlaces;
                activePlaces = "abort";
                //currentActivePlacesPublisher.publish(activePlaces);
            }

        } // if executor getMainPlanName ...

        delete executor;

    } // if executor!=NULL


	cout << "End." << endl;

	return 0;
}

