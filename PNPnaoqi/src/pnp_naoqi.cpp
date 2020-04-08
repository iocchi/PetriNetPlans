// qibuild make -c linux64 -w ../../Pepper/qi_ws/

#include <iostream>
#include <fstream>
#include <unistd.h>

#include <pnp/basic_plan/basic_plan.h>
#include <pnp/pnp_executer.h>
#include <pnp/basic_plan/xml_plan_instantiator.h>
#include <pnp/connection_observer.h>

#include <qi/session.hpp>
#include <qi/anyobject.hpp>
#include <qi/applicationsession.hpp>

#include <boost/algorithm/string.hpp>

#include "NaoqiActionProxy.h"

using namespace PetriNetPlans;
using namespace std;

// external use in NaoqiActionProxy
qi::SessionPtr session;
qi::AnyObject memProxy;


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
		
		virtual PetriNetPlans::PnpExecutable* createExecutable(const std::string& name); // throw(std::runtime_error)
};



Inst::Inst(ExternalConditionChecker* checker, const string& planFolder) : checker(checker), planFolder(planFolder) 
{}
	
Inst::~Inst() 
{}

PnpExecutable* Inst::createExecutable(const string& name) // throw (runtime_error)
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

    if (atom.substr(0,7)=="timeout") {
        // cerr << "Checking timeout condition: " << atom << endl;

        vector<std::string> v_params;
        boost::split(v_params, atom, boost::is_any_of("_"));
        int n = v_params.size();
        if (n!=3) {
            cerr << "Error in condition " << atom << endl;
            cerr << "Format: timeout_<actionname>_<timeout in seconds> (no parameters of the action)" << atom << endl;
        }
        else {
            string actionname = v_params[1];
            float timeoutValue = atof(v_params[2].c_str());
            if (timeoutValue<=0) {
                cerr << "Error in timeout value " << timeoutValue << " in condition " << atom << endl;
            }
            string action_timestart_key = "NAOqiAction/"+actionname+"/startTime";
            float at = -1;
		    try{
			    at = memProxy.call<float>("getData",action_timestart_key);
		    }
		    catch (const std::exception& e) {
			    cerr << "Cannot find variable " << action_timestart_key << endl;
		    }

            double ct = getCurrentTime();

            //cerr << "Current time: " << ct << " - Action started at time: " << at <<
            //        " - Diff: " << (ct-at) << " - Timeout value: " << timeoutValue << endl;
            r = (ct-at>timeoutValue);
        }
    }

    if (r==-1 && atom.find('@') == std::string::npos) {
        // Try to read condition from ALmemory
 		string key = "PNP_cond_" + atom;
		string val="";
		try{
			val = memProxy.call<string>("getData",key);
		}
		catch (const std::exception& e) {
			cerr << "Cannot find variable " << key << endl;
		}
		if (val=="0" || val=="false") r=0;
		if (val=="1" || val=="true") r=1;
		// printf("Cond: %s value: %s -> %d\n", atom.c_str(), val, r);
    }


    if (r!=-1) {
        result = (r==1);
        ConditionCache[atom]=result;
    }
    else
        result = false;


    // cout <<  "Condition: " << atom << " -> r = " << r << " - result = " << result << endl;

    return result;
}



string planToExec = "", currentPlanName="stop";

string activeplaces_key = "PNP/ActivePlaces";
string currentplan_key = "PNP/CurrentPlan";
string currentaction_key = "PNP/CurrentAction";
string plantoexec_key = "PNP_planToExec";
string pnpnaoqirun_key = "PNP/running";

bool naoqi_ok() {
	string p = memProxy.call<string>("getData",plantoexec_key);
	// cout << "     almem read: " << p << endl;
	if (p!="") {
		planToExec = p;
		memProxy.call<void>("insertData",plantoexec_key,"");
	}

	return true;  // node must still run
}


int main(int argc, char** argv)
{
	cout << "PNP naoqi" << endl;

    string planName = "stop", planFolder = "plans/";
	
	bool use_java_connection = false, autorestart = false;

	string ip = getenv("PEPPER_IP"), port="9559";
	cout << "Connecting to naoqi on " << ip << ":" << port << endl;

    string connection_url = "tcp://"+ip+":"+port;


    qi::ApplicationSession app(argc, argv, 0, connection_url);

    cout << "app ok" << endl;
    app.start();

    cout << "app started" << endl;

    session = app.session();

    // cout << "Session ptr = " << session << endl;


	memProxy = session->service("ALMemory");

    memProxy.call<void>("declareEvent",currentplan_key);
	memProxy.call<void>("insertData",plantoexec_key,"");
	memProxy.call<void>("insertData",pnpnaoqirun_key,"true");

	PnpExecuter<PnpPlan> *executor = NULL;

	ExternalConditionChecker *conditionChecker = new Conds();

	cout << "\033[0mCurrent plan: \033[0m\033[22;32;1m" << planToExec << "\033[0m" << endl;
	cout << "\033[0mPlan folder: \033[0m\033[22;32;1m" << planFolder << "\033[0m" << endl;

	// cout << "Loading plan " << planToExec << " from folder " << planFolder << endl;



	while (naoqi_ok()) {

		// new plan to exec
		if (planToExec!="") {
            if (planToExec=="<currentplan>")
                planName = currentPlanName;
            else
                planName = planToExec;
		  	planToExec = "";
            memProxy.call<void>("raiseEvent",currentplan_key,planName);
		}			


        

		// wait for a plan different from stop
        if (planName=="stop") {
		  cerr << "\033[22;31;1mWaiting for a plan...\033[0m" << endl;

		  while (planToExec=="" && naoqi_ok()) {
		      usleep(250000);
		  }
		}

		else {
			// planName is a new plan to execute (not stop)

			currentPlanName = planName;
		  
            cerr << "\033[0mExecuting plan: \033[22;31;1m" << planName << "\033[0m  autorestart: " << autorestart <<
            " use_java_connection: " << use_java_connection << endl;


			string path = planFolder + "/" + planName + ".pnml";
	
			ifstream fTemp;
			fTemp.open(path.c_str(),ifstream::in);
			fTemp.close();
	
			if (fTemp.fail()) {
				cerr << "\033[22;31;1mERROR - plan " << planName << " not found... executing 'stop' \033[0m" << endl;
				planName = "stop";
			}


			// Instantiate executor and condition checker
			cout << "Creating executable for the plan" << endl;
			try {		
				ExecutableInstantiator* i = new Inst(conditionChecker,planFolder);
				if (i!=NULL) {
			  		executor = new PnpExecuter<PnpPlan>(i);
				}
		   	}
			catch(int e) {
				cout << "ERROR No Instantiator available." << endl;
				planName = "stop";
			}


			if (executor!=NULL) {

				if (use_java_connection)
					cout << "Using GUI execution monitoring\nWaiting for a client to connect on port 47996" << endl;


				// FIX - segfault				
				//PlanObserver* observer = new ConnectionObserver(planName, use_java_connection);
				//executor->setObserver(observer);

		    	executor->setMainPlan(planName);


		    	if (executor->getMainPlanName()!="") {

				    cout << "Starting plan " << executor->getMainPlanName() << endl;
				    //cout << "   goal: " << executor->goalReached() << endl;
				    //cout << "   fail: " << executor->failReached() << endl;

				    while (!executor->goalReached() && !executor->failReached() && naoqi_ok() && planToExec=="")
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

						// cout << "-- active places: " << str_activePlaces << endl;
                        memProxy.call<void>("insertData",activeplaces_key, str_activePlaces);

						executor->execMainPlanStep();

				        usleep(250000);
				    }

                    ActionProxy_endAllActions(); // send 'end' signal to all the active actions

				    if (executor->goalReached()) {
				        cout << "GOAL NODE REACHED!!!" << endl;
				        string activePlaces;
				        activePlaces = "goal";
				        memProxy.call<void>("insertData",activeplaces_key, activePlaces);
				        if (!autorestart)
				          planToExec="stop";
				    }
				    else if (executor->failReached()) {
				        cout << "FAIL NODE REACHED!!!" << endl;
				        string activePlaces;
				        activePlaces = "fail";
				        memProxy.call<void>("insertData",activeplaces_key, activePlaces);
				        if (!autorestart)
				          planToExec="stop";
				    }
				    else {
				        cout << "PLAN STOPPED OR CHANGED!!!" << endl;
				        string activePlaces;
				        activePlaces = "abort";
				        memProxy.call<void>("insertData",activeplaces_key, activePlaces);
				    }

				} // if executor getMainPlanName ...

			} // if executor!=NULL

		} // else

	} // while naoqi_ok()


	// Cleanup.
	delete conditionChecker;
	delete executor;

    memProxy.call<void>("insertData",pnpnaoqirun_key,"false");
	cout << "End." << endl;

	return 0;
}

