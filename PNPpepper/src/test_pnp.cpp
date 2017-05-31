#include <iostream>
#include <fstream>

#include <pnp/basic_plan/basic_plan.h>
#include <pnp/pnp_executer.h>
#include <pnp/basic_plan/xml_plan_instantiator.h>

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
		// cout << "No plan found. Trying with an action ..." << endl;
		
		return new ActionProxy(name);
	}
	
	PnpPlan *plan = new PnpPlan(this,checker);
	
	try
	{
		XMLPnpPlanInstantiator planLoader;
		
		planLoader.loadFromPNML(path,plan);
	}
	catch(const runtime_error&)
	{
		string errorString("No action nor plan with name: ");
		
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
	std::cout << "Hello, world!" << std::endl;

	string planFolder=".", planToExec="test";

	PnpExecuter<PnpPlan> *executor = NULL;

	ExternalConditionChecker *conditionChecker = new Conds();

	cout << "Loading plan " << planToExec << " from folder " << planFolder << endl;
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

	if (executor!=NULL) {
		cout << "Exec plan..." << endl;

		executor->setMainPlan(planToExec);

	}

	cout << "End." << endl;

	return 0;
}

