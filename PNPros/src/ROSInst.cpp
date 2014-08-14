#include "ROSInst.h"
#include "ActionProxy.h"
#include "ROSConds.h"
#include <pnp/basic_plan/xml_plan_instantiator.h>
#include <pnp/pnp_plan.h>
#include <fstream>

using namespace PetriNetPlans;
using namespace std;

namespace pnpros
{
	ROSInst::ROSInst(ExternalConditionChecker* checker, const string& planFolder) : checker(checker), planFolder(planFolder) {;}
	
	ROSInst::~ROSInst() {;}
	
	PnpExecutable* ROSInst::createExecutable(const string& name) throw (runtime_error)
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
			
			// Try an action.
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
}
