#include <pnp/pnp_action.h>

#include <pnp/utils.h>

#include <cstdlib>

using namespace std;

namespace PetriNetPlans {

void PnpAction::start() { PNP_OUT("starting action");}

/**
*\todo test this
*/
void PnpAction::resume() { PNP_OUT("resuming action");}

void PnpAction::end() {PNP_OUT("ending action"); }

/**
*\todo test this
*/
void PnpAction::interrupt() {PNP_OUT("interrupting action"); }
void PnpAction::fail() { PNP_OUT("failing action");}
void PnpAction::executeStep() {PNP_OUT("executing action step"); }
  
  
void PnpAction::init(const string& params2)  throw(invalid_argument)
{
	initParams();
	string params = params2;
	vector<string> v = tokenize(params, ",");
	for (size_t i = 0; i < v.size(); i++) {
		vector<string> p = tokenize(v[i], "=");
		setParamByName(trim(p[0]), trim(p[1]));
	}
}

void PnpAction::setParamByName(const string& paramName, const string& value) throw(invalid_argument)
{
	map<string, pair<string, void*> >::iterator it = params.find(paramName);
	if (it == params.end()) {
		PNP_OUT("Unknown param '"<<paramName<<"'");
		throw invalid_argument("parameter " + paramName + " does not exist");
	}

	if (it->second.first == "int") *((int*)(it->second.second)) = atoi(value.c_str());
	else if (it->second.first == "double") *((double*)(it->second.second)) = atof(value.c_str());
	else if (it->second.first == "string") *((string*)(it->second.second)) = value;
	else {
		PNP_OUT("Uknown param type '"<<it->second.first<<"'");
		throw invalid_argument("parameter " + paramName + " has the wrong type");
	}
}

map<string, PnpAction*>& getActionDatabase()
{
	static map<string, PnpAction*> actionDatabase;
	return actionDatabase;
}

ActionFactory::ActionFactory(const string& actionName, PnpAction* actionObject)
{
	map<string, PnpAction*>& actionDatabase = getActionDatabase();
	if (actionDatabase.find(actionName) != actionDatabase.end()) {
		PNP_OUT("Action class '"<<actionName<<"' already present in action prototype database");
		return;
	}
	PNP_OUT("Adding action class '"<<actionName<<"' in the action prototype database");
	actionDatabase.insert(make_pair(actionName, actionObject));
}

PnpAction* ActionFactory::forName(const string& actionName, const string& params)
{
	map<string, PnpAction*>& actionDatabase = getActionDatabase();
	map<string, PnpAction*>::iterator it = actionDatabase.find(actionName);
	if (it == actionDatabase.end()) {
		PNP_OUT("Action class '"<<actionName<<"' unknown");
		return 0;
	}
	PnpAction* act = it->second->clone();
	act->init(params);
	return act;
}

} // namespaces
