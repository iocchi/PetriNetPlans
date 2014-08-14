
#include "plan_observer.h"

#include <stdexcept>
#include <algorithm>
#include <functional>
#include <cstdio>

namespace PetriNetPlans {
	
template<typename PnpPlaceClass, typename PnpTransitionClass> std::map<std::string,std::string> PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::activePlaces;

template<typename PnpPlaceClass, typename PnpTransitionClass>
    PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::PnpPlanTemplate(
                      ExecutableInstantiator* ei, ExternalConditionChecker* cc, const std::string name) :
	planName(name), executableInstantiator(ei), conditionChecker(cc), observer(NULL)
{ }

template<typename PnpPlaceClass, typename PnpTransitionClass>
PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::~PnpPlanTemplate()
{
	PNP_OUT("Destroying plan '"<<getPlanName()<<"'");
	for (std::map<std::string, PnpExecutable*>::iterator it = inactiveExecutables.begin();
	it != inactiveExecutables.end(); ++it) {
		PNP_OUT("  Destroying sub-executable '"<<it->first<<"'");
		delete it->second;
	}

	for (std::map<std::string, PnpExecutable*>::iterator it = activeExecutables.begin();
	it != activeExecutables.end(); ++it) {
		PNP_OUT("  Destroying sub-executable '"<<it->first<<"'");
		delete it->second;
	}
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
bool PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::isInGoalState() const
{
	bool noGoalMarking = true;
	std::set<PnpPlace*>::iterator p_iter=PetriNet<PnpPlaceClass, PnpTransitionClass>::places.begin();
	for(;p_iter!=PetriNet<PnpPlaceClass, PnpTransitionClass>::places.end();p_iter++) {
		PnpPlace* p = *p_iter;
		if (p->goalMarking == -1) continue;
		else {
			noGoalMarking = false;
			if (p->currentMarking!=(size_t)p->goalMarking) return false;
		}
	}
	if (noGoalMarking) return false;
	else
	{
		std::map<std::string,std::string>::iterator it = activePlaces.find(getPlanName());
		
		if (it != activePlaces.end()) activePlaces.erase(it);
		
		return true;
	}
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
bool PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::isInFailState() const
{
	bool noFailMarking = true;
	std::set<PnpPlace*>::iterator p_iter=PetriNet<PnpPlaceClass, PnpTransitionClass>::places.begin();
	for(;p_iter!=PetriNet<PnpPlaceClass, PnpTransitionClass>::places.end();p_iter++) {
		PnpPlace* p = *p_iter;
		if (p->failMarking == -1) continue;
		else {
			noFailMarking = false;
			if (p->currentMarking!=(size_t)p->failMarking) return false;
		}
	}
	if (noFailMarking) return false;
	else return true;
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
void PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::start()
{
	this->resetInitialMarking();
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
void PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::resume()
{
	// no action
}

template <typename Key, typename Value, typename Return>
class MapSecondArgument : public  std::unary_function<std::pair<Key,Value>, void> {

	std::mem_fun_t<Return, Value> toCall;
	public:
	MapSecondArgument(const std::mem_fun_t<Return, Value>& toCall) : toCall(toCall) {}

	void operator () (const std::pair<Key, Value*>& pair) {
		toCall(pair.second);
	}

};

template<typename PnpPlaceClass, typename PnpTransitionClass>
void PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::end()
{
	std::map<std::string,std::string>::iterator it = activePlaces.find(getPlanName());
	
	if (it != activePlaces.end()) activePlaces.erase(it);
	
	//call end() on each active executable
	MapSecondArgument<std::string, PnpExecutable, void> callEnd(std::mem_fun(&PnpExecutable::end));
	for_each(activeExecutables.begin(), activeExecutables.end(), callEnd);
	
	//move them to the inactive map
	inactiveExecutables.insert(activeExecutables.begin(), activeExecutables.end());
	activeExecutables.clear();
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
void PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::interrupt()
{
	std::map<std::string,std::string>::iterator it = activePlaces.find(getPlanName());
	
	if (it != activePlaces.end()) activePlaces.erase(it);
	
	//call end() on each active executable
	MapSecondArgument<std::string, PnpExecutable, void> callEnd(std::mem_fun(&PnpExecutable::end));
	for_each(activeExecutables.begin(), activeExecutables.end(), callEnd);
	
	//move them to the inactive map
	inactiveExecutables.insert(activeExecutables.begin(), activeExecutables.end());
	activeExecutables.clear();
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
void PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::fail()
{
	std::map<std::string,std::string>::iterator it = activePlaces.find(getPlanName());
	
	if (it != activePlaces.end()) activePlaces.erase(it);
	
	//call end() on each active executable
	MapSecondArgument<std::string, PnpExecutable, void> callEnd(std::mem_fun(&PnpExecutable::end));
	for_each(activeExecutables.begin(), activeExecutables.end(), callEnd);
	
	//move them to the inactive map
	inactiveExecutables.insert(activeExecutables.begin(), activeExecutables.end());
	activeExecutables.clear();
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
void PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::executeStep()
{
	PNP_OUT("\nEXECUTESTEP of plan '"<<getPlanName()<<"'");

	if (isInGoalState())
	{
		std::map<std::string,std::string>::iterator it = activePlaces.find(getPlanName());
		
		if (it != activePlaces.end()) activePlaces.erase(it);
		
		PNP_OUT("Goal reached!");
		
		return;
	}
	else if (isInFailState())
	{
		std::map<std::string,std::string>::iterator it = activePlaces.find(getPlanName());
		
		if (it != activePlaces.end()) activePlaces.erase(it);
		
		PNP_OUT("Plan failed");
		
		return;
	}
	
	std::map<std::string,std::string>::iterator it = activePlaces.find(getPlanName());
	
	if (it != activePlaces.end()) activePlaces.erase(it);
	
	std::set<PnpPlace*> nepForTest = this->getNonEmptyPlaces();
	std::string places;
	
	places = "";
	
	for (std::set<PnpPlace*>::iterator it = nepForTest.begin(); it != nepForTest.end(); ++it)
	{
		places += (*it)->pnmlString + ";";
	}
	
	activePlaces.insert(make_pair(getPlanName(),places));
	
	#if PNP_OUTPUT_ENABLED
	std::set<PnpPlace*> nepForTest2 = this->getNonEmptyPlaces();
	PNP_OUT("Current active places:");
	
	for (std::set<PnpPlace*>::iterator it = nepForTest2.begin(); it != nepForTest2.end(); ++it) {
		PNP_OUT("  " << (*it)->nodeId << " \"" << (*it)->pnmlString << "\"");
	}
	#endif

	executeAllActiveActions();

	fireAllEnabledTransitions();

	if(this->observer != NULL)
		this->observer->markingChanged(this->currentMarking());

}

template<typename PnpPlaceClass, typename PnpTransitionClass>
void  PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::fireAllEnabledTransitions() {
	
	std::set<PnpTransition*> enabledTr = getAllEnabledTransitions();

	#if PNP_OUTPUT_ENABLED
	PNP_OUT("Enabled transitions:");
	for (std::set<PnpTransition*>::iterator it = enabledTr.begin();
	it != enabledTr.end(); it++) {
		PNP_OUT("  " << (*it)->nodeId << " \"" << (*it)->pnmlString << "\"");
	}
	#endif
	
	if (enabledTr.empty()) {
		PNP_OUT("No enabled transition");
		return;
	}
	
	PNP_OUT("FIRING TRANSITIONS");
	// Transitions firing
	for (std::set<PnpTransition*>::iterator t_iter=enabledTr.begin();
	t_iter!=enabledTr.end(); t_iter++) {
		PnpTransition* t = *t_iter;
		if (this->isEnabled(t)) {	//can this be possible? legged say: IT IS!!!!
			//fireing some transitions may disable others
			if (!this->fire(t)) {
				PNP_ERR("Cannot fire the Transition!");
			}
			else {
				for (size_t i = 0; i < t->functionsToCall.size(); i++) {
					size_t q = t->functionsToCall[i].find_last_of(".");
					if (q == std::string::npos) {
//					std::vector<std::string> v = tokenize(t->functionsToCall[i], ".");
//					if (v.size() != 2) {
						PNP_ERR("Malformed function call (2): '"<<t->functionsToCall[i]<<"'");
					}
					else {
						std::vector<std::string> v;
						v.push_back(t->functionsToCall[i].substr(0, q));
						v.push_back(t->functionsToCall[i].substr(q+1));
						PnpExecutable* a = getExecutable(v[0]);
						if (!a) {
							PNP_ERR("Unknown executable name '"<<v[0]<<"' in function call '"
							<<t->functionsToCall[i]<<"'");
						}
						else {
							if (v[1] == "start" || v[1] == "resume" || v[1] == "end"
								|| v[1] == "fail" || v[1] == "interrupt") {
								PNP_OUT("Executing '"<<t->functionsToCall[i]<<"'");
							}
							if (v[1] == "start") a->start();
							else if (v[1] == "resume") a->resume();
							else if (v[1] == "end") a->end();
							else if (v[1] == "interrupt") a->interrupt();
							else if (v[1] == "fail") a->fail();
							else {
								PNP_ERR("Unknown function name '"<<v[1]<<"' in function call '"
								<<t->functionsToCall[i]<<"'");
							}
							if (v[1] == "end" || v[1] == "fail") {
								
								#if DELETE_CONCLUDED_EXECUTABLES
								PNP_OUT("******* I can destroy '"<<v[0]<<"'");
								activeExecutables.erase(v[0]);
								delete a;
								#else
								PNP_OUT("******* I won't destroy '"<<v[0]<<"'");
								inactiveExecutables.insert(make_pair(v[0], a));
								activeExecutables.erase(v[0]);
								#endif
							}
						}
					}
				}
			}
		}
	}
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
void PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>::executeAllActiveActions() {
	std::set<PnpPlace*> nep = this->getNonEmptyPlaces();
	
	PNP_OUT("EXECUTING NONEMPTY PLACES");
	for (std::set<PnpPlace*>::iterator p_iter=nep.begin();
	p_iter!=nep.end();p_iter++) {
		PnpPlace* p = *p_iter;
		if (p->functionToCall == "") continue;
		std::vector<std::string> v = tokenize(p->functionToCall, ".");
		if (v.size() != 2) {
			PNP_ERR("Malformed function call (1): '"<<p->functionToCall<<"'");
		}
		else {
			PnpExecutable* a = getExecutable(v[0]);
			if (!a) {
				PNP_ERR("Unknown executable '"<<v[0]<<"' in function call '"<<p->functionToCall<<"'");
			}
			else {
				if (v[1] != "exec") {
					PNP_ERR("Unknown function '"<<v[1]<<"' in function call '"<<p->functionToCall<<"'");
				}
				else {
					PNP_OUT("Executing '"<<p->functionToCall<<"'");
					a->executeStep();
				}
			}
		}
	}
}


template<typename PnpPlaceClass, typename PnpTransitionClass>
std::set<PnpTransitionClass*> PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>
	::getAllEnabledTransitions()
{
	std::set<PnpTransitionClass*> r =
		PetriNet<PnpPlaceClass, PnpTransitionClass>::getAllEnabledTransitions();
		
	for (typename std::set<PnpTransitionClass*>::iterator it = r.begin(); it != r.end(); ) {

		if ((*it)->guardCondition != "") {

			bool guardRes = evaluateCondition("("+(*it)->guardCondition+")");

			PNP_OUT("Checking guard '"<<(*it)->guardCondition<<"': "
				<<guardRes);
			if (!guardRes) r.erase(it++);
			else ++it;

		}
		else ++it;
	}
	return r;
}

/**
*\todo consider making this function private
*/
template<typename PnpPlaceClass, typename PnpTransitionClass>
PnpExecutable* PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>
	::getExecutable(const std::string& name)
{
	typename std::map<std::string, PnpExecutable*>::iterator it = inactiveExecutables.find(name);
	if (it != inactiveExecutables.end()) {
		
		activeExecutables.insert(make_pair(name,it->second)); 
		PnpExecutable *exec = it->second;
		inactiveExecutables.erase(it);
		
		return  exec;
	}

	it = activeExecutables.find(name);
	if (it != activeExecutables.end()) return it->second;

	try {
			PnpExecutable* ea = executableInstantiator->createExecutable ( name );
			activeExecutables.insert ( make_pair ( name, ea ) );
			return ea;

	} catch ( std::runtime_error& error ) {
			PNP_ERR ( error.what() );
			return 0;
		}
}



template<typename PnpPlaceClass, typename PnpTransitionClass>
bool PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>
	::evaluateAtomicCondition(const std::string& atom)
{
	std::vector<std::string> toks = tokenize(atom, ".");
	if (toks.size() == 1) {
		return conditionChecker->evaluateAtomicExternalCondition(atom);
	}
	else if (toks.size() == 2) {
		PnpExecutable* e = getExecutable(toks[0]);
		return e->getInternalConditionValue(toks[1]);
	}
	else {
		PNP_ERR("Malformed atomic condition '"<<atom<<"'");
		return false;
	}
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
void PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>
::setObserver( PlanObserver *newObserver) {

	delete this->observer;
	this->observer = newObserver;
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
PlanObserver *PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>
::removeObserver() {

	PlanObserver *ret = this->observer;
	delete this->observer;
	this->observer = NULL;
	return ret;
}

template<typename PnpPlaceClass, typename PnpTransitionClass>
std::map<std::string,int> PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass>
::currentMarking() const {
	
	std::map<std::string,int> marking;
	typename std::set<PnpPlaceClass*>::const_iterator place = this->places.begin();
	
	for (; place != this->places.end(); ++place) {
		marking.insert(make_pair((*place)->nodeId, (*place)->currentMarking));
	}
	
	return marking;
}

}//namespace 
