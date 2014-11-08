#ifndef learnpnp_LearnPlan_h__guard
#define learnpnp_LearnPlan_h__guard

#include <pnp/basic_plan/basic_plan.h>
#include <pnp/pnp_plan.h>
#include <pnp/pnpfwd.h>

#include <pnp/learning_plan/RewardCollector.h>
#include <pnp/learning_plan/Marking.h>
#include <pnp/learning_plan/Controller.h>

#include <pnp/learning_plan/ExpPolicy.h>

#include <set>
#include <vector>

namespace learnpnp {

/**
*\brief A PetriNetPlans::PnpPlan that uses a Reinforcement Learning algorithm to choose among non-deterministic transitions.
* 
* The behaviour of a LearnPlan is the same as PetriNetPlans::PnpPlan as long as no non-deterministic
* choice happens. Where two or more transitions are conflicting, that is
* they are all enabled but the firing of one precludes the firing of the others
* (for instance they all consume the same token from the same place), the Controller chooses the
* next marking.
*
*\author Matteo Leonetti
*/
class LearnPlan : public PetriNetPlans::PnpPlan {


public:

	/**
	*\brief ctor
	*
	*\attention this object becomes the owner of every object passed to the
	* constructor but the RewardCollector.
	*
	*\param ei is the executable instantiator as for PetriNetPlans::PnpPlan
	*\param rc is the reward collector that is also an external condition checker.
	* It is used to accumulate the reward and test the conditions about the environment.
    *\param controller is the component responsible for making the choices at non-deterministic points
	*\param name is the name of this plan
	*/
	LearnPlan(PetriNetPlans::ExecutableInstantiator* ei, 
		learnpnp::RewardCollector* rc, 
		Controller *controller,
		const std::string name = "unnamed");

	LearnPlan(const PetriNetPlans::PnpPlan& networkStructure, Controller *controller);

	/**
	*\brief dtor
	*/
	virtual ~LearnPlan();

	//same as PnpPlan, get the documentation from there.
	virtual std::set<PetriNetPlans::PnpTransition*> getAllEnabledTransitions();
//	virtual std::set<PetriNetPlans::PnpTransition*> getAllEnabledTransitions2();
	virtual void executeStep();

private:
  
	//owns
	Controller *controller;
	bool transitionFired; //updated by getAllEnabledTransitions(), it is
			      // true iff there are enabled transitions that
			      //executeStep() will fire

	std::map< std::string , int> curmar;     /*profiling showed that the calls to currentMarking() are heavy.
																I try to store the marking here at the beginning of
																executeStep() so that I don't have to call currentMarking() too
																often */
	
	//doesn't own
	RewardCollector *collector;
	

	//useful functors to call class functions in std algorithms
	friend class CheckInputPlacesAndAdd;
	friend class CheckForConflictAndAdd;
	friend class CompareSets;
	friend class ValueOfSet;
	
	//test classes
	friend class LearnPlanTest;
	friend class LearnPlanTest2;

	std::vector< std::set<PetriNetPlans::PnpTransition*> > separateConflictingTransitions(
			const std::set<PetriNetPlans::PnpTransition*>& ) ;

	void computeAllTransitionSets(	std::vector< std::set<PetriNetPlans::PnpTransition*> >::iterator current, 
								std::vector< std::set<PetriNetPlans::PnpTransition*> >::iterator end,
								std::set<PetriNetPlans::PnpTransition*> &choice,
								std::set<PetriNetPlans::PnpTransition*> &notAvailable,
								std::vector< std::set<PetriNetPlans::PnpTransition*> > &possibileSets) const;

	std::map<std::string,int> simulateFireing(const std::set<PetriNetPlans::PnpTransition*>&) const;

};


}

#endif
