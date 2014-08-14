/*
 * pnp_plan.h
 * authors: Vittorio Amos Ziparo and Daniele Calisi
 */

#ifndef PNP_PLAN_H
#define PNP_PLAN_H

#include <string>
#include <map>
#include <set>

// #ifdef HAVE_LIBXML2
// #include <libxml/xmlmemory.h>
// #include <libxml/parser.h>
// #endif

#include <pnp/config.h>
#include <pnp/petrinet.h>
#include <pnp/pnp_nodes.h>
#include <pnp/pnp_action.h>
#include <pnp/pnp_executable.h>
#include <pnp/utils.h>
#include <pnp/conditionchecker.h>
#include <pnp/externalconditionchecker.h>
#include <pnp/pnp_instantiators.h>

namespace PetriNetPlans {

class PlanObserver;
	
/**
*\brief A generic plan
*
* This class is usually used through the typedef ::PnpPlan
*
*\author Vittorio Amos Ziparo
*\author Daniele Calisi
*/
template<typename PnpPlaceClass, typename PnpTransitionClass>
class PnpPlanTemplate :
	public PetriNet<PnpPlaceClass, PnpTransitionClass>,
	public PnpExecutable,
  public ConditionChecker
{
public:
	typedef PnpPlanTemplate<PnpPlaceClass, PnpTransitionClass> PnpPlanClass;

	/**
	*\brief ctor
	*
	*\param ei is the executable instantiator, it is used to create actions and subplans
	*\param ec is the external condition checker.
	*\param name is the name of the plan
	*/
	PnpPlanTemplate(ExecutableInstantiator* ei, ExternalConditionChecker* ec, const std::string name = "unnamed");
	
	/**
	*\brief dtor
	*/
	virtual ~PnpPlanTemplate();

	
	/**
	*\brief Returns \c true iff this plan has reached a goal marking
	*
	*\return \c true iff this plan has reached a goal marking
	*/
	virtual bool isInGoalState() const;
	
	/**
	*\brief Returns \c true iff this plan has reached a fail marking
	*
	*\return \c true iff this plan has reached a fail marking
	*/
	virtual bool isInFailState() const;

	// overloads (and calls) PetriNet function, considers also guards
	/**
	*\brief Tests all the conditions and determines which transitions are enabled
	*
	* A transition is enabled if its input places have enough tokens with respect
	* to the edges' weights, and the condition associated to it holds. If no
	* condition is associated to the transition than it is supposed to be \c true,
	* that is the condition that always holds.
	*
	* Please notice that not necessarily all the enabled transitions will be able
	* to fire, since some of them could be competing for the same tokens creating
	* a non-determinism.
	*
	*\return the set of all enabled transitions
	*/
	virtual std::set<PnpTransitionClass*> getAllEnabledTransitions();


	/**
	*
	\brief Registers a plan observer to be notified when the marking changes
	*
	*\author Matteo Leonetti
	*
	*\param observer the observer to add. The plans owns this object unless deleted with removeCurrentObserver()
	*/
	void setObserver( PlanObserver *observer);

	/**
	*\brief Removes the current observers if any and returns it.
	*
	*\author Matteo Leonetti
	*
	*\return the current observer if any was registered, \c NULL otherwise.
	*/
	PlanObserver *removeObserver();

	/**
	*\brief Returns the current marking
	*
	*\author Matteo Leonetti
	*
	*\return the current marking
	*/
	std::map<std::string,int> currentMarking() const;

	/**
	*\name PnpExecutable interface
	*/
	//@{
	virtual void start();
	virtual void resume();
	virtual void end();
	virtual void interrupt();
	virtual void fail();
	virtual void executeStep();
	
	/**
	*\brief Returns \c true iff this plan has reached a goal marking
	*
	* This function returns isInGoalState()
	*
	*\sa PnpExecutable::finished()
	*\sa isInGoalState()
	*/
	virtual bool finished() { return isInGoalState(); }
	
	/**
	*\brief Returns \c true iff this plan has reached a fail marking
	*
	* This function returns isInFailState()
	*
	*\sa PnpExecutable::failed()
	*\sa isInFailState()
	*/
	virtual bool failed() { return isInFailState(); }
	//@}

	/**
	*\brief Either returns or creates the requested executable
	*
	* If the executable with name \p name is in the list of the
	* executables activated by this plan a pointer to it is returned.
    * Otherwise, the method tries to create it through its ExecutableInstantiator.
    * If this fails to \c null is returned.
    *
	*\param name is the name of the executable to return or create.
    *\return an executable corresponding to the name \p name or \c null if it
	* can't be found nor created.
	*/
	virtual PnpExecutable* getExecutable(const std::string& name);

	/**
	*\brief Returns the name of this plan
	*
	*\return the name of this plan
	*/
	inline std::string getPlanName() const { return planName; }
        
	/**
	*\brief Sets a new name for this plan
	*
	*\param newname is the new name of this plan
	*/
	inline void setPlanName(const std::string& newname) { planName = newname;}
	
	/**
	*\brief Returns the active places
	*
	*\author Fabio Previtali
	*
	*\return the active places
	*/
	inline static std::map<std::string,std::string> getActivePlaces() { return activePlaces; }

protected:
	std::string planName;
	std::map<std::string, PnpExecutable*> inactiveExecutables;
	std::map<std::string, PnpExecutable*> activeExecutables;

  ExecutableInstantiator *executableInstantiator;
	ExternalConditionChecker *conditionChecker;

 	virtual bool evaluateAtomicCondition(const std::string& atom);
	virtual void fireAllEnabledTransitions();
	virtual void executeAllActiveActions();

	private:
		PlanObserver *observer;
		
		// Fabio Previtali: analizing the code and trying to understand how get all the active places, creating a static map was the
		//					only possible solution.
		static std::map<std::string,std::string> activePlaces;
};

} // namespace

#include "pnp_plan.hpp"

#endif //_PNP_EXEC

