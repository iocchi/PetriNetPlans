/*
 * pnp_executer.h
 * authors: Vittorio Amos Ziparo and Daniele Calisi
 */

#ifndef PNP_EXECUTER_H
#define PNP_EXECUTER_H

#include "pnp_plan.h"
#include "externalconditionchecker.h"
#include "pnp_instantiators.h"


namespace PetriNetPlans {
/**
* \brief The Main class for executing Petri Net Plans
*
* In order to execute a Petri Net Plan you need to instantiate this class
* providing an ExecutableInstantiator that will be used to create the main
* plan. Than you set the main plan name through setMainPlan() and repeatedly call
* execMainPlanStep() until the either goal marking or the failing marking has 
* been reached (or just for a fixed number of times).
* \author Vittorio Amos Ziparo
* \author Daniele Calisi
*/
template<typename PnpPlanClass>
class PnpExecuter {
public:
	/**
	 * \brief ctor
	 *
	 * \attention The PnpExecuter becomes the \a  owner of the instantiator
	 * \param instantiator is the object able to create plans and actions.
	 * */
	PnpExecuter(ExecutableInstantiator* instantiator);
	
	/**
	* \brief Sets the name of the main plan
	*
	* The executor will make use of the ExecutableInstantiator provided
	* in the constructor to create the main plan every time this method is
	* invoked. The main plan is the root of every plan in execution.
	* \param planName is the name of the plan that will become the new main plan.
	*/
	void setMainPlan(const std::string& planName);
	
	/**
	* \brief Returns the name of the actual main plan
	* 
	* \return the name of the actual main plan.
	*/
	std::string getMainPlanName() const;
	
	/**
	* \brief Executes one step of the main plan and all its sub plans
	*
	* A step consists in all enabled transitioned being fired top down from the main
	* plan to the leaves, and all the executeStep() for the actions in active places
	* (after the transition) begin called once.
	* 
	* \return \c false if the main plan was not set or it is either in a goal or a fail
	* state. It returns \c true otherwise.
	*/
	bool execMainPlanStep();
	
	/** 
	* \brief Tests whether the main plan is in a goal state
	* 
	* \return \c true iff the main plan is in a goal state
	*/
	bool goalReached();
	
	/** 
	* \brief Tests whether the main plan is in a fail state
	* 
	* \return \c true iff the main plan is in a fail state
	*/
	bool failReached();

	/**
	*\brief Returns the current active places.
	*
	*\author Fabio Previtali
	*
	*\return Returns the current active places.
	*/
	inline std::vector<std::string> getNonEmptyPlaces()
	{
		std::vector<std::string> nonEmptyPlaces;
		
		const std::map<std::string,std::string>& activePlaces = PnpPlanClass::getActivePlaces();
		
		for (std::map<std::string,std::string>::const_iterator it = activePlaces.begin(); it != activePlaces.end(); ++it)
		{
			nonEmptyPlaces.push_back(it->second);
		}
		
		return nonEmptyPlaces;
	}
	
	/**
	 * \brief dtor
	 *
	 * \attention deletes the main plan \em and the instantiator.
	 * */
	~PnpExecuter();

        /**
    *\brief sets observer of Planner
    *
    * */
    inline void setObserver(PlanObserver *observer)
    {
        if (mainPlan != NULL)
            mainPlan->setObserver(observer);
    }


protected:
	PnpPlanClass* mainPlan;
	ExecutableInstantiator *istantiator;

};

#include "pnp_executer.hpp"

} // namespace

#endif
