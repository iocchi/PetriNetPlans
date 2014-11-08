#ifndef learnpnp_Controller_h__guard
#define learnpnp_Controller_h__guard

#include <pnp/learning_plan/Marking.h>

namespace learnpnp {


/**
*\brief Controllers make decisions in non-deterministic choice points and take care of learning
*
*Controllers can be either learning or not; during the periods in which they are not learning,
*no change is made to the information used (e.g. the value function), but they still make decisions
*based on it to allow the evaluation of their current policy.
*
*learnpnp::LearnPlan calls updateV() when the marking changes, and tick() when a time step
*has elapsed and no transition has fired. It also invokes choose() when the choice between to
*possible next markings is non-deterministic.
*
*\author Matteo Leonetti
*/
struct Controller {

  	/**
	*\brief Updates the value function when a transition between two states (markings) has occurred
	*
	*\param current is the state before the transition occurred
	*\param reward is the reward accumulated since the last transition happened
	*\param next is the state after the transition occurred
	*/
	virtual void updateV(const Marking &current, double reward,const Marking &next) = 0;


	/**
	*\brief Notifies the controller that one timestep has passed although the marking has not changed
	*/
	virtual void tick() = 0;

	/**
	*\brief Returns the position in the vector \c states of the chosen next marking
	*
	*\param current is the current marking
	*\param states is the vector of all possible next markings
	*/
	virtual int choose(const Marking& current, const std::vector<Marking> &states) =0;

	virtual ~Controller() {}

    /**
	*\brief Sets whether this controller is allowed to modify its information (e.g. the value function)
	*/
	virtual void setLearning( bool newLearning) {
		learning = newLearning;
	}

    /**
	*\brief  Returns whether this controller is learning at the moment 
	*/
	bool isLearning() const {
		return learning;
	}

protected:
	bool learning;
};

}

#endif
