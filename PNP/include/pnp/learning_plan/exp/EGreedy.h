#ifndef learnpnp_EGreedy_h__guard
#define learnpnp_EGreedy_h__guard

#include <pnp/learning_plan/ExpPolicy.h>

namespace learnpnp {

/**
* \brief Implements an epsilon-greedy exploration strategy.
*
* This strategy returns the next marking with highest value with probability (1- \p epislon),
* and a random marking (among the ones available) with probability \p epsilon
*
*\author Matteo Leonetti
*/
class EGreedy : public ExpPolicy {

private:

double epsilon;

public:

	/**
	*\brief Creates an epsilon-greedy policy with the specified epsilon
	*
	*\param epsilon is the probability according to which a uniformly distributed random action
	*will be selected
	*\param remember whether or not this exploration policy should remember previous
	*choices
	*\see ExpPolicy for more information about memory.
	*/
	explicit EGreedy(double epsilon, bool remember = true);


private:

	int makeChoice(Learner *learner,const Marking &current, const std::vector<Marking> &states);

};

}

#endif
