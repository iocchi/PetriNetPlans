#ifndef learnpnp_SoftMax_h__guard
#define learnpnp_SoftMax_h__guard

#include <pnp/learning_plan/ExpPolicy.h>

namespace learnpnp {

/**
* \brief Implements an epsilon-greedy exploration strategy.
*
* SoftMax chooses an action a with probability Pr(a) = 
*
*\author Matteo Leonetti
*/
class SoftMax : public ExpPolicy {
  
public:
	SoftMax(double tau, bool remember = true);

private:
	double tau; //temperature

	int makeChoice(Learner *learner,const Marking &current, const std::vector<Marking> &states);

	double computeWeight(double value);

	
};

}

#endif // SOFTMAX_H
