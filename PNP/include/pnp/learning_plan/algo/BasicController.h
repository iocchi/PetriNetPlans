#ifndef learnpnp_BasicController_h__guard
#define learnpnp_BasicController_h__guard

#include <pnp/learning_plan/algo/LoggingController.h>

namespace learnpnp {
	
class ExpPolicy;
class Learner;

/**
*\brief A simple controller that delegates learning to a Learner and exploration to a ExpPolicy
*
*Simple controllers can be built making use of this class to compose the available
*update rules and exploration strategies. A simple and common combination
*could be BasicController(learnpnp::TDLambda(), learnpnp::EGreedy()) (passing
*to both the appropriate parameters)
*
*\author Matteo Leonetti
*/
class BasicController: public LoggingController {

public:
	/**
	 * \brief Creates a BasicController from the given update rule and exploration strategies
	 *
	 * \param learner manages the value function and incapsulates the update rule
	 * \param policy makes decisions at choice points
	 */
	BasicController( Learner * learner, ExpPolicy *policy);

	/**
	 * \brief virtual dtor
	 */
	virtual ~BasicController();

	
	virtual int choose(const Marking& current, const std::vector<Marking> &states);

	virtual void updateV(const Marking &current, double reward,const Marking &next) ;

	virtual void tick();

protected:
	//owns
	/**
	 * The policy used to make decisions at choice points.
	 *
	 * The class owns this object (will be deleted by the destructor)
	 */
	ExpPolicy *policy;
};

}

#endif
