#ifndef learnpnp_RewardCollector_h__guard
#define learnpnp_RewardCollector_h__guard

#include <pnp/externalconditionchecker.h>

namespace learnpnp {

/**
*\brief Interface to collect the reward during execution and test external conditions
*
*\author Matteo Leonetti
*/
class RewardCollector : public PetriNetPlans::ExternalConditionChecker {

public:

/**
*\brief returns the reward accumulated since the last time this method was invoked
*
*\return the reward collected since the last time this method was invoked.
*/
virtual double reward() = 0;

/**
*\brief Virtual dtor.
*/
virtual ~RewardCollector() {}

};

}

#endif
