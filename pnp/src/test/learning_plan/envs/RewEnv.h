#ifndef learnpnp_RewEnv1_h__guard
#define learnpnp_RewEnv1_h__guard

#include <pnp/learning_plan/RewardCollector.h>

namespace learnpnp {

class RewEnv : public RewardCollector {

private:
	double rew;
	
public:

	RewEnv() : rew(0) {}
    virtual bool evaluateAtomicExternalCondition(const std::string& atom);
    virtual double reward();

	void sumRew(double );
};


}

#endif