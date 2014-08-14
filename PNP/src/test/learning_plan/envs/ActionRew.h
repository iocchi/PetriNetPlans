#ifndef learnpnp_ActionRew_h__guard
#define learnpnp_ActionRew_h__guard

#include <pnp/pnp_action.h>

namespace learnpnp {

class RewEnv;
	
class ActionRew : public PetriNetPlans::PnpAction {
private:

RewEnv *env;
double reward;
	
public:

	ActionRew(RewEnv *env, double reward);
	
    virtual void end();
    virtual bool finished();
	

};

}

#endif
