#ifndef learnpnp_Env1DQInst_h__guard
#define learnpnp_Env1DQInst_h__guard

#include "../../../TestInstantiator.h"

#include <pnp/learning_plan/algo/DelayedQLearning.h>

namespace learnpnp {

class Env1DQInst : public TestInstantiator {

public:
	Env1DQInst ( RewEnv* checker, DelayedQLearning::ParamType par ) :
			TestInstantiator ( checker ),par ( par ) {}

protected:
	virtual Controller* createController ( const std::string& );
private:
	DelayedQLearning::ParamType par;
};

}

#endif
