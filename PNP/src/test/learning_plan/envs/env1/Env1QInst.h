#ifndef learnpnp_Env1QInst_h__guard
#define learnpnp_Env1QInst_h__guard

#include "../../../TestInstantiator.h"

#include <pnp/learning_plan/algo/QLearning.h>

namespace learnpnp {

class Env1QInst : public TestInstantiator {

public:
	Env1QInst ( RewEnv* checker, QLearning::ParamType par ) :
			TestInstantiator ( checker ),par ( par ) {}

protected:
	virtual Controller* createController ( const std::string& );
private:
	QLearning::ParamType par;
};

}

#endif
