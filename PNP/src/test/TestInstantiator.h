#ifndef learnpnp_TestInstantiator_h__guard
#define learnpnp_TestInstantiator_h__guard

#include <pnp/pnpfwd.h>
#include <pnp/pnp_instantiators.h>
#include <pnp/basic_plan/xml_plan_instantiator.h>


#include <string>

namespace learnpnp {

class Controller;
class RewEnv;

class TestInstantiator : public PetriNetPlans::ExecutableInstantiator {

public:

	TestInstantiator(RewEnv *checker);

	virtual PetriNetPlans::PnpExecutable* createExecutable(const std::string& name) throw(std::runtime_error);

	virtual ~TestInstantiator();

protected:
	virtual Controller *createController(const std::string&) = 0;
	
private:

	PetriNetPlans::XMLPnpPlanInstantiator planLoader;
	RewEnv *checker;
};

}

#endif
