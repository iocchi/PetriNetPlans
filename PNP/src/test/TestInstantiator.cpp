#include "TestInstantiator.h"

#include <pnp/learning_plan/learnPlan.h>

#include "learning_plan/envs/ActionRew.h"
#include "learning_plan/envs/RewEnv.h"

#include "configTest.h"

#include <string>

using namespace PetriNetPlans;
using namespace learnpnp;
using namespace std;

namespace learnpnp {

class FakeAction : public PetriNetPlans::PnpAction {};

TestInstantiator::TestInstantiator ( RewEnv *checker ) :
		planLoader(), checker ( checker ) {


}

PnpExecutable* TestInstantiator::createExecutable ( const std::string& name )
throw ( std::runtime_error ) {

	if ( name == "fake" )
		return new LearnPlan ( this,checker,createController ( "fake_txt" ) );

	if ( name.substr ( 0,2 ) == "RA" ) {
		stringstream ss ( name.substr ( 3 ) );
		double rew;
		ss >> rew;
		return new ActionRew ( checker,rew );
	}

	try {
		string path = PLAN_DIR;
		path+= name;

		Controller * controller = createController ( path + "_.txt" );

		PnpPlan *plan = new LearnPlan ( this,checker,controller );

		string pnml = path + ".pnml";
		planLoader.loadFromPNML ( pnml, plan );


		return plan;
	} catch ( std::runtime_error& ) {



		throw std::runtime_error ( "unknown action " + name );
	}
}

TestInstantiator::~TestInstantiator() {}

}
