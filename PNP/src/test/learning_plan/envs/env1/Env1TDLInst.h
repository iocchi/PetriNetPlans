#ifndef learnpnp_Env1TDLInst_h__guard
#define learnpnp_Env1TDLInst_h__guard

#include "../../../TestInstantiator.h"

#include <pnp/learning_plan/exp/EGreedy.h>
#include <pnp/learning_plan/algo/BasicController.h>

namespace learnpnp {

template<typename Evaluator>
class Env1TDLInst : public TestInstantiator {

public:
	Env1TDLInst ( RewEnv* checker, typename Evaluator::ParamType par ) :
			TestInstantiator ( checker ),par ( par ) {}

protected:
	virtual Controller* createController ( const std::string& vFile ) {
		ExpPolicy *policy = new EGreedy ( 0.5,false );

// 		std::set<std::string> places;
// 		places.insert ( "p1" );
// 		places.insert ( "p2" );
// 		places.insert ( "p3" );
// 		places.insert ( "p4" );
// 		places.insert ( "p5" );
// 		learnpnp::PlaceLogger p1 ( vFile+"_log.txt",places );

		Evaluator *learner = new Evaluator ( vFile,par );
		BasicController *cont = new BasicController ( learner,policy );
// 		cont->addLogger ( p1 );


		return cont;
	}

private:
	typename Evaluator::ParamType par;
};

}

#endif
