#include "Env1QInst.h"

#include <pnp/learning_plan/exp/EGreedy.h>

#include <set>

using namespace std;

namespace learnpnp {
	
	Controller* Env1QInst::createController(const std::string& vFile) {
		ExpPolicy *policy = new EGreedy(0.5,false);
		
// 		set<string> places;
// 		places.insert("p1");
// 		places.insert("p2");
// 		places.insert("p3");
// 		places.insert("p4");
// 		places.insert("p5");
// 		PlaceLogger p1(vFile+"_log",places);
		
		LoggingController *cont = new QLearning(vFile,par,policy);
//  		cont->addLogger(p1);
		
		return cont;
		
	}
	
	
}
