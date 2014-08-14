#include "Env1DQInst.h"

#include <set>

using namespace std;

namespace learnpnp {
	
	Controller* Env1DQInst::createController(const std::string& vFile) {
		
// 		set<string> places;
// 		places.insert("p1");
// 		places.insert("p2");
// 		places.insert("p3");
// 		places.insert("p4");
// 		places.insert("p5");
// 		PlaceLogger p1(vFile+"_log",places);
		
		LoggingController *cont = new DelayedQLearning(vFile,par);
// 		cont->addLogger(p1);
		
		return cont;
		
	}
	
	
}
