#include "RewEnv.h"


#include <iostream>
using namespace std;
namespace learnpnp {

bool RewEnv::evaluateAtomicExternalCondition(const std::string& atom) {
	return true;
}

double RewEnv::reward() {
	double val = rew;
	rew = 0;
	return val;
}

void RewEnv::sumRew(double val ) {
	rew += val;
}


}
