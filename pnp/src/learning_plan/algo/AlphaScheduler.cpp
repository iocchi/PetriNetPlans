#include <pnp/learning_plan/algo/AlphaScheduler.h>

#include <iostream>

namespace learnpnp {
DynamicAlpha::DynamicAlpha ( double min_alpha, const string& file ) :
	min_alpha(min_alpha), file(file){
		readVFunctionFromTxt(file, state_count);
	}

double DynamicAlpha::alpha(const Marking& state) {
	if(state_count[state] == 0)
		std::cerr << "Dynamic Alpha: a marking has count 0" << std::endl;

	return std::max(1/state_count[state],min_alpha);
}

void DynamicAlpha::visited(const Marking& state) {
	state_count[state]++;
}


DynamicAlpha::~DynamicAlpha() {
	if(file != "")
		writeVFunctionToTxt(file, state_count);
}

void DynamicAlpha::reset() {
	state_count.clear();
}

}
