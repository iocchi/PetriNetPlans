#include <pnp/learning_plan/algo/QLearning.h>
#include <pnp/learning_plan/algo/TDLambda.h>

#include <boost/bind.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace learnpnp {

static TDLParams parametersForQLearning(const TD0Params& params) {
	TDLParams ret;
	ret.alpha = params.alpha;
	ret.gamma = params.gamma;
	ret.lambda = 0;
	ret.initialValue = params.initialValue;
	return ret;
}

QLearning::QLearning(const std::string &filePath, const ParamType& parameters,  ExpPolicy *policy) :
		BasicController(new TDLambda(filePath,parametersForQLearning(parameters)),policy),
		maxNextMarking() {}

QLearning::QLearning(const std::string &filePath, const ParamType& parameters,
                     ExpPolicy *policy, AlphaScheduler *scheduler) :
		BasicController(
		    new TDLambda(filePath,parametersForQLearning(parameters), scheduler),
		    policy), maxNextMarking() {}

int QLearning::choose(const Marking& current, const std::vector<Marking> &states) {
	std::vector<double> values;
	std::transform(states.begin(),
	               states.end(),
	               std::back_inserter(values),
	               boost::bind(&Learner::valueOf,learner,_1));

	std::vector<double>::iterator elem = std::max_element(values.begin(), values.end());
	maxNextMarking = states[ std::distance(values.begin(), elem)];

	return BasicController::choose(current,states);
}

void QLearning::updateV(const Marking &current, double reward,const Marking &next) {
	if (maxNextMarking != Marking()) {
		BasicController::updateV(current,reward,maxNextMarking);
		maxNextMarking = Marking();
	} else
		BasicController::updateV(current,reward,next);

}

}
