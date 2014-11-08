#include <pnp/learning_plan/algo/BasicController.h>

#include <pnp/learning_plan/ExpPolicy.h>
#include <pnp/learning_plan/Learner.h>

namespace learnpnp {

BasicController::BasicController(Learner * learner, ExpPolicy *policy) :
								LoggingController(learner),
								policy(policy){}

BasicController::~BasicController() {
	delete policy;
}

int BasicController::choose(const Marking& current, const std::vector<Marking> &states) {
	return policy->choose(learner,current, states);
}

void BasicController::updateV(const Marking &current, double reward,const Marking &next) {
	if(learning)
		learner->update(current,reward,next);

	this->visited(current);//for logging
}

void BasicController::tick(){
	if(learning)
		learner->tick();
}


}

