#include <pnp/learning_plan/algo/MCESP_SAA.h>

#include <pnp/learning_plan/exp/Exploit.h>
#include <pnp/learning_plan/exp/ExploitWithHysteresis.h>
#include <pnp/learning_plan/algo/AlphaScheduler.h>

#include <algorithm>
#include <iterator>
#include <limits>

using namespace std;

namespace learnpnp {


class SelectiveDynamicAlpha : public DynamicAlpha {
public:

	SelectiveDynamicAlpha(double min_alpha, const std::string& file) :
		DynamicAlpha(min_alpha,file) {}
	
	double alpha(const Marking& state) {
// 		std::cout << state.getId() << " selected " << selected_state.getId() << endl;

		if(state == selected_state) {
// 			cout << DynamicAlpha::alpha(state) << endl;
			return DynamicAlpha::alpha(state);
		}
// 		cout << 0 << endl;
		return 0;
	}

	void setSelectedState(const Marking& state) {
		selected_state = state;
// 		cout << "setting " << state.getId() << " to " << selected_state.getId() << endl;
	}

private:
	Marking selected_state;
	
};


std::map< std::string, std::queue< std::pair<Marking, Marking> > > MCESP_SAA::queue_map;
std::map< std::string, std::set<Marking> > MCESP_SAA::choice_map;



MCESP_SAA::MCESP_SAA(const string& file,const MCESP_SAA::ParamType& params, bool learning) :
		LoggingController(NULL), current_episode(params.current_episode),
		stage_length(params.stage_length) {

	action_queue = &queue_map[file];
	choice_points = &choice_map[file];

	exp_strategy = new ExploitWithHysteresis(file, std::numeric_limits<double>::max());

	sched = new SelectiveDynamicAlpha(params.alpha, file+"_count");

	learner = new MonteCarlo(file,params,sched);

//  	std::cout << "episode " << current_episode << std::endl;

	if (params.current_episode % params.stage_length == 1) {
		sched->reset();
		
		if(!action_queue->empty()) {
		ExpPolicy::MemoryType *policy = exp_strategy->getPolicy();
		
		Marking &previous = (*policy)[action_queue->front().first];
		Marking &explorative = action_queue->front().second;
		
		double exploration_value = learner->valueOf(explorative);
		double previous_value = learner->valueOf(previous);
		
		if(exploration_value > previous_value) {
		  //update policy
		  (*policy)[action_queue->front().first] = explorative;
		
		  cout << "last jump at " << current_episode << endl;
		}
		  
		  //rotate the elements
		rotate_queue();
		}
	}
}

MCESP_SAA::~MCESP_SAA() {
	delete exp_strategy;
}


int MCESP_SAA::choose(const Marking& current, const std::vector<Marking> &states) {

	if (!isLearning())
		return Exploit(false).choose(learner,current,states);

	//add state-action pairs to the queue if not present
	grow_queue(current,states);

	if (action_queue->front().first == current) {
		vector<Marking>::const_iterator found = find(states.begin(),states.end(),action_queue->front().second);

		if (found != states.end()) {
			//last_policy[current]=*found;
//   		std::cout << "choosing " << states[distance(states.begin(),found)].getId() << std::endl;
			return distance(states.begin(),found);
		}
	}

//  std::cout << "using best " << states[exp_strategy->choose(learner,current,states)].getId() << std::endl;
	return exp_strategy->choose(learner,current,states);
}

void MCESP_SAA::updateV(const Marking &current, double reward,const Marking &next) {
	this->visited(current);

	if (learning)
		learner->updateV(current,reward,next);
}

void MCESP_SAA::tick() {
	learner->tick();
}

void MCESP_SAA::rotate_queue() {
	if (!action_queue->empty() && isLearning()) {
		action_queue->push(action_queue->front());
		action_queue->pop();
		sched->setSelectedState(action_queue->front().second);
// 		std::cout << current_episode << " exploring " << action_queue->front().second.getId() << std::endl;
	}
}

void MCESP_SAA::grow_queue(const Marking& current, const std::vector<Marking> &states) {
	if (choice_points->insert(current).second) {
		vector<Marking>::const_iterator stIt = states.begin();
		
		for (; stIt != states.end(); ++stIt)
			action_queue->push(make_pair(current,*stIt));
		
		cout << "queue size: " << action_queue->size() << endl;
	}
	//might have changed the front if it was empty
	sched->setSelectedState(action_queue->front().second);
}

}
