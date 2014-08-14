
#include <pnp/learning_plan/ExpPolicy.h>

#include <algorithm>

using namespace std;

namespace learnpnp {

	ExpPolicy::ExpPolicy(bool remember) : remember(remember) {}

	int ExpPolicy::choose(Learner *learner,const Marking& current, const std::vector<Marking> &states) {

	 if(remember) {
		MemoryType::const_iterator  memorized = memory.find(current);
		if(memorized != memory.end()) {
			//is the memorized marking among the admissible ones?
			vector<Marking>::const_iterator found = find(states.begin(),states.end(),memorized->second);
			if(found != states.end())
				return distance(states.begin(),found);
		}
	 }

	int ret = makeChoice(learner,current,states);

	if(remember)
		memory.insert(std::make_pair(current,states[ret]));
	
	return ret;
}

}
