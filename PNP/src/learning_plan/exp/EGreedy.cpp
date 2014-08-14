#include <pnp/learning_plan/exp/EGreedy.h>

#include <pnp/learning_plan/exp/Exploit.h>

namespace learnpnp {

	EGreedy::EGreedy ( double epsilon, bool remember ) : ExpPolicy(remember), epsilon ( epsilon ) {}

	int EGreedy::makeChoice(Learner *learner,const Marking &current, const std::vector<Marking> &states)  {
		
		int v = rand();

		if ( v < epsilon * RAND_MAX ) {

			//pick one randomly
			int chosen = ( rand() / ( RAND_MAX / states.size() ) );

			return chosen;
		}

		return Exploit().choose(learner,current,states);

    }

}

