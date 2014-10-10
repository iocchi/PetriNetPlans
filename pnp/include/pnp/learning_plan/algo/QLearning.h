#ifndef learnpnp_QLearning_h__guard
#define learnpnp_QLearning_h__guard

#include <pnp/learning_plan/algo/BasicController.h>
#include <pnp/learning_plan/Learner.h>
#include <pnp/learning_plan/algo/TDParams.h>
#include <pnp/learning_plan/algo/AlphaScheduler.h>

namespace learnpnp {
	
	/**
	*\brief An implementation of Q-Learning
	*
	*\author Matteo Leonetti
	*/
	class QLearning : public BasicController  {
		
		public:
			
			typedef TD0Params ParamType;

			QLearning(const std::string &filePath, const ParamType& parameters,  ExpPolicy *policy);
			QLearning(const std::string &filePath, 
						const ParamType& parameters,  
						ExpPolicy *policy,
						AlphaScheduler *scheduler);
			
			//is documented by the base class
			virtual void updateV(const Marking &current, double reward,const Marking &next);

			virtual int choose(const Marking& current, const std::vector<Marking> &states);
			
			virtual ~QLearning() {}

		private:
			Marking maxNextMarking;
	};
	
}

#endif
