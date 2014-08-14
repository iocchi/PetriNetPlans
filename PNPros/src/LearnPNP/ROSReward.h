#ifndef ROS_REWARD_H
#define ROS_REWARD_H

#include <pnp/learning_plan/RewardCollector.h>
#include <ros/ros.h>

namespace pnpros
{
	namespace LearnPNP
	{
		class ROSReward : public learnpnp::RewardCollector
		{
			private:
				ros::ServiceClient clientCondition, clientReward;
				double rewardValue;
				
			public:
				ROSReward();
				
				bool evaluateAtomicExternalCondition(const std::string& atom);
				virtual double reward();
		};
	}
}

#endif
