#include <pnp_ros/ActionProxy.h>
#include <pnp_ros/LearnPNP/World/World.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_msgs/PNPReward.h>

#include<pnp_ros/LearnPNP/ROSReward.h>

using namespace std;

namespace pnpros
{
	namespace LearnPNP
	{
		ROSReward::ROSReward() : rewardValue(0.0)
		{
			ros::NodeHandle n;
			
			ROS_INFO("Setting service client for PNPConditionEval.");
			
            clientCondition = n.serviceClient<pnp_msgs::PNPCondition>("PNPConditionEval");
			
            pnp_msgs::PNPCondition conditionService;
			
			conditionService.request.cond = string("hello");
			
			if (clientCondition.call(conditionService))
			{
				ROS_INFO("Test Condition Checker: %s value: %d.", conditionService.request.cond.c_str(),
																  conditionService.response.truth_value);
			}
			else
			{
				ROS_ERROR("Failed to call service conds.");
			}
			
            clientReward = n.serviceClient<pnp_msgs::PNPReward>("PNPRewardEval");
			
            pnp_msgs::PNPReward rewardService;
			
			rewardService.request.rewardRequest = string("RewardRequest");
			
			if (clientReward.call(rewardService))
			{
				ROS_INFO("Test Reward: %s value: [%f].", rewardService.request.rewardRequest.c_str(), rewardService.response.reward);
			}
			else
			{
				ROS_ERROR("Failed to call service reward.");
			}
		}
		
		bool ROSReward::evaluateAtomicExternalCondition(const string& atom)
		{
            pnp_msgs::PNPCondition conditionService;
			
			conditionService.request.cond = atom;
			
			if (clientCondition.call(conditionService))
			{
				ROS_INFO("Cond: %s value: %d ", atom.c_str(), conditionService.response.truth_value);
			}
			else
			{
				ROS_ERROR("Failed to call service conds.");
				
				return false;
			}
			
			rewardValue = 0.0;
			
			if (conditionService.response.truth_value)
			{
                pnp_msgs::PNPReward rewardService;
				
				rewardService.request.rewardRequest = atom;
				
				// The request can fail and we cannot proceed without the correct value of the reward.
				while (!clientReward.call(rewardService))
				{
					ROS_ERROR("Failed to call service reward.");
				}
				
				rewardValue = rewardService.response.reward;
				
				World::w->sumReward(rewardValue);
			}
			
			return conditionService.response.truth_value;
		}
		
		double ROSReward::reward()
		{
			// Used to reset the reward when the plan has reached the goal.
			if (rewardValue == 1)
			{
				rewardValue = 0.0;
				
				return 1;
			}
			
			return rewardValue;
		}
	}
}
