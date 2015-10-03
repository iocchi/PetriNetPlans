#ifndef pnpros_ROSConds_h__guard
#define pnpros_ROSConds_h__guard

#include <pnp/externalconditionchecker.h>
#include <pnp_msgs/PNPCondition.h>
#include <ros/ros.h>

namespace pnpros
{
	class ROSConds : public PetriNetPlans::ExternalConditionChecker
	{
		private:
			ros::ServiceClient client;

            // map<string,int> ConditionCache; - defined in ExternalConditionChecker
			
		public:
			ROSConds();
			
			bool evaluateAtomicExternalCondition(const std::string& atom);

	};
}

#endif
