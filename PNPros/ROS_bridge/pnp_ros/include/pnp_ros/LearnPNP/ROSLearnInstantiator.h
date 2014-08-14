#ifndef ROS_LEARN_INSTANTIATOR_H
#define ROS_LEARN_INSTANTIATOR_H

#include <pnp_ros/ROSInst.h>
#include <pnp/basic_plan/xml_plan_instantiator.h>
#include <pnp/learning_plan/Controller.h>
#include <pnp/learning_plan/RewardCollector.h>
#include <pnp/learning_plan/algo/LoggingController.h>

namespace pnpros
{
	namespace LearnPNP
	{
		class ROSLearnInstantiator : public pnpros::ROSInst
		{
			private:
				PetriNetPlans::XMLPnpPlanInstantiator planLoader;
				learnpnp::RewardCollector* reward;
				PetriNetPlans::PnpPlan* network;
				bool logPlaces;
				
				learnpnp::Controller* createController(const std::string& VfunFilePath);
				void setLogger(learnpnp::LoggingController*);
				
			public:
				ROSLearnInstantiator(PetriNetPlans::ExternalConditionChecker*,const std::string&,bool);
				
				virtual ~ROSLearnInstantiator();
				
				virtual PetriNetPlans::PnpExecutable* createExecutable(const std::string& name) throw(std::runtime_error);
		};
	}
}

#endif
