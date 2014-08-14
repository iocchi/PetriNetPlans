#include <pnp/pnp_action.h>
#include <ros/ros.h>
#include <set>
#include <string>

#define USE_MESSAGES 0
#define USE_ACTIONLIB 1

//#if USE_MESSAGES
#include <PNPros/Action.h>
#include <PNPros/ActionFinished.h>
//#endif

//#if USE_ACTIONLIB
//#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
//#include <client_helpers.h>
#include <PNPros/PNPAction.h>
#include <PNPros/PNPResult.h>
#include <PNPros/PNPFeedback.h>
//#endif

namespace pnpros
{
	class ActionProxy : public PetriNetPlans::PnpAction
	{
		private:
			static std::set<std::string> activeActions;
			static unsigned long long maxID;
			
			actionlib::ClientGoalHandle<PNPros::PNPAction> goalhandler;
			std::string robotname, name, params, id;
			
			void feedbackCb(actionlib::ClientGoalHandle<PNPros::PNPAction> gh,const PNPros::PNPFeedbackConstPtr& feedback);
			void transitionCb(actionlib::ClientGoalHandle<PNPros::PNPAction> gh);
			
		public:
			static ros::Publisher publisher;
			
			ActionProxy(const std::string& name);
			
			static void actionTerminationCallback(const PNPros::ActionFinished::ConstPtr& message);
			
			virtual void start();
			virtual void interrupt();
			virtual void end();
			virtual bool finished();
	};
}
