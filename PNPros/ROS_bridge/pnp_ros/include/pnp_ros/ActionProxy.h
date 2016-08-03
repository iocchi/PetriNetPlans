#include <pnp/pnp_action.h>
#include <ros/ros.h>
#include <set>
#include <string>

#define USE_MESSAGES 0
#define USE_ACTIONLIB 1

//#if USE_MESSAGES
#include <pnp_msgs/Action.h>
#include <pnp_msgs/ActionFinished.h>
//#endif

//#if USE_ACTIONLIB
//#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
//#include <client_helpers.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPResult.h>
#include <pnp_msgs/PNPFeedback.h>
//#endif

namespace pnpros
{
	class ActionProxy : public PetriNetPlans::PnpAction
	{
		private:
			static std::set<std::string> activeActions;
			static unsigned long long maxID;
			
            actionlib::ClientGoalHandle<pnp_msgs::PNPAction> goalhandler;
			std::string robotname, name, params, id;
            unsigned long long iid;
            bool active;
			
            void feedbackCb(actionlib::ClientGoalHandle<pnp_msgs::PNPAction> gh,const pnp_msgs::PNPFeedbackConstPtr& feedback);
            void transitionCb(actionlib::ClientGoalHandle<pnp_msgs::PNPAction> gh);
			
		public:
			static ros::Publisher publisher;
			
			ActionProxy(const std::string& name);
			~ActionProxy();
			
            static void actionTerminationCallback(const pnp_msgs::ActionFinished::ConstPtr& message);
			
			virtual void start();
			virtual void interrupt();
			virtual void end();
			virtual bool finished();
	};
}
