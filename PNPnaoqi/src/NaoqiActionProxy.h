#include <pnp/pnp_action.h>
#include <set>
#include <string>

#include <alvalue/alvalue.h>

double getCurrentTime();

class ActionProxy;

static std::set<ActionProxy *> activeActions;
void ActionProxy_endAllActions();

class ActionProxy : public PetriNetPlans::PnpAction
{
	private:
		
		static unsigned long long maxID;
		
		std::string robotname, name, params, id;
        unsigned long long iid;
        bool active;

		qi::AnyObject memProxy;
		qi::AnyObject acb_signal;
		qi::SignalLink signalID;

	public:
		
		ActionProxy(const std::string& name);
		~ActionProxy();
				
		virtual void start();
		virtual void interrupt();
		virtual void end();
		virtual bool finished();

		virtual void actionTerminationCallback();
  
};

