#ifndef PetriNetPlans_PlanObserver
#define PetriNetPlans_PlanObserver

#include <map>
#include <string>

namespace PetriNetPlans {


	struct PlanObserver {

		virtual void markingChanged(const std::map<std::string,int>& newMarking) = 0;
		virtual ~PlanObserver(){}
	};

}


#endif
