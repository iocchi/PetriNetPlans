#ifndef pnpfwd_guard
#define pnpfwd_guard

/**
*\file pnpfwd.h
*\brief forward declarations. Include this file in headers instead of the respective header files.
*/

namespace PetriNetPlans {

	class PnpExecutable;
	class ExecutableInstantiator;
	class ExternalConditionChecker;
	
	template<typename PnpPlanClass>
	class PnpExecuter;
}

#endif
