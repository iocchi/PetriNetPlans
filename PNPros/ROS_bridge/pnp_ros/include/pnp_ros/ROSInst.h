#ifndef pnpros_ROSInst_h__guard
#define pnpros_ROSInst_h__guard

#include <pnp/pnp_instantiators.h>
#include <pnp/pnpfwd.h>

namespace pnpros
{
	class ROSInst : public PetriNetPlans::ExecutableInstantiator
	{
		private:
			// Does not own.
			PetriNetPlans::ExternalConditionChecker* checker;
			
		protected:
			const std::string& planFolder;
			
		public:
			explicit ROSInst(PetriNetPlans::ExternalConditionChecker*,const std::string& planFolder);
			
			virtual ~ROSInst();
			
			virtual PetriNetPlans::PnpExecutable* createExecutable(const std::string& name) throw(std::runtime_error);
	};
}

#endif
