#ifndef PetriNetPlans_ExternalConditionChecker_guard
#define PetriNetPlans_ExternalConditionChecker_guard

#include <string>

namespace PetriNetPlans {

	/**
	 * \brief A class to test atomic conditions not related to an action.
	 *
	 * Object of subclasses of this class must be able to test external atomic
	 * conditions, typically on the environment
	 *
	 * \author Matteo Leonetti
	 * */
	class ExternalConditionChecker {
		public:

			/**
			 * \brief tests an external atomic condition
			 *
			 * \param atom the condition to test
			 * \return \c true iff the condition is true. It must return false
			 * if it's not able to test the condition (for instance because no
			 * \p atom condition actually exists) and log the error.
			 * */
			virtual bool evaluateAtomicExternalCondition(const std::string& atom) = 0;

			/**
			*\brief dtor
			*/
			virtual ~ExternalConditionChecker() {}
	};

}

#endif
