#ifndef PetriNetPlans_ExternalConditionChecker_guard
#define PetriNetPlans_ExternalConditionChecker_guard

#include <string>
#include <map>

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

        protected:
            // Stores values of conditions
            std::map<std::string,bool> ConditionCache;

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

            void clearCache() { ConditionCache.clear(); }

			/**
			*\brief dtor
			*/
			virtual ~ExternalConditionChecker() {}
	};

}

#endif
