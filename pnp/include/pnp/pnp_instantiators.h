#ifndef ExecutableInstantiator_H
#define ExecutableInstantiator_H

#include <string>
#include <stdexcept>
#include "basic_plan/basic_plan.h"
#include "pnpfwd.h"

namespace PetriNetPlans {

  /**
  * \brief The interface for classes accountable for the creation of plans and actions
  * 
  */
class ExecutableInstantiator {
  public:

	/** 
	* \brief The method to implement to return plans and actions.
	*
	* The executor does not distinguish between actions and plans so both
	* are returned as a reference to a PnpExecutable. Executables are or are not
	* destroyed by the executor according to a parameter in config.h you can set
	* at compile time. The default for this value is to destroy the objects
	* so the pointers returned by this method in that case should be to newly
	* created objects.
	* 
	* \param name is the name of either the plan or the action that must be
	* instantiated
	* \return a pointer to the executable object correspoding to \c name
	* \exception runtime_error if it cannot istantiate the executable (e.g. the name
	* is wrong)
	*/
    virtual PnpExecutable* createExecutable(const std::string& name) throw(std::runtime_error) = 0;

	/** 
	* \brief dtor
	*/
    virtual ~ExecutableInstantiator() {}
};


} // namespace

#endif
