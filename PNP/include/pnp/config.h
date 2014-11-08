/**
* \file config.h
* \brief Compilation parameters for the executor
*
* At the moment this file is solely responsible to set whether 
* the executor must or must not delete actions and plans when it does not
* need them anymore. The default behavior is to delete them.
*
* \sa PetriNetPlans::ExecutableInstantiator::createExecutable()
*/

#define DELETE_CONCLUDED_EXECUTABLES 1	// delete plans or actions that  end()-ed or fail()-ed

