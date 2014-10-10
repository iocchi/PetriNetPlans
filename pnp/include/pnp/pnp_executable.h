#ifndef PNP_EXECUTABLE_H
#define PNP_EXECUTABLE_H

#include <pnp/utils.h>

#include <string>

namespace PetriNetPlans {

#define BEGIN_INTERNAL_CONDITIONS virtual bool getInternalConditionValue(const std::string& name) { if (false) ;
#define INTERNAL_CONDITION(functionName) else if (name == #functionName) return functionName();
#define END_INTERNAL_CONDITIONS else { PNP_OUT("Unknown condition '" <<name<< "'"); return false; } }


/**
*\brief The interface for executables (i.e. actions and plans).
*/
class PnpExecutable {
public:
	/**
	*\brief ctor
	*/
	PnpExecutable() { }
	
	/**
	*\brief dtor
	*/
	virtual ~PnpExecutable() { }
	
	/**
	*\name Execution functions
	*/
	//@{
	
	/**
	*\brief Starts the execution
	*
	* This function is called once before any executeStep() and can
	* perform initialization procedures
	* \sa end()
	*/
	virtual void start() = 0;
	
	/**
	*\brief Resumes the execution after an interrupt()
	*
	* Resumes the execution from the state in which it was when
	* interrupt() was called
	* \attention This functionality on plans and actions has not been tested
	* thoroughly yet.
	* \sa interrupt()
	*/
	virtual void resume() = 0;
	
	/**
	*\brief Terminates the execution
	*
	* After end() has been called executeStep() cannot be invoked unless
	* another start() has been executed.
	* \sa start()
	*/
	virtual void end() = 0;
	
	/**
	*\brief Interrupts the execution keeping its state
	*
	* Temporarily pauses the execution so that it can be subsequently
	* reactivated from the state it was stopped in.
	* \attention This functionality on plans and actions has not been tested
	* thoroughly yet.
	* \sa resume()
	*/
	virtual void interrupt() = 0;
	
	/**
	*\brief Terminates the execution with a failure
	*
	* This function must be called instead of end() from the plan 
	* to notify the executable that a failure condition has been detected. 
	*/
	virtual void fail() = 0;
	
	/**
	*\brief Execute a step of a time extended executable.
	*
	* If this executable is actually a plan this function fires the 
	* enabled transitions and
	* in turn calls executeStep() on the action in the active places.
	* If it is a temporary extended action it is just supposed to execute
	* a step of that action. If the action is instantaneous this method
	* can be let empty and never called (you will indeed use an adapter class
	* (PnpAction) so that you don't even have to implement this method at all).
	*/
	virtual void executeStep() = 0;
	
	//@}
	
	/**
	*\name Condition functions
	*/
	//@{
	
	/**
	*\brief Returns \c true iff this executable has internally detected its termination
	* 
	* If this executable is \e externally terminated it must always return \c false
	*
	*\return \c true iff this executable has internally detected its termination
	*/
	virtual bool finished() = 0;
	
	/**
	*\brief Returns \c true iff this executable has internally detected a failure
	*
	*\return \c true iff this executable has internally detected a failure
	*/
	virtual bool failed() = 0;
	//@}
	
	virtual bool getInternalConditionValue(const std::string& name);
	
};

} // namespace

#endif
