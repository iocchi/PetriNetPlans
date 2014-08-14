/*
 * pnp_action.h
 * authors: Vittorio Amos Ziparo and Daniele Calisi
 */

/**
*\file pnp_action.h
* \brief Contains the definition of PnpAction and a few macros usuful to deal with it
*/

#ifndef PNP_ACTION_H
#define PNP_ACTION_H

#include <string>
#include <map>
#include <string>
#include <stdexcept>

#include "pnp_executable.h"

/**
*\brief A macro for helping in declaring action factory for a specific action
*/
#define ACTION_FACTORY(actionName) \
	PetriNetPlans::ActionFactory action_factory_ ## actionName(# actionName, new actionName())

/**
*\brief A macro for implementing a simple default clone that calls the copy constructor
*/
#define ACTION_DEFAULT_CLONE(Klass) \
	PetriNetPlans::PnpAction* clone() const { return new Klass(*this); }

/**
*\name Macros for helping declaring and initializing parameters
*/
//@{
#define PARAM(t, a) params.insert(std::make_pair(#a, std::make_pair(t, &a)));
#define PARAM_INT(a) PARAM("int", a);
#define PARAM_DOUBLE(a) PARAM("double", a);
#define PARAM_FLOAT(a) PARAM("float", a);
#define PARAM_BOOL(a) PARAM("bool", a);
#define PARAM_STRING(a) PARAM("string", a);
//@}

namespace PetriNetPlans {

/**
*\brief Base class for all actions.
*
* In order to create your own actions subclass this class and override
* the methods you need from the interface of PnpExecutable. You don't have
* to implement all of them. Also, pay attention to the termination condition.
* If you want it to be \e internal you must override finished() and have it
* return \c true when your action internally detects that its goal has been reached.
* On the other hand, if you want the termination to be \e external leave finished()
* to its default behavior and add a termination condition in the plan associated
* to the call of the method end().
*
* \author Vittorio Amos Ziparo
* \author Daniele Calisi
*/
class PnpAction : public PnpExecutable
{
public:

    /**
	* \brief dtor
	*/
	virtual ~PnpAction() { }

	/**
	*\name PnpExecutable interface
	*/
	//@{
	
	virtual void start();
	virtual void resume();
	virtual void end();
	virtual void interrupt();
	virtual void fail();
	
	/**
	*\brief Execute a step of a time extended executable.
	*
	* If this is a temporary extended action it is just supposed to execute
	* a step of that action. If the action is instantaneous this method
	* can be not implemented, you can use end() instead (don't use start or
	* the action will not be destroyed.
	*/
	virtual void executeStep();

	virtual bool finished() { return false; }
	virtual bool failed() { return false; }
	//@}


	/**
	*\name Parameter setting
	*
	* \attention this functions have not been thouroghly tested yet and the
	* functionality of automatically loading parameters is still under
	* development.
	*
	*/
	//@{
	/**
	*\brief Sets default parameter values
	*
	* In case some parameter is missing from the string provided to init()
	* this method sets the parameters to their default value. You can override
	* this method using the PARAM_* macros
	*/
	virtual void initParams() { }
	
	/**
	*\brief Initializes the parameters of this action parsing the input string
	*
	* The input string must be a comma-separated list of name=value.
	* If initiParams() correctly initialized every parameter with its type
	* this function will parse each value to its right type. The types supported
	* at the moment are \c int \c double and \c string.
	*
	* Example: maxXspeed=2.1, maxYspeed=1.0
	* \param params is a comma-separated string of name=value pairs.
	* \throw std::invalid_argument if the string \p params contains parameters that
	* do not exist or their type is not recognized
	*/
	virtual void init(const std::string& params)  throw(std::invalid_argument);
	
	/**
	*\brief Sets a single parameter from its string representation
	*
	* You can call this function to set single parameters, and it is also
	* used by init(). 
	* \param paramName is the name of the parameter you want to set. It must have
	* been initialized by initParams first to get its right type.
	* \param value is a string representation of your parameter, for 
	* instance "2.0" for a double.
	*\throw std::invalid_argument if the \p paramName does not correspond to any parameter
	* or its type is not recognized
	*/
	virtual void setParamByName(const std::string& paramName, const std::string& value)  throw(std::invalid_argument);
	//@}
	
	/**
	*\brief Clones this object, must be implemented by subclasses if they want to use the ActionFactory.
	*
	* This method provides a default implementation so that subclasses 
	* that do not use ActionFactory for their instantiation will not 
	* have to implement it. You can also use the macro #ACTION_DEFAULT_CLONE()
	*/
	virtual PnpAction* clone() const {throw std::runtime_error("This class does not prove a clone() method");}

protected:
    /**
	*\brief A map internally used by the parameter system to store parameters and their type.
	*/
	std::map<std::string, std::pair<std::string, void*> > params;
};

/**
	*\brief Stores the prototypes of the actions in a map that the ExecutableInstantiator can use to return actions.
	*
	* In order to make use of ActionFactory you must declare a static variable
	* for each action you want to generate. Before main() is invoked, when
	* static variables are created those actions will be stored in a map
	* that you can use in your ExecutableInstantiator.
	*
	* Here is an example.
	* MyAction.cpp:
	\code
	PetriNetPlans::ActionFactory action_factory_MYNAME("MYNAME", new MyAction()); 
	\endcode
	* 
	*You can also use the macro #ACTION_FACTORY()
	*
	*"MYNAME" is the name of the action, that is the string by which the action
	*is referred to in the plans.
	*
	*/
class ActionFactory {
public:
	
  /**
  *\brief Stores the new action in the map
  *
  *\param actionName is the name of the action. The name is the string by which
  * the action is referred to in the plans.
  *\param actionObject is the prototype of an action that will be cloned
  * by forName()
  */
  ActionFactory(const std::string& actionName, PnpAction* actionObject);
	
  /**
  *\brief Clones and initializes an action
  *
  * Call this method from your ExecutableInstantiator to clone the prototype
  * of the action stored in the map and initialize it with the param string.
  *
  *\param actionName is the name of the action you want to obtain
  *\param params is the string to pass to PnpAction::init() to the have the
  * action parameters initialized.
  *\sa PnpAction::init()
  */
  static PnpAction* forName(const std::string& actionName, const std::string& params = "");
};

} // namespace


#endif //_PNP_ACTION_
