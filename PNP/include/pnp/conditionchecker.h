#ifndef PetriNetPlans_ConditionChecker_Guard
#define PetriNetPlans_ConditionChecker_Guard

#include <string>

#include "parser/parserfwd.h"
//#include "parser/ConditionParser.hpp"
//#include "parser/ConditionScanner.h"

namespace PetriNetPlans {

/**
*\brief The base class for parser-based condition checkers.
*
*This class uses a flex-bison parser to compute the actual value of
*a condition. Atomic condition checking is delegated to subclasses
*through the method ::evaluateAtomicCondition(const std::string&) .
*
*The grammar of conditions is as follows:<br>
* <pre>
* condition -> '(' (and|or) condition condition+ ')' |
*              '(' not condition ')'                 |
*              ATOMIC_CONDITION
* </pre>
*where "and", "or" and "not" are case insensitive.
*\author Matteo Leonetti
*/

class ConditionChecker {
public:
    /**
    *ctor
    */
    ConditionChecker();
	
	ConditionChecker(const ConditionChecker&);
    
    /**
    *\brief Parses the string \a condition computing
    *         its value.
    *
    *If the parsing fails due to sintactic errors the function
    *logs the error and returns false.
    *\param condition the condition to parse
    *\return the value of the condition if the parsing succeeds.
    *          On sintactic errors returns \a false.
    */
    virtual bool evaluateCondition(const std::string& condition);

    /**
    *dtor
    */
    virtual ~ConditionChecker();
    
    friend class yy::parser;
    
protected:
    /**
    *\brief Evaluates atomic conditions delegating it to subclasses
    *
    *If the condition cannot be evaluated function implementations must
    *log the error and return \a false
    *\attention This is the only method that a class must define
    *              to have a working ConditionChecker.
    *\param condition the condition to evaluate
    *\return the value of the condition or \a false if it can't be
    *          determined
    */
    virtual bool evaluateAtomicCondition(const std::string& condition) = 0;

private:
    ConditionScanner *theLexer;
    yy::parser *theParser;
    bool lastResult;
};

}

#endif
