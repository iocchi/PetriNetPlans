#include <pnp/conditionchecker.h>

#include <pnp/utils.h>
#include <pnp/parser/ConditionScanner.h>

#include <pnp/parser/ConditionParser.hpp>

using namespace std;

PetriNetPlans::ConditionChecker::ConditionChecker() : 
        theLexer(new ConditionScanner()), theParser(new yy::parser(theLexer,this)) {}


PetriNetPlans::ConditionChecker::ConditionChecker(const ConditionChecker&) :
	theLexer(new ConditionScanner()), theParser(new yy::parser(theLexer,this)) {}



PetriNetPlans::ConditionChecker::~ConditionChecker() {

    delete theParser;
    delete theLexer;
}

bool PetriNetPlans::ConditionChecker::evaluateCondition(const string& condition) {

    theLexer->setString(condition);
    if(theParser->parse() == 0)
        return lastResult;
     
    PNP_ERR("parsing failed on condition: "<< condition);
    
    return false; 

}
