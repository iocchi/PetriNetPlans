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

    int l = condition.size();
    string cnd = condition;

    if (condition.substr(0,2)=="(("  &&  condition.substr(l-2,2)=="))" )
        cnd = condition.substr(1,l-2);
    
// PNP_OUT("Parsing " << condition << " begin ...");
    theLexer->setString(cnd);
    int r = theParser->parse();
    // PNP_OUT("Parsing " << condition << " ... end");
    if(r == 0)
        return lastResult;
     
    PNP_ERR("parsing failed on condition: "<< cnd);
    
    return false; 

}
