%{
//included in the hpp file
namespace PetriNetPlans {
class ConditionScanner;
class ConditionChecker;
}

%}

%skeleton "lalr1.cc"
%require "2.3"
%output="ConditionParser.cpp"

%defines
%union
{
    bool            bval;
    std::string     *sval;
 };

%{
//included in the cpp file
#include <string>

#include <pnp/parser/ConditionScanner.h>
#include <pnp/conditionchecker.h>

using namespace PetriNetPlans;

#define yylex theLexer->yylex
%}

%parse-param {PetriNetPlans::ConditionScanner* theLexer}
%parse-param {PetriNetPlans::ConditionChecker* condChecker}

%token AND 
%token OR 
%token NOT
%token OP_BRACKET 
%token CL_BRACKET
%token <sval> ATOMIC_COND


%type <bval> cond
%type <bval> inCond
%type <bval> atomic
%type <bval> andList
%type <bval> orList

%destructor { delete $$; } ATOMIC_COND


%%

cond: OP_BRACKET  inCond   CL_BRACKET {$$ = $2; condChecker->lastResult = $$;}
      | atomic        {$$ = $1; condChecker->lastResult = $$;}

inCond: AND andList  {$$ = $2; condChecker->lastResult = $$;}
      | OR orList {$$ = $2; condChecker->lastResult = $$;}
      | NOT cond {$$ = !$2; condChecker->lastResult = $$;}
      | atomic {$$ = $1; condChecker->lastResult = $$;}
      
andList:   andList cond {$$ = $1 && $2}
         | cond cond {$$ = $1 && $2};
         
orList: cond orList {$$ = $1 || $2}
        | cond cond {$$ = $1 || $2};

atomic: ATOMIC_COND {$$ = condChecker->evaluateAtomicCondition(*$1); 
                     delete $1;
                     condChecker->lastResult = $$;};

%%


void yy::parser::error (const location_type& , const std::string& ) {}
