%option noyywrap
%option c++
%option outfile="ConditionScanner.cpp"
%{
#include <string>
#include <sstream>

#include <pnp/parser/ConditionScanner.h>
#include <pnp/parser/ConditionParser.hpp>
#undef yylex //defined in ConditionParser.hpp

%}

IDENTIFIER [a-zA-Z][@a-zA-Z0-9._#{,}<>]*

%%

and         | 
AND       return yy::parser::token::AND;
or          | 
OR         return yy::parser::token::OR;
not         | 
NOT       return yy::parser::token::NOT;
\(              return yy::parser::token::OP_BRACKET;
\)              return yy::parser::token::CL_BRACKET;
{IDENTIFIER}    return yy::parser::token::ATOMIC_COND;
[ \t\f\n\r]   //ignore witespace, tab, newline, carriage return

%%

PetriNetPlans::ConditionScanner::ConditionScanner(std::istream* arg_yyin, std::ostream* arg_yyout) :
        inputStream(arg_yyin), lexer(new yyFlexLexer(inputStream, arg_yyout)) {}
        
PetriNetPlans::ConditionScanner::~ConditionScanner() {
    delete lexer;
    delete inputStream;
}

int PetriNetPlans::ConditionScanner::yylex(yy::parser::semantic_type* yylval) {

    int tok = lexer->yylex();
    if(tok == yy::parser::token::ATOMIC_COND)
        yylval->sval = new std::string(lexer->YYText());

    return tok;
}

void PetriNetPlans::ConditionScanner::setString(const std::string& s) {
    delete inputStream;
    inputStream = new std::istringstream(s);
    lexer->yyrestart(inputStream);
}
