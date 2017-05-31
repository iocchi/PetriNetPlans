#ifndef PetriNetPlans_ConditionScanner_Guard
#define PetriNetPlans_ConditionScanner_Guard
 
#include <iosfwd>
 
#include "ConditionParser.hpp"

class yyFlexLexer;

namespace PetriNetPlans {

/**
*\brief A helper class for lexer-parser comunication
*
*\author Matteo Leonetti
*/
class ConditionScanner {

public:
        /**
        *\attention Both the input and the output stream can be set
        *to NULL, calling ::setString(const std::string&) to set the 
        *input stream to an istringstream.
        *
        *\param arg_yyin the stream from which reading characters, defaults to cin
        *\param arg_yyout the stram to which write output, defaults to cout
        */
        explicit ConditionScanner(std::istream* arg_yyin = 0, std::ostream* arg_yyout = 0);

        /**
        *\brief the lexer method
        */
        int yylex(yy::parser::semantic_type* v);
        
        /**
        *\brief changes the input stream of the lexer with a new
        *         istringstream
        *\param s the string the lexer must analyse
        */
        void setString(const std::string& s);

        /**
        *dtor
        */
        virtual ~ConditionScanner();
private:
    std::istream* inputStream;
    yyFlexLexer* lexer;
};

}

#endif

