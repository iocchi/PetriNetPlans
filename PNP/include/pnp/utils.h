#ifndef _PETRI_NET_PLANS_UTILS_
#define _PETRI_NET_PLANS_UTILS_

#include <string>
#include <vector>
#include <iostream>

/*
#include <rdkcore/logging/logging.h>

#ifndef LOGGING_MODULE
#define LOGGING_MODULE "Pnp Executor Library"
#endif
*/
#define PNP_OUTPUT_ENABLED 0
#define PNP_ERRORS_ENABLED 1

#ifndef PNP_OUT

	#if PNP_OUTPUT_ENABLED
		#define PNP_OUT(arg) {std::cerr << arg << std::endl;}
	#else
		#define PNP_OUT(arg) {}
	#endif

#endif

#ifndef PNP_ERR
	#if PNP_ERRORS_ENABLED
		#define PNP_ERR(arg)  {std::cerr << arg << std::endl;}
	#else
		#define PNP_ERR(arg)  {}
	#endif

#endif

namespace PetriNetPlans {

std::vector<std::string> tokenize(std::string s, const std::string& seps);
std::vector<std::string> tokenizeWithQuotes(std::string s, const std::string& seps);
std::string trim(const std::string& s);

bool parsePlaceString(const std::string& PlaceString, std::string& functionToCall, int& goalMarking, int& failMarking);
bool parseTransitionString(const std::string& TransitionString, std::string& guardCondition,
	std::vector<std::string>& functionsToCall);
} // namespace

#endif
