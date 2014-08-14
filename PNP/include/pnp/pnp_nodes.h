/*
 * pnp_nodes.h
 * author: Vittorio Amos Ziparo and Daniele Calisi
 */

#ifndef PNP_PNP_NODES
#define PNP_PNP_NODES

#include "nodes.h"
#include "utils.h"

namespace PetriNetPlans {

struct PnpTransition : public Transition
{
	PnpTransition(const std::string& nodeId, const std::string& pFunctionsToCall,
		const std::string& guardCondition) :
		Transition(), nodeId(nodeId), pnmlString(nodeId), guardCondition(guardCondition)
	{
		functionsToCall = tokenize(pFunctionsToCall, " ");
	}

	PnpTransition(const std::string& NodeId, const std::vector<std::string>& functionsToCall,
		const std::string& guardCondition) :
		Transition(), nodeId(NodeId), pnmlString(nodeId), guardCondition(guardCondition),
		functionsToCall(functionsToCall)
	{}

	std::string nodeId;		// from pnml
	std::string pnmlString;	// from pnml
	std::string guardCondition;
	std::vector<std::string> functionsToCall;	// start, resume, etc (each is <actionName>.<funcName>) or nothing
};

struct PnpPlace : public Place
{
	/// goalMarking or failMarking = -1 means don't care
	PnpPlace(const std::string& nodeId, const std::string& functionToCall,
		unsigned int initialMarking, int goalMarking = -1, int failMarking = -1) :
		Place(initialMarking), nodeId(nodeId), pnmlString(nodeId), functionToCall(functionToCall),
		goalMarking(goalMarking), failMarking(failMarking) { }

	std::string nodeId;				// from pnml
	std::string pnmlString;			// from pnml
	std::string functionToCall;		// exec function (<actionName>.exec), or nothing
	int goalMarking;
	int failMarking;
};

} // namespace

#endif // PNP_PNP_NODES
