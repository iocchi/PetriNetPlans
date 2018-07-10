/*
 * petrinet.h
 * author: Vittorio Amos Ziparo
 */

#ifndef PNP_PETRINET_H
#define PNP_PETRINET_H

#include <vector>
#include <string>
#include <set>
#include <map>
#include <iostream>
#include <algorithm>
#include <functional>

#include "nodes.h"
#include "utils.h"

namespace PetriNetPlans {

template<typename PlaceClass, typename TransitionClass>
class PetriNet {
public:
	virtual ~PetriNet();
	PetriNet();
	//Matteo: added copy constructor to make deep copies
	PetriNet(const PetriNet<PlaceClass, TransitionClass> &other);
	
	inline PetriNet(std::string n) { name=n; }
	inline std::string getName() { return name; }
	inline std::set< PlaceClass*> getPlaces() { return places; }
	void resetInitialMarking();
    void setInitialMarking(std::vector<std::string> placenames);
	virtual bool fireAll();
	bool isEnabled(TransitionClass* );
	bool fire(TransitionClass* );
	bool addEdge(PlaceClass* p, TransitionClass* t,unsigned int weight=1, bool inhibitor=false);
	bool addEdge(TransitionClass* t, PlaceClass* p,unsigned int weight=1);
	void printCurrentMarking();
	virtual void clearAll();
	std::set<TransitionClass*> getAllEnabledTransitions();
	std::set<PlaceClass*> getNonEmptyPlaces();

protected:
  friend class XMLPnpPlanInstantiator; //FIXME very bad hack... remove it when the instantiator
                                       //is rewritten and its code is  acceptable
	std::string name;
	std::multimap<TransitionClass*, EdgePlace<PlaceClass> > outputPlaces;
	std::multimap<TransitionClass*, EdgePlace<PlaceClass> > inputPlaces;
	std::set<TransitionClass*> transitions;
	std::set<PlaceClass*> places;
	std::map<PlaceClass*, std::set<TransitionClass*> > outputTransitions;

};

#include "petrinet.hpp"

} // namespace

#endif // PNP_PETRINET_H


