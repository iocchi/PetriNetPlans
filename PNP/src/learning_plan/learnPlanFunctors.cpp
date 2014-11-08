#include "learnPlanFunctors.h"

#include <algorithm>

using namespace std;
using namespace PetriNetPlans;

namespace learnpnp {


CheckInputPlacesAndAdd::CheckInputPlacesAndAdd( LearnPlan *plan,
													set<PetriNetPlans::PnpTransition*>  &conflictingTrans,
												    std::set<PetriNetPlans::PnpTransition*>& enabledTrans) :
		plan(plan),
		conflictingTrans(conflictingTrans),
		enabledTransitions(enabledTrans){}


 void CheckInputPlacesAndAdd::operator() (PairInPlaces& couple) {
 
	PnpPlace *inplace = couple.second.p;
 	
 	if(inplace->currentMarking == 1) {
 	
 		const std::set<PnpTransition*> &outtrans = plan->outputTransitions[inplace];
		//I used to call plan->getEnabledTransitions() here instead of passing enabledTransitions around
		//but profiling showed this was very costly.
		vector<PnpTransition*> intersection;
 		set_intersection(enabledTransitions.begin(), enabledTransitions.end(), outtrans.begin(), outtrans.end(), back_inserter(intersection));
 		conflictingTrans.insert(intersection.begin(), intersection.end());
 	}
 	
 }


CheckForConflictAndAdd::CheckForConflictAndAdd( LearnPlan *plan, VectorOfSet &vec, const  std::set<PetriNetPlans::PnpTransition*>& transitions) : 
				plan(plan),
				conflictingSets(vec),
				enabledTransitions(transitions){}



void CheckForConflictAndAdd::operator() (PnpTransition* transition) {

	//now we check for other conflicting transitions
	pair<PlaceMultiMap::iterator, PlaceMultiMap::iterator> range = 
										plan->inputPlaces.equal_range(transition);
	
	set<PetriNetPlans::PnpTransition*> subset;
	subset.insert(transition); //at least the current transition is in the set
	
	for_each(range.first, range.second,CheckInputPlacesAndAdd(plan,subset,enabledTransitions));

	conflictingSets.push_back(subset);

}


FireTransition::FireTransition(const	PlaceMultiMap& in, 
								const PlaceMultiMap& out, 
								std::map< std::string , int>& mark) :
									inputs(in),
									outputs(out),
									marking(mark) {}

void FireTransition::operator()(PetriNetPlans::PnpTransition* transition) {

	pair<PlaceMultiMap::const_iterator, PlaceMultiMap::const_iterator> rangeIn = 
										inputs.equal_range(transition);
	
	PlaceMultiMap::const_iterator inArc = rangeIn.first;
	for(; inArc != rangeIn.second; ++inArc) {
		string id = inArc->second.p->nodeId;
		marking[id] -= inArc->second.weight;
	}
	
	pair<PlaceMultiMap::const_iterator, PlaceMultiMap::const_iterator> rangeOut =
										outputs.equal_range(transition);
	
	PlaceMultiMap::const_iterator outArc = rangeOut.first;
	for(; outArc != rangeOut.second; ++outArc) {
		string id = outArc->second.p->nodeId;
		marking[id] += outArc->second.weight;
	}

}

}

