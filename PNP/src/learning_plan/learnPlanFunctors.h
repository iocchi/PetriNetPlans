#ifndef learnpnp_LearnPlanFunctors_h__guard
#define learnpnp_LearnPlanFunctors_h__guard

#include <set>
#include <vector>

#include <pnp/basic_plan/basic_plan.h>
#include <pnp/pnp_plan.h>
#include <pnp/pnpfwd.h>

#include <pnp/learning_plan/learnPlan.h>

namespace learnpnp {


typedef std::multimap<PetriNetPlans::PnpTransition* , PetriNetPlans::EdgePlace<PetriNetPlans::PnpPlace> > PlaceMultiMap;
typedef std::pair<PetriNetPlans::PnpTransition* const, PetriNetPlans::EdgePlace<PetriNetPlans::PnpPlace> > PairInPlaces;

struct CheckInputPlacesAndAdd {

CheckInputPlacesAndAdd( LearnPlan *plan, std::set<PetriNetPlans::PnpTransition*>  &conflicting,
						std::set<PetriNetPlans::PnpTransition*>& enabledTransitions);

 LearnPlan *plan;
 std::set<PetriNetPlans::PnpTransition*>  &conflictingTrans;
 std::set<PetriNetPlans::PnpTransition*> &enabledTransitions;

 
 void operator() (PairInPlaces& couple);
 };
 
 
 typedef std::vector< std::set<PetriNetPlans::PnpTransition*> > VectorOfSet;
 
struct CheckForConflictAndAdd {

CheckForConflictAndAdd( LearnPlan *, VectorOfSet&, const  std::set<PetriNetPlans::PnpTransition*>& );

LearnPlan *plan;
VectorOfSet  &conflictingSets;
std::set<PetriNetPlans::PnpTransition*> enabledTransitions;

void operator() (PetriNetPlans::PnpTransition* transition);

};

struct FireTransition {

const PlaceMultiMap &inputs;
const PlaceMultiMap &outputs;
std::map< std::string , int> &marking;

FireTransition(const PlaceMultiMap&,const PlaceMultiMap&, std::map< std::string , int>&);

void operator()(PetriNetPlans::PnpTransition*);

};

}

#endif
