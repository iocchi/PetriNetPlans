#include <pnp/learning_plan/learnPlan.h>


#include <boost/mem_fn.hpp>
#include <boost/bind.hpp>
#include <algorithm>
#include <cstdlib>


using namespace std;
using namespace PetriNetPlans;

#include "learnPlanFunctors.h"

namespace learnpnp {

LearnPlan::LearnPlan(PetriNetPlans::ExecutableInstantiator* ei,
                     learnpnp::RewardCollector* ec,
                     Controller* controller,
                     const std::string name) :

        PnpPlan(ei,ec,name),
        controller(controller),
        transitionFired(false),
        collector(ec) {}

LearnPlan::LearnPlan(const PetriNetPlans::PnpPlan& networkStructure, Controller *controller) :
	PnpPlan(networkStructure),
	controller(controller),
	transitionFired(false) {

		collector = dynamic_cast<RewardCollector*>(conditionChecker);
		if(collector == NULL)
			throw std::runtime_error("Cast from ExternalConditionChecker to RewardCollector failed. This network cannot be used for LearnPlan");
}

LearnPlan::~LearnPlan() {
	if(controller!=NULL) {
		vector<int> finalMarking(1,0);
		Marking final(finalMarking,"finalMarking");
		controller->updateV(currentMarking(),collector->reward(),final);
		delete controller;
	}
}

std::set<PetriNetPlans::PnpTransition*> LearnPlan::getAllEnabledTransitions() {

    set<PnpTransition*> allEnabledTr = PnpPlan::getAllEnabledTransitions();

    int numEnabled = allEnabledTr.size();
    transitionFired = numEnabled;
    if (numEnabled <= 1) return allEnabledTr; // don't call evaluate and all the rest

    vector< set<PnpTransition*> > conflictingSets = separateConflictingTransitions(allEnabledTr);

    set<PnpTransition*> currentSet;
    set<PnpTransition*> notAvailable;
    vector< set<PnpTransition*> > possibleSets;

    computeAllTransitionSets(conflictingSets.begin(), conflictingSets.end(),currentSet,notAvailable, possibleSets);

    if (possibleSets.size() == 1) return possibleSets[0]; //not much to choose

    vector<Marking> possSetsMarkings;
    transform(	possibleSets.begin(),
               possibleSets.end(),
               back_inserter(possSetsMarkings),
               boost::bind(&LearnPlan::simulateFireing,this,_1)); //calls simulateFireing on every element of possibleSets


    int chosen = controller->choose(curmar,possSetsMarkings);

    return possibleSets[chosen];

}

void LearnPlan::executeStep() {
    curmar = currentMarking(); //this is the instance variable

    PnpPlan::executeStep();

    if (transitionFired) {
        map<string,int> marking = currentMarking();
        double reward = collector->reward();
        controller->updateV(curmar,reward,marking);
    }
    else
        controller->tick();
}

void LearnPlan::computeAllTransitionSets(	std::vector< std::set<PetriNetPlans::PnpTransition*> >::iterator current,
        std::vector< std::set<PetriNetPlans::PnpTransition*> >::iterator end,
        std::set<PetriNetPlans::PnpTransition*> &choice,
        std::set<PetriNetPlans::PnpTransition*> &notAvailable,
        std::vector< std::set<PetriNetPlans::PnpTransition*> > &possibileSets) const {

    /*This algorithm is complicated, it's not the code to be dirty :)
    	* Let's consider an example. We have three transitions: t1, t2, t3.
    	* The conflicting sets are:
    	* {t1, t2}, {t1,t2,t3}, {t2,t3}
    	* a set is conflicting iff only one transition in the set may be fired
    	* for it removes the token that enables the others.
    	*
    	* This algorithm computes all the sets of transitions that can be fired together and
    	* returns them
    	*
    	*step by step:
    	*
    	*----
    	*current = {t1, t2}
    	*choice = {} , notAvailable = {}
    	*
    	*the intersection of current and choice is empty, therefore current is
    	*considered.
    	*the difference of current and notAvailable is {t1, t2} so both transitions must
    	*be used while backtracking. The first choice is t1 *remember this point to backtrack
    	*
    	*----
    	*current = {t1,t2,t3}
    	*choice = {t1} , notAvailable = {t1,t2}
    	*t2 is not available because it conflicts with t1.
    	*here the intersection of current and choice is not empty, therefore current is
    	*skipped.
    	*
    	*----
    	*current = {t2,t3}
    	*choice = {t1} , notAvailable = {t1,t2}
    	*the intersection of current and choice is empty, therefore current is fine.
    	*difference = {t3} is computed and t3 is added to choice
    	*choice = {t1,t3} is valid.
    	*
    	*----
    	*backtrack to the first choice. t2 is chosen this time:
    	*
    	*----
    	*current = {t1,t2,t3}
    	*choice = {t2} , notAvailable = {t1,t2}
    	*intersection not empty then skipped
    	*
    	*----
    	*current = {t2,t3}
    	*choice = {t2} , notAvailable = {t1,t2}
    	*again intersection not empty, the skipped. {t2} is indeed a correct solution
    	*/

    if (current == end) {
        //if best is still empty the current solution becomes the current best
        //there must be at least one solution if we are here therefore an empty best
        //is a wrong choice to compare the current one with, because the empty (wrong)
        //set could be better then the current (acceptable) one.

        possibileSets.push_back(choice);
        return;
    }


    vector<PnpTransition*> intersection;
    set_intersection (current->begin(),current->end(),
                      choice.begin(), choice.end(),
                      back_inserter(intersection));

    if (intersection.empty()) {

        vector<PnpTransition*> difference;
        set_difference (current->begin(),current->end(),
                        notAvailable.begin(),notAvailable.end(),
                        back_inserter(difference));

        set<PnpTransition*> copyAvail(notAvailable);
        copyAvail.insert(difference.begin(), difference.end());

        vector<PnpTransition*>::iterator setElement = difference.begin();
        for (; setElement != difference.end(); ++setElement) {

            set<PnpTransition*> copy(choice);
            copy.insert(*setElement);

            computeAllTransitionSets(current+1, end,copy,copyAvail, possibileSets);
        }
    }
    else
        computeAllTransitionSets(current+1, end,choice,notAvailable, possibileSets);

}

map<string,int>
LearnPlan::simulateFireing(const std::set<PetriNetPlans::PnpTransition*> &transitions) const {

    map<string,int> marking(curmar);

    FireTransition fire(inputPlaces,outputPlaces,marking);

    for_each(transitions.begin(), transitions.end(), fire);

    return marking;
}

vector< set<PetriNetPlans::PnpTransition*> >
LearnPlan::separateConflictingTransitions(const set<PnpTransition*>& transitions )  {

    vector< set<PetriNetPlans::PnpTransition*> > resultvec;

    CheckForConflictAndAdd checker(this, resultvec,transitions);

    for_each(transitions.begin(), transitions.end(), checker);

    return resultvec;

}

}


