#include "LearnPlanTest.h"

#include <set>
#include <algorithm>

#include <pnp/test/testpnp.h>

#include <pnp/externalconditionchecker.h>

#include <pnp/basic_plan/xml_plan_instantiator.h>
#include <pnp/test/TestInstantiator.h>
#include <pnp/pnp_nodes.h>


#include <pnp/learning_plan/learnPlanFunctors.h>

using namespace PetriNetPlans;
using namespace std;

namespace learnpnp {

CPPUNIT_TEST_SUITE_NAMED_REGISTRATION( LearnPlanTest, registryName() );
CPPUNIT_TEST_SUITE_NAMED_REGISTRATION( LearnPlanTest2, registryName() );



class LessId {
public:
	bool operator() (PnpTransition *a, PnpTransition *b) {
		return a->nodeId < b->nodeId;
	}
};

class FakeConditionChecker : public learnpnp::RewardCollector {
virtual bool evaluateAtomicExternalCondition(const std::string& atom) {
return true;
}

virtual void startCollecting() {}

virtual double reward() {return 0;}
};


void LearnPlanTest::setUp() {
FakeConditionChecker *fc = new FakeConditionChecker();
TestInstantiator *ti = new TestInstantiator(fc);

plan = static_cast<LearnPlan*>(ti->createExecutable("enabledTransitions"));

plan->resetInitialMarking();

}

void LearnPlanTest::tearDown() {
delete plan;
}


void LearnPlanTest::testEnabledTransitions() {

	set<PnpTransition*> transitionSet = plan->getAllEnabledTransitions();

	int size = transitionSet.size();

	CPPUNIT_ASSERT_EQUAL(1, size);
}

void LearnPlanTest::testCheckInAndAdd(string transitionId, string placeId, int numTrans) {

	set<PnpTransition*>::iterator t1 =  
		find_if(plan->transitions.begin(), plan->transitions.end(), CompareId<PnpTransition>(transitionId));
		
	set<PnpPlace*>::iterator p1 =  
		find_if(plan->places.begin(), plan->places.end(), CompareId<PnpPlace>(placeId));
	
	PetriNetPlans::EdgePlace<PetriNetPlans::PnpPlace> edgep;
	edgep.p = *p1;
	edgep.inhibitor = false;
	edgep.weight = 1;
	
	PairInPlaces couple = make_pair(*t1, edgep);
	
	std::set<PetriNetPlans::PnpTransition*> conflicting;
	conflicting.insert(*t1);
	
	CheckInputPlacesAndAdd checkInput(plan, conflicting);
	
	checkInput(couple);

	int size = conflicting.size();

	CPPUNIT_ASSERT_EQUAL(numTrans, size);
}

void LearnPlanTest::testCheckInAndAdd() {

	testCheckInAndAdd("t1", "p1", 2);
	testCheckInAndAdd("t3", "p2", 1);

}

void LearnPlanTest::testCheckForConflict() {

	set<PnpTransition*>::iterator t1 =  
		find_if(plan->transitions.begin(), plan->transitions.end(), CompareId<PnpTransition>("t1"));
	
	std::vector< std::set<PetriNetPlans::PnpTransition*> > vectorOfSets;
		
	CheckForConflictAndAdd checker(plan, vectorOfSets);
	checker(*t1);
	
	set<PnpTransition*>::iterator t2 =  
		find_if(plan->transitions.begin(), plan->transitions.end(), CompareId<PnpTransition>("t2"));

	checker(*t2);

	int sizeVector = vectorOfSets.size();
	CPPUNIT_ASSERT_EQUAL(2, sizeVector);

	for(int i=0; i< 2; ++i) {

		set<PetriNetPlans::PnpTransition*> trans = vectorOfSets[i];
		int sizeSet = trans.size();
	
		CPPUNIT_ASSERT_EQUAL(2, sizeSet);
	
		set<PetriNetPlans::PnpTransition*> testSet;
		testSet.insert(*t1);
		testSet.insert(*t2);
	
	
		vector<PetriNetPlans::PnpTransition*> resultSet;
		set_symmetric_difference (trans.begin(), trans.end(), testSet.begin(), testSet.end(), back_inserter(resultSet));
	
		int simmetricSize = resultSet.size();
		CPPUNIT_ASSERT_EQUAL(0, simmetricSize);
	}
}

void LearnPlanTest::testSimulateFireing() {
std::set<PetriNetPlans::PnpTransition*> transitionSet;

transitionSet.insert(lookFor<PnpTransition>("t1"));

map<string, int> marking = plan->simulateFireing(transitionSet);

int mp1 = marking["p1"];
int mp2 = marking["p2"];

CPPUNIT_ASSERT_EQUAL(0, mp1);
CPPUNIT_ASSERT_EQUAL(1, mp2);

}



void LearnPlanTest2::setUp() {
FakeConditionChecker *fc = new FakeConditionChecker();
TestInstantiator *ti = new TestInstantiator(fc);

plan = static_cast<LearnPlan*>(ti->createExecutable("enabledTransitions2"));

plan->resetInitialMarking();

}

void LearnPlanTest2::tearDown() {
delete plan;
}

void LearnPlanTest2::testEnabledTransitions() {

	set<PnpTransition*> transitionSet = plan->getAllEnabledTransitions();


		PnpTransition* t1 = lookFor<PnpTransition>("t1"); 
		PnpTransition* t6 =  lookFor<PnpTransition>("t6");

		set<PetriNetPlans::PnpTransition*> testSet1;
		testSet1.insert(t1);
		testSet1.insert(t6);
	
	
		vector<PetriNetPlans::PnpTransition*> resultSet1;
		set_symmetric_difference (transitionSet.begin(), transitionSet.end(), testSet1.begin(), testSet1.end(), back_inserter(resultSet1));
	
		PnpTransition* t2 = lookFor<PnpTransition>("t2"); 
		set<PetriNetPlans::PnpTransition*> testSet2;
		testSet2.insert(t2);
	
		vector<PetriNetPlans::PnpTransition*> resultSet2;
		set_symmetric_difference (transitionSet.begin(), transitionSet.end(), testSet2.begin(), testSet2.end(), back_inserter(resultSet2));

		CPPUNIT_ASSERT(resultSet1.empty() || resultSet2.empty());
}

void LearnPlanTest2::testSimulateFireing() {
std::set<PetriNetPlans::PnpTransition*> transitionSet;

transitionSet.insert(lookFor<PnpTransition>("t2"));

map<string, int> marking = plan->simulateFireing(transitionSet);

int mp1 = marking["p1"];
int mp3 = marking["p3"];
int mp7 = marking["p7"];
int mp8 = marking["p8"];

CPPUNIT_ASSERT_EQUAL(0, mp1);
CPPUNIT_ASSERT_EQUAL(0, mp7);
CPPUNIT_ASSERT_EQUAL(1, mp3);
CPPUNIT_ASSERT_EQUAL(1, mp8);

}

}

