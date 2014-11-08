#ifndef learnpnp_LearnPlanTest_h__guard
#define learnpnp_LearnPlanTest_h__guard

#include <set>

#include <pnp/learning_plan/learnPlan.h>


#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

namespace learnpnp {

template <typename T>
class CompareId {
public:
	CompareId(std::string id) : id(id) {}

	bool operator() (T *node) {
		return node->nodeId == id;
	}
private:
	std::string id;
};

class LearnPlanTest : public CppUnit::TestFixture {
	
	CPPUNIT_TEST_SUITE( LearnPlanTest );
	CPPUNIT_TEST( testCheckInAndAdd );
	CPPUNIT_TEST( testCheckForConflict );
  	CPPUNIT_TEST( testEnabledTransitions );
  	CPPUNIT_TEST( testSimulateFireing );
  	CPPUNIT_TEST_SUITE_END();

public:
  void setUp();
  void tearDown();

  void testEnabledTransitions();
  void testCheckInAndAdd();
  void testCheckInAndAdd(std::string , std::string, int);
  void testCheckForConflict();
  void testSimulateFireing();

private:
LearnPlan *plan;

template<typename T>
T *lookFor(std::string id) {

typename std::set<T*>::iterator thing = 
find_if(plan->transitions.begin(), plan->transitions.end(), CompareId<T>(id));

return *thing;

}

};

class LearnPlanTest2 : public CppUnit::TestFixture {
	
	CPPUNIT_TEST_SUITE( LearnPlanTest2 );
  	CPPUNIT_TEST( testEnabledTransitions );
  	CPPUNIT_TEST( testSimulateFireing );
  	CPPUNIT_TEST_SUITE_END();

public:
  void setUp();
  void tearDown();

  void testEnabledTransitions();
  void testSimulateFireing();

private:
LearnPlan *plan;

template<typename T>
T *lookFor(std::string id) {

typename std::set<T*>::iterator thing = 
find_if(plan->transitions.begin(), plan->transitions.end(), CompareId<T>(id));

return *thing;

}

};

}

#endif
