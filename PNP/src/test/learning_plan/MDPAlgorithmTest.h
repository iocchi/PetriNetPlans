#ifndef learnpnp_MDPAlgorithmTest_h__guard
#define learnpnp_MDPAlgorithmTest_h__guard

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

namespace learnpnp {

class MDPAlgorithmTest : public CppUnit::TestFixture {

	CPPUNIT_TEST_SUITE( MDPAlgorithmTest );
	CPPUNIT_TEST( testTDLambda );
 	CPPUNIT_TEST( testQLearning );
	CPPUNIT_TEST( testMonteCarlo );
 	CPPUNIT_TEST( testDQLearning );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testTDLambda();
	void testQLearning();
	void testMonteCarlo();
	void testDQLearning();

};

}

#endif
