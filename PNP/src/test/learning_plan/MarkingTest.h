#ifndef learnpnp_MarkingTest_h__guard
#define learnpnp_MarkingTest_h__guard

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <pnp/learning_plan/algo/TDParams.h>

#include <vector>

namespace learnpnp {

class MarkingTest : public CppUnit::TestFixture {
	
	CPPUNIT_TEST_SUITE( MarkingTest );
	CPPUNIT_TEST( testReadVFunctionFromTxt );
	CPPUNIT_TEST( testReadNonExistingFile );
	CPPUNIT_TEST( testReadMisTypedFile );
  	CPPUNIT_TEST_SUITE_END();

public:
  void setUp();
  void tearDown();

  void testReadVFunctionFromTxt();
  void testReadNonExistingFile();
  void testReadMisTypedFile();

private:
	std::vector<int> m1,m2;
	TDLParams pars;

};

}
#endif

