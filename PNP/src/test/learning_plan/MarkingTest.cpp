#include "MarkingTest.h"

#include "../testpnp.h"

#include <pnp/learning_plan/Learner.h>
#include <pnp/learning_plan/algo/TDLambda.h>
#include <pnp/learning_plan/Marking.h>

#include "../configTest.h"

using namespace std;


namespace learnpnp {

CPPUNIT_TEST_SUITE_REGISTRATION( MarkingTest );

void MarkingTest::setUp() {
    m1.push_back(1);
    m1.push_back(0);

    m2.push_back(0);
    m2.push_back(1);

    pars.alpha = 0.3;
    pars.gamma = 1;
    pars.lambda = 0;

}

void MarkingTest::tearDown() {}

void MarkingTest::testReadVFunctionFromTxt() {

	string path = PLAN_DIR + string("testMarking.txt");

    TDLambda learn(path, pars);

    double vm1 = learn.valueOf(m1);
    double vm2 = learn.valueOf(m2);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.1, vm1, 0.000000001);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2, vm2, 0.000000001);

}

void MarkingTest::testReadNonExistingFile() {
    string path = "AFileThatDoesntExist.txt";

	TDLambda *learn;
	CPPUNIT_ASSERT_NO_THROW(learn = new TDLambda(path, pars));

	double vm1 = learn->valueOf(m1);
	double vm2 = learn->valueOf(m2);
	
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0., vm1, 0.000000001);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0., vm2, 0.000000001);
}

void MarkingTest::testReadMisTypedFile() {
	string path = PLAN_DIR + string("misTyped.txt");

    TDLambda learn(path, pars);

    double vm1 = learn.valueOf(m1);
    double vm2 = learn.valueOf(m2);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.1, vm1, 0.000000001);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2, vm2, 0.000000001);
}

}
