#include "MDPAlgorithmTest.h"


#include "../configTest.h"
#include "envs/env1/Env1TDLInst.h"
#include "envs/env1/Env1QInst.h"
#include "envs/env1/Env1DQInst.h"
#include "envs/RewEnv.h"

#include <pnp/pnp_executer.h>
#include <pnp/learning_plan/learnPlan.h>
#include <pnp/learning_plan/algo/TDLambda.h>
#include <pnp/learning_plan/algo/MonteCarlo.h>
#include <pnp/learning_plan/algo/DelayedQLearning.h>


#include <iostream>

using namespace std;
using namespace PetriNetPlans;

namespace learnpnp {

CPPUNIT_TEST_SUITE_REGISTRATION( MDPAlgorithmTest );

void MDPAlgorithmTest::setUp() {

}


void MDPAlgorithmTest::tearDown() {

    //move this code to setUp() if you want to keep the last value function

    //clear files
    stringstream command;
    command << "rm " << PLAN_DIR << "env1_*";
    system(command.str().c_str());
}


template<typename T1, typename T2>
static void callExecutor(T2 pars) {

    //clear files

    RewEnv rew;
    T1 *inst = new T1(&rew, pars);

    PnpExecuter<LearnPlan> executor ( inst );

    for (int i=0; i<300; ++i) {
        executor.setMainPlan ( "env1" );
        while (!executor.goalReached()) {
            executor.execMainPlanStep();
        }
        executor.setMainPlan ( "fake" ); //reset the learner
    }
}

void MDPAlgorithmTest::testTDLambda() {

    //test TD0

    TDLambda::ParamType pars;
    pars.alpha = 0.1;
    pars.lambda = 0.;
    pars.gamma = 0.8;
    pars.initialValue = 0.;

    callExecutor<Env1TDLInst<TDLambda>,TDLambda::ParamType>(pars);

    TDLambda learn(string(PLAN_DIR) + "env1_.txt", TDLambda::ParamType());

    CPPUNIT_ASSERT_DOUBLES_EQUAL(4.,learn.valueOfId("p2"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.8,learn.valueOfId("p3"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5.,learn.valueOfId("p4"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.,learn.valueOfId("p5"),0.01);

    //test TDLambda

    pars.alpha = 0.1;
    pars.lambda = 0.9;
    pars.gamma = 0.8;
    pars.initialValue = 0.;

    callExecutor<Env1TDLInst<TDLambda>,TDLambda::ParamType>(pars);

    TDLambda learn2(string(PLAN_DIR) + "env1_.txt", TDLambda::ParamType());

    CPPUNIT_ASSERT_DOUBLES_EQUAL(4.,learn2.valueOfId("p2"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.8,learn2.valueOfId("p3"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5.,learn2.valueOfId("p4"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.,learn2.valueOfId("p5"),0.01);

}

void MDPAlgorithmTest::testQLearning() {

    QLearning::ParamType par;
    par.alpha = 0.1;
    par.gamma = 0.8;
    par.initialValue = 0.;

    callExecutor<Env1QInst, QLearning::ParamType>(par);

    TDLambda learn(string(PLAN_DIR) + "env1_.txt", TDLambda::ParamType());

    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.2,learn.valueOfId("p1"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(4.,learn.valueOfId("p2"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.8,learn.valueOfId("p3"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5.,learn.valueOfId("p4"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.,learn.valueOfId("p5"),0.01);
}

void MDPAlgorithmTest::testMonteCarlo() {
    MonteCarlo::ParamType par;
    par.alpha = 0.1;
    par.gamma = 0.8;
    par.initialValue = 0.;

    callExecutor<Env1TDLInst<MonteCarlo>,MonteCarlo::ParamType>(par);

    TDLambda learn(string(PLAN_DIR) + "env1_.txt", TDLambda::ParamType());

    CPPUNIT_ASSERT_DOUBLES_EQUAL(4.,learn.valueOfId("p2"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.8,learn.valueOfId("p3"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5.,learn.valueOfId("p4"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.,learn.valueOfId("p5"),0.01);

}

void MDPAlgorithmTest::testDQLearning() {

    DelayedQLearning::ParamType par;

    par.gamma = 0.8;
    par.epsilon_1 = 0.0001;
    par.m = 1;
	par.r_max = 5;
	par.r_min = 0;
	
	callExecutor<Env1DQInst,DelayedQLearning::ParamType>(par);
	
	TDLambda learn(string(PLAN_DIR) + "env1_.txt", TDLambda::ParamType());

    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.2/5,learn.valueOfId("p1"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(4./5,learn.valueOfId("p2"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.8/5,learn.valueOfId("p3"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5./5,learn.valueOfId("p4"),0.01);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1./5,learn.valueOfId("p5"),0.01);

}


}
