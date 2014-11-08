#include <pnp/learning_plan/algo/DelayedQLearning.h>

#include <pnp/learning_plan/algo/TDLambda.h>
#include <pnp/learning_plan/exp/Exploit.h>

#include <limits>
#include <iostream>

using namespace std;

namespace learnpnp {

static TDLParams  processParams(const DelayedQLearning::ParamType& parameters) {
    TDLParams par;
    par.lambda = 0;
    par.alpha = 1.;
    par.gamma = 0; //we take control of updates

    par.initialValue = (parameters.gamma != 1)? 1/(1 - parameters.gamma)
                       : numeric_limits<double>::max();

    return par;
}

DelayedQLearning::DelayedQLearning(const std::string &filePath,
                                   const DelayedQLearning::ParamType& parameters) :
        BasicController(new TDLambda(filePath,processParams(parameters)),new Exploit(false)),
        filePath(filePath),U(),l(),t(),LEARN(),t_star(0),t_cur(0),
        params(parameters) {
    readVFunctionFromTxt(filePath+"_U",U);
    readVFunctionFromTxt(filePath+"_l",l);
    readVFunctionFromTxt(filePath+"_t",t);
    readVFunctionFromTxt(filePath+"_LEARN",LEARN);

    vector<int> vec;
    vec.push_back(0);

    Marking cur(vec, "t_cur");
    t_cur = t[cur];

    vec[0] = 1;
    Marking star(vec,"t_star");

    t_star = t[star];

    vector<int> finalMarking(1,0);
    learner->updateV(finalMarking,0.,finalMarking);

}

void  DelayedQLearning::tick()  {
    BasicController::tick();
    t_cur++;
}

void DelayedQLearning::updateV(const Marking &current, double rewardIn,const Marking &next) {

    t_cur++;

//     cout << current.getId() <<  " v=" << learner->valueOf(current) << " " ;

    double reward  = (rewardIn - params.r_min) / (params.r_max - params.r_min); // bound reward in [0,1]

    LoggingController::visited(current);

    if (LEARN.find(current) == LEARN.end())
        LEARN[current] = true;


    if (/*LEARN[current]*/true) {

        U[current] += reward + params.gamma * learner->valueOf(next);
        l[current] += 1;

//         cout << "reward= " << reward << " l= " << l[current] << " U= " << U[current] << " " << endl;

        if (l[current] == params.m) {
//             cout << " average= " << U[current] << " ";

            if (learner->valueOf(current) - (U[current]/params.m) >= 2 * params.epsilon_1) {

                BasicController::updateV(current,((U[current]/params.m)+params.epsilon_1),next); //next is irrelevant

                cout << current.getId() << " " << ((U[current]/params.m)+params.epsilon_1) << endl;

                //because of how we set the parameters in processParams()
                t_star = t_cur;
            } else {
                if ( t[current] >= t_star) {
                    LEARN[current] = false;
                }
            }

            t[current] = t_cur;
            U[current] = 0;
            l[current] = 0;
        }

    } else {
        if ( t[current] < t_star)
            LEARN[current] = true;
    }


//     cout << "v= " << learner->valueOf(current) << endl;

}

DelayedQLearning::~DelayedQLearning() {

    vector<int> vec;
    vec.push_back(0);

    Marking cur(vec, "t_cur");
    t[cur] = t_cur;

    vec[0] = 1;
    Marking star(vec,"t_star");

    t[star] = t_star;


    writeVFunctionToTxt(filePath+"_U",U);
    writeVFunctionToTxt(filePath+"_l",l);
    writeVFunctionToTxt(filePath+"_t",t);
    writeVFunctionToTxt(filePath+"_LEARN",LEARN);
}

}

