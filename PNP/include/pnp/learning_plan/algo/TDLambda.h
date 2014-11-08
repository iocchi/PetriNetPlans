#ifndef learnpnp_TDLambda_h__guard
#define learnpnp_TDLambda_h__guard

#include <pnp/learning_plan/Learner.h>
#include <pnp/learning_plan/Marking.h>
#include <pnp/learning_plan/algo/TDParams.h>
#include <pnp/learning_plan/algo/AlphaScheduler.h>

namespace learnpnp {

/**
*\brief An on-policy learning algorithm based on TD(lambda)
*
*\author Matteo Leonetti
*/
class TDLambda : public Learner {

public:

typedef TDLParams ParamType;
  
/**
*\brief ctor.
*
*\param filePath is the file to store and retrieve the value function
*\param parameters is the collection of the parameters for the algorithm
*/
TDLambda(const std::string &filePath, const TDLParams& parameters);

TDLambda(const std::string &filePath, const TDLParams& parameters, AlphaScheduler *sched);

//gets documented by the base class
virtual void updateV(const Marking &current, double reward,const Marking &next);

virtual void tick();

virtual void finalizeV();

virtual ~TDLambda();


private:

std::map<Marking, double> e;
std::map<Marking, double> memory;
double lastV;
double last_e_s;
Marking lastMarking;
TDLParams par;
int ticks;
AlphaScheduler *scheduler;

};

}

#endif
