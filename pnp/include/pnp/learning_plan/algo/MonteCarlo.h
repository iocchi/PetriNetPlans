#ifndef learnpnp_MonteCarlo_h__guard
#define learnpnp_MonteCarlo_h__guard

#include <pnp/learning_plan/Learner.h>
#include <pnp/learning_plan/Marking.h>
#include <pnp/learning_plan/algo/TDParams.h>
#include <pnp/learning_plan/algo/AlphaScheduler.h>

namespace learnpnp {

/**
*\brief An on-policy learning algorithm based on MonteCarlo method
*
*\author Matteo Leonetti
*/
class MonteCarlo : public Learner {

public:

typedef TD0Params ParamType;

/**
*\brief ctor.
*
*\param filePath is the file to store and retrieve the value function
*\param parameters is the collection of the parameters for the algorithm
*/
MonteCarlo(const std::string &filePath, const ParamType& parameters);

MonteCarlo(const std::string &filePath, const ParamType& parameters, AlphaScheduler *sched);

//gets documented by the base class
virtual void updateV(const Marking &current, double reward,const Marking &next);

virtual void tick();

virtual ~MonteCarlo();

virtual void finalizeV();

std::map<Marking, double> e;
std::map<Marking, double> accumulatedV;
ParamType par;
int ticks;
AlphaScheduler *scheduler;

};

}

#endif
