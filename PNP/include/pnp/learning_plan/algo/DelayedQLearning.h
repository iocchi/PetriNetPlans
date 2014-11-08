#ifndef learnpnp_DelayedQLearning_h__guard
#define learnpnp_DelayedQLearning_h__guard

#include <pnp/learning_plan/algo/BasicController.h>
#include <pnp/learning_plan/algo/TDParams.h>

#include <list>

namespace learnpnp {

  struct DQParams {
	
	double gamma;
	double m;
	double epsilon_1;
	double r_max;
	double r_min;
  };
  
/**
*\brief An implementation of Delayed Q-Learning
*
* This implementation corresponds to the pseudo code by Strehl,
* Li, Wiewiora, Langford, Littman in "PAC model-free reinforcement learning",
* ICML 2006
*
*\author Matteo Leonetti
*/
class DelayedQLearning : public BasicController {
public:
  
  typedef DQParams ParamType;
  
  DelayedQLearning(const std::string &filePath, const ParamType& parameters);
  
  virtual void updateV(const Marking &current, double reward,const Marking &next);
  
  virtual void tick();

  virtual ~DelayedQLearning();

private:
 std::string filePath;
 std::map<Marking, double> U,l,t,LEARN;
 int t_star, t_cur;
 ParamType params;
};

}



#endif
