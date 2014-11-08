#ifndef MCESP_SAA__guard_h
#define MCESP_SAA__guard_h

#include <pnp/learning_plan/algo/LoggingController.h>
#include <pnp/learning_plan/algo/MonteCarlo.h>

#include <queue>
#include <set>

namespace learnpnp {

class ExploitWithHysteresis;


class SelectiveDynamicAlpha;

struct MCESP_SAA_Params : public MonteCarlo::ParamType {
	int current_episode;
	int stage_length;
};

/**
 * @brief Implementation of MCESP_SAA, an algorithm for local optimization in N-MDPs
 *
 * This class implements MCESP_SAA as presented in:
 * T. J. Perkins. Reinforcement learning for POMDPs based on action values and
 * stochastic optimization. In Proceedings of the National Conference on Artiﬁcial
 * Intelligence, pages 199–204, 2002.
 *
 * The algorithm proceeds in stages, evaluating one choice point at a time. The stage length may be
 * set through MCESP_SAA_Params.
 *
 * \author Matteo Leonetti
 **/
class MCESP_SAA : public learnpnp::LoggingController {
public:

typedef MCESP_SAA_Params ParamType;

/**
 * @brief ctor
 *
 * @param file the file to store the value function
 * @param params parameters for the algorithm, must be of type MCESP_SAA_Params
 * @param learning whether the controller should learn or evaluate the current best policy. Default true.
 **/
MCESP_SAA(const std::string& file, const ParamType& params, bool learning = true);

virtual ~MCESP_SAA();
	
virtual int choose(const Marking& current, const std::vector<Marking> &states);

virtual void updateV(const Marking &current, double reward,const Marking &next);

virtual void tick();

private:

	void rotate_queue();
	void grow_queue(const Marking& current, const std::vector<Marking> &states);

	//owns
	int current_episode;
	int stage_length;
	ExploitWithHysteresis *exp_strategy;
	

	//doesn't own
	SelectiveDynamicAlpha* sched;
	
	typedef std::map<Marking, Marking > Policy;

	std::queue< std::pair<Marking, Marking> > *action_queue;
	std::set<Marking> *choice_points;

	static std::map< std::string, std::queue< std::pair<Marking, Marking> > > queue_map;
	static std::map< std::string, std::set<Marking> > choice_map;
};

}

#endif
