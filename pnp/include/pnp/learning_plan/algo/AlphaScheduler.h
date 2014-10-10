#ifndef learnpnp_DynamicAlpha_h__guard
#define learnpnp_DynamicAlpha_h__guard

#include <pnp/learning_plan/Learner.h>

#include <map>
#include <string>

using namespace std;

namespace learnpnp {


/**
 * \brief allows to specify a different alpha for each state
 *
 * Alpha is the learning step parameter used in many algorithms based on value functions.
 * Can be sub-classed to implement various types of scheduling
 *
 * \author Matteo Leonetti
 */
struct AlphaScheduler {
	/**
	 * \brief Returns the value of alpha for a specific state
	 *
	 * \param state the marking for which the value of alpha is requested
	 * \return the value of alpha for that state
	 */
	virtual double alpha(const Marking& state) = 0;

	/**
	 * \brief Notifies the scheduler that a state has been visited
	 *
	 * \param state the state just visited
	 */
	virtual void visited(const Marking& state) = 0;

	virtual ~AlphaScheduler(){}
};

/**
 * \brief The simplest and most common scheduling, just setting alpha at a constant
 *
 * \author Matteo Leonetti
 */
class ConstantAlpha : public AlphaScheduler {
public:

	/**
	 * \brief Constructs an AlphaScheduler with a given value of alpha
	 *
	 * The value of alpha is used for every state.
	 *
	 * \param value the constant value of alpha
	 */
	explicit ConstantAlpha(double value) : value(value) {}
	
	void visited(const Marking&) {}
	
	virtual double alpha(const Marking&) {
		return value;
	}

	virtual ~ConstantAlpha() {}
	
private:
	double value;
};

/**
 * \brief A scheduler that returns 1/ number of times a state has been visited
 *
 * The value used is max(min_alpha, 1/ n(s)) where n(s) is the number of times state s has
 * been visited since the last time the scheduler was reset().
 *
 * \author Matteo Leonetti
 */
class DynamicAlpha : public AlphaScheduler {
public:

	/**
	 * \brief Constructs a dynamic alpha scheduler with a given min value for alpha
	 *
	 * \param min_alpha the minimum value that must be returned for alpha
	 * \param file a file used to store how many times each state has been visited
	 */
	DynamicAlpha(double min_alpha, const std::string& file);
	
	virtual double alpha(const Marking& state);
	void visited(const Marking& state);

	/**
	 * \brief Resets the count for each state's visits
	 */
	void reset();
	
	virtual ~DynamicAlpha();
	
private:
	double min_alpha;
	std::string file;
	std::map<Marking, double> state_count;
};
	
}

#endif
