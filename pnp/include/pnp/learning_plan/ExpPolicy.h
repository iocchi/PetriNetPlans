#ifndef learnpnp_ExpPolicy_h__guard
#define learnpnp_ExpPolicy_h__guard

#include "Learner.h"
#include "Marking.h"

#include <vector>
#include <map>


namespace learnpnp {

/**
*\brief The interface for exploration policies based on a value function
*
* Exploration policies make decision at <i>choice points</i>, that is when there is more
* then a possible next marking, but they are conflicting for the tokens. In that case,
* only one next marking must be chosen, and sub-classes of ExpPolicy make this decision
* by looking up the value of each possible next marking. For this reason the method choose()
* requires a Learner, so that its value function may be used.
*
* Exploration policies may or may not remember previous choices at the same choice points.
* If they do remember they make a choice only the first time the choice point is encountered
* then always return the same marking.
*
*\author Matteo Leonetti
*/

class ExpPolicy {

public:

	/**
	 * \brief Creates an ExpPolicy, specifying whether or not it should remember previous choices
	 *
	 * \param remember set to true if you want the policy to return the same marking in the same choice point
	 */
	explicit ExpPolicy(bool remember);

	/**
	 * \brief chooses the next marking among the available ones
	 *
	 * \param leaner the value function that may be used for the choice
	 * \param current the current marking, or <i>choice point </i>
	 * \param states a vector of possible next marking
	 */
	virtual int choose(Learner *learner,const Marking &current, const std::vector<Marking> &states);

	/**
	*\brief virtual dtor
	*/
	virtual ~ExpPolicy() {}

	/**
	 * A map from marking (choice point) to marking (choice made)
	 * to memorise previous choices.
	 */
	typedef std::map<Marking, Marking > MemoryType;

private:

	virtual int makeChoice(Learner *learner,const Marking &current, const std::vector<Marking> &states) = 0;

protected:

	MemoryType memory;

	bool remember;
};

}

#endif
