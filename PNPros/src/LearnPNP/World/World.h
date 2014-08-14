#ifndef WORLD_H
#define WORLD_H

#include <pnp/pnpfwd.h>
#include <string>

/**
 * \defgroup base Base Classes
 *
 * Classes for the general set up, must be extended to create a specific problem
 * instance.
 */ 

/**
 * \brief A collection of properties about the particular domain, and the state of the execution
 *
 * The methods of this class are split into two categories: property methods and
 * state methods.
 *
 * Property methods specify the name of the main plan, the directories in which the
 * plans can be found on a specific problem instance and so on.
 *
 * State methods return information about the current state of the execution.
 * The experiment are run in alternating cycles of learning and non-learning
 * episodes. The agent starts not learning, and executes some episodes in which
 * the initial policy is evaluated. Then it starts learning, and during this period it is
 * allowed to change the value function and modify its behaviour. At some point
 * learning is paused and the current policy is evaluated again. Then learning is
 * restarted, and so on until the end of the epoch. The length of those periods is
 * specified in the configuration file parameters.h
 *
 * A global instance of this class is pointed by World::w so that the rest of the
 * application can access this information.
 *
 * \ingroup base
 *
 * \author Matteo Leonetti
 */
namespace pnpros
{
	namespace LearnPNP
	{
		struct World
		{
			private:
				double totalReward;
				int actions, actionsLearning, epoch, lastPeriod;
				int episode;										// Doesn't take into account sample episodes
				bool learning;
				
			public:
				static int learningPeriod, samples;
				
				static World* w;
				
				explicit World();
				
				virtual ~World();
				
				int currentEpisode() const { return episode; }
				
				int currentEpoch() const { return epoch; }
				
				void endEpisode();
				
				void endEpoch();
				
				double getReward() const { return totalReward; }
				
				void increaseActionValue()
				{
					learning ? ++actionsLearning : ++actions;
				}
				
				bool isLearning() const { return learning; }
				
				void sumReward(double value) { totalReward += value; }
				
				int totalActions() const;
		};
	}
}

#endif
