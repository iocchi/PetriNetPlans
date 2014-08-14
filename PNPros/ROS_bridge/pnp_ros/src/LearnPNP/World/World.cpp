#include <pnp_ros/LearnPNP/World/World.h>

using namespace std;

namespace pnpros
{
	namespace LearnPNP
	{
		World::World() : totalReward(0), epoch(0)
		{
			// Initialize variables.
			endEpoch();
		}
		
		World::~World() {;}
		
		World* World::w = new World();
		int World::learningPeriod;
		int World::samples;
		
		void World::endEpisode()
		{
			if (learning) actions += actionsLearning;
			
			if (learning && (lastPeriod == learningPeriod))
			{
				learning = false;
				lastPeriod = 1;
			}
			else if (!learning && (lastPeriod == samples))
			{
				learning = true;
				lastPeriod = 1;
				++episode;
			}
			else
			{
				++lastPeriod;
				
				if (learning) ++episode;
			}
			
			actions = 0;
			actionsLearning = 0;
			totalReward = 0;
		}
		
		void World::endEpoch()
		{
			learning = false;		// False because we start not learning, and put this at one at the begininng of the learning period.
			lastPeriod = 1;
			episode = 0;
			actions = 0;
			actionsLearning = 0;
			totalReward = 0;
			++epoch;
		}
		
		int World::totalActions() const
		{
			int value = actions;
			
			if (learning) value += actionsLearning;
			
			return value;
		}
	}
}
