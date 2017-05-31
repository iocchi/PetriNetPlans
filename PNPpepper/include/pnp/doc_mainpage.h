/**
\mainpage Petri Net Plans

The Petri Net Plan (PNP) formalism allows for high level description of
complex action interactions. Such interactions are necessary in programming cognitive
agents with real-time requirements, like mobile robots or video game AIs.
PNPs are inspired to languages for reasoning about actions. Nonetheless, they are more
expressive than most of them, offering a full fledged set of operators 
for dealing with non-instantaneous actions, sensing actions, action
failures, concurrent actions and cooperation in a multi-agent context.

For an introduction to PNP please refer to http://pnp.dis.uniroma1.it

\section who Authors and Maintainers

PNP has been initially developed by Vittorio A. Ziparo ( http://www.dis.uniroma1.it/~ziparo/ )

It has then been improved and maintained for a while by Daniele Calisi ( http://www.dis.uniroma1.it/~calisi/ )

It is currently in the clutches of Matteo Leonetti ( http://www.dis.uniroma1.it/~leonetti/ ) who has
also developed the learning part. Please report to him any error in this documentation.

\section usage General Usage

You want to follow, more or less, this list:

-# Write a plan and save it in the format you prefer. We suggest the Petri 
   Net Markup Language ( http://www.informatik.hu-berlin.de/top/pnml/ )because we developed
   a plan loader for it. Otherwise you would have to implement one yourself. 
   Our favourite tools to do so are Jarp ( http://jarp.sourceforge.net/ ) and Pipe
   ( http://pipe2.sourceforge.net/ ).
-# At this point you should have clarified to yourself what actions you need. Implement 
   those actions sub-classing for each of them PetriNetPlans::PnpAction and overriding the
   methods you want (typically among PetriNetPlans::PnpAction::start(), 
   PetriNetPlans::PnpAction::executeStep(), PetriNetPlans::PnpAction::end())
   Pay attention to the termination condition, if you want it to be internal (triggered by the action rather than the plan)
   you must also override PetriNetPlans::PnpAction::finished().
-# Make sure you have the interface between the agent and the environment
   ready. This can be as complicated as a Knowledge Base updated according
   to the agent perceptions, or as simple as a synthetic representation
   of the environment itself (e.g. the board in tic-tac-toe).
-# Subclass PetriNetPlans::ExternalConditionChecker and implement PetriNetPlans::ExternalConditionChecker::evaluateAtomicExternalCondition()
   to test the conditions you used in the plan(s) on your interface to the environment.
   This class is the bridge between PNP and the outside world. For each <i>atomic</i>
   condition (or <i>predicate</i>) it must return whether or not it holds according
   to the agent's knowledge at the time it is invoked.
-# Subclass PetriNetPlans::ExecutableInstantiator and have it create your plans and actions. PNP does not
   distinguish between them, so they are all returned as PetriNetPlans::PnpExecutable instances.
   This object must know where the plans are stored and what parameters (if any) provide
   to the actions. None of these things are enforced by PNP. You can implement this
   class in many different ways and in complicated systems it might require some
   ingenuity. In order to load plans
   from pnml files, you can use PetriNetPlans::XMLPnpPlanInstantiator. Create an empty plan and pass it
   to PetriNetPlans::XMLPnpPlanInstantiator::loadFromPNML(). This object also commonly passes itself to the newly created plan
   to be its instantiator too.
-# Finally instantiate PetriNetPlans::PnpExecuter, and set the name of the main plan with 
   PetriNetPlans::PnpExecuter::setMainPlan(). Repeatedly call PetriNetPlans::PnpExecuter::execMainPlanStep() at the frequency you need
   (this depends on how often you want your agent to make decisions, that usually
   depends on how fast the environment changes).
   
This list is in the order of the dependencies but I suggest to write the plan first and then
follow the other steps in reverse order, so that you'll clearly see when and where you
need every component before actually implementing it.

\section learning PNP and Reinforcement Learning

A tutorial about learning in PNP is available at http://www.dis.uniroma1.it/~leonetti/index.php?option=com_content&task=view&id=56&Itemid=54

To use the learning capability you must follow the same steps as for normal PNP with
a few differences:

- Instead of PetriNetPlans::ExternalConditionChecker you must subclass learnpnp::RewardCollector. It IS-A
  ExternalConditionChecker but in addition has a function to communicate to PNP
  the reward obtained between two subsequent calls of learnpnp::RewardCollector::reward().
  Since both accumulating the reward and testing conditions need access to the agent's
  knowledge about the environment they are implemented by the same component.
- The executable instantiator (the subclass of PetriNetPlans::ExecutableInstantiator) must return
  through the method PetriNetPlans::ExecutableInstantiator::createExecutable() an instance of learnpnp::LearnPlan instead of PetriNetPlans::PnpPlan.
- When more than one next marking is possible, that is in non-deterministic choice points,
the control is taken over by the learnpnp::Controller provided to the plan.

\subsection controllers A word about controllers

Controllers are in charge of learning and making decisions in non-deterministic choice
points.  A few default implementations are provided for ease of use.
A learnpnp::BasicController delegates learning to a learnpnp::Learner, and to a and to
a learnpnp::ExpPolicy. The former manages
the value function and implements an update rule. The latter chooses
one of the possible next markings, given their values. Two common choices are learnpnp::TDLambda
for the learner and learnpnp::EGreedy for the exploration policy.

Controllers are available in the directory pnp/learning_plan/algo/ and exploration
strategies in pnp/learning_plan/exp/. Both have generic interfaces that can be
extended to implement one's particular needs.

Worth mentioning is also that learnpnp::BasicController  is a learnpnp::LoggingController, which means
it is also able to log onto a file the values of the specified markings at the end of each
episode. This allows to monitor how the learning is going at specific choice points.
**/

