// Compile a PRU+ into a MDP
#ifndef __PRU2MDP__
#define __PRU2MDP__

#include "PRUplus.h"
#include "MDP.h"
#include "PRU2MDPprogress.h"
#include "PRU2MDPactionDescriptor.h"

class MDPaction;
class PRU2MDP;

/**
 * Builds a MDP from a PRUplus.
 */
class PRU2MDP {
 private:
  /** Builds Progresses from the specified PRULayer.
   * Each progress then builds instanciated actions.
   * Each action the builds resulting states.
   */
  void buildProgress(const PRUlayer *layer,
		     vector<string>::const_iterator itSV, 
		     map<string, string> &params);
 public:
  /** The initial state of the MDP */
  PRU2MDPstateStore states;
  MDPstate *init;
  vector<PRU2MDPprogress*> progress;
  map<string, domain_type> *stateVariableDomain; // domains are created but not deleted here
  /** All the MDP actions, indexed by a string layer.action,parameters.$state-variables
   * for example: 1.advertise,X=DoorE.$location=DoorE or 2.wait.
   */
  map<string, PRU2MDPactionDescriptor> allActions;

  /** Builds a MDP from a PRU */
  PRU2MDP (const PRUplus &pru);
  ~PRU2MDP();

  /** Matches specified modules with MDPactions and store them into s->availableActions */
  void matchActions(const vector<string> &nextModules, MDPstate *s) const;

  /** Fills the specified vector with all the MDP states */
  void getStates(vector<MDPstate*> &result) const;

  /** Builds the MDP. Will not be deleted here! */
  MDP *getMDP() const;
};

#endif
