// Compile a PRU+ into a MDP
#ifndef __PRU2MDPprogress__
#define __PRU2MDPprogress__

#include "PRUplus.h"
#include "MDP.h"
#include "PRU2MDPactionDescriptor.h"
#include "PRU2MDPstateStore.h"

class PRU2MDP;

/**
 * Represents a PRU layer and the associated state variables.
 * Builds and stores the actions defined for this set of variables.
 */
class PRU2MDPprogress {
 private:
  void buildActions(const PRUmodule *mod, 
		    map<string,domain_type>::const_iterator iParam,
		    const MDPaction &act, map<string, domain_type> *SVdomain,
		    PRU2MDPstateStore &states);
 public:
  const PRUlayer *lay;
  const PRUstate stateVariables; // reuse strings from SV domains (PRU2MDP.stateVariableDomain)
  vector<MDPaction*> actions; // actions are created but not deleted here

  PRU2MDPprogress(const PRUlayer *l, const PRUstate &sv,
		  map<string, domain_type> *stateVariableDomain,
		  PRU2MDPstateStore &states);

  ~PRU2MDPprogress();

  /** Tests whether this progress has compatible state variables.
   * Specified state variables may include irrelevant ones.
   */
  bool isMatching(const PRUstate &stateVariables) const;

  /** Matches all PRU modules with MDPactions from all MDPstate */
  void matchActions(const PRU2MDP *model);

  /** Stores a descriptor of all the actions of this progress into the specified log */
  void registerActions(map<string, PRU2MDPactionDescriptor> &allActions) const;
};

std::ostream& operator<<(std::ostream& os, const PRU2MDPprogress& act);

#endif
