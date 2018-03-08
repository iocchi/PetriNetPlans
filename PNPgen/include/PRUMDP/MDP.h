// Represents a MDP for solving PRUs
#ifndef __MDP__
#define __MDP__

#include "PRUplus.h"

class MDPstate;

/** Represents a MDP action.
 * A MDP action is a PRUaction with instanciated parameters that can be used in a specific state of a the layer the PRUaction belongs to.
 */
class MDPaction {
 public:
  static std::string NIL;
  const string &actionName;
  PRUstate parameters; // DON'T reuse strings from actions domain (PRUmodule.parameters)
  set<MDPstate*> outcomes;

  MDPaction(const string &name);  
  MDPaction(const MDPaction &copy);

  /** Copies the specified action, changing the given parameter by the specified value.*/
  MDPaction(const MDPaction &copy, string param, const string &value);

  ~MDPaction();  
  
  string const &getParameter(string &key) const;

  friend std::ostream& operator<<(std::ostream& os, const MDPaction& act); // allows that method to access private fields 
};
std::ostream& operator<<(std::ostream& os, const MDPaction& act);

/** Represents a MDP state resulting from the outcome of an action (with specific parameters) in a layer (with specific variables). */
class MDPstate {
 public:
  const string name;
  /** The unique index of this state */
  int index; 
  set<const MDPaction *> availableActions; // references to PRU2MDPprogress.actions
  const MDPaction *prevAction; // may be null for initial state
  const PRUoutcome *prevOutcome; // may be null for initial state
  PRUstate stateVariables; // references to PRU2MDP.stateVariableDomain

  MDPstate(const string &description);
  MDPstate(const string &description, const MDPaction *act, const PRUoutcome *out,
	   const PRUstate &SV);

  /** Builds a string using variables as predicates separated by '&'. 
   */
  const string getPredicates() const;
  const string getPredicates(std::string stepPredicate) const;

  ~MDPstate(); 
};
std::ostream& operator<<(std::ostream& os, const MDPstate& state);

class MDP {
 private:
  vector<MDPstate*>    states;

  vector<float>            value[2]; // the expected value of each state
  vector<vector<const MDPaction*> > policy;   // the optimal action to use in each state

 public:
  ~MDP();

  MDP (vector<MDPstate*> &states);

  /** Prepares the MDP for computing Value Iteration */
  void initVI(); 
  /** Computes one iteration of VI.
   * @param gamma the discout factor.
   * @return the variation of the value function during this iteration.
   */
  float iterate(float gamma);

  /** Gets the policy for a given horizon
   * @param horizon the number of actions left, including this one
   * @return a reference to the requested policy */
  const vector<const MDPaction*> &getPolicy(int horizon) const;

  void printPolicy(std::ostream& os, int horizon) const;
  void printPolicy(std::ostream& os) const;

  const MDPstate *getState(int idx) const;
  const vector<const MDPstate *> &getStates() const;

  float getValue(int state) const;
}; // class MDP

#endif
