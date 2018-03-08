// Represents a MDP for solving PRUs

#include "PRUMDP/MDP.h"
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#undef PRINT
#include "PRUMDP/DEBUGprint.h"

std::string MDPaction::NIL("NIL");

MDPaction::MDPaction(const string &name): actionName(name) {
  // Nothing more to do
};
MDPaction::MDPaction(const MDPaction &copy): 
  actionName(copy.actionName), parameters(copy.parameters) {
  for (set<MDPstate*>::const_iterator it=copy.outcomes.begin();
       it != copy.outcomes.end(); ++it) {
    outcomes.insert(new MDPstate((*it)->name, this, (*it)->prevOutcome,(*it)->stateVariables));
  }
};

MDPaction::MDPaction(const MDPaction &copy, string param, const string &value): 
  actionName(copy.actionName) ,parameters(copy.parameters) {
  parameters[param] = value;
  for (set<MDPstate*>::const_iterator it=copy.outcomes.begin();
       it != copy.outcomes.end(); ++it) {
    outcomes.insert(new MDPstate((*it)->name, this, (*it)->prevOutcome,(*it)->stateVariables));
  }
}
string const &MDPaction::getParameter(string &key) const {
    PRUstate::const_iterator it = parameters.find(key);
    if (it == parameters.end())
      return NIL;
    else
      return it->second;
}
MDPaction::~MDPaction() {
  /* Don't delete states, they are managed by the store
  for (set<MDPstate*>::iterator it = outcomes.begin();
       it != outcomes.end(); ++it)
    delete (*it);
  */
}

std::ostream& operator<<(std::ostream& os, const MDPaction& act) {
  os << act.actionName;
  for (PRUstate::const_iterator it = act.parameters.begin();
	 it != act.parameters.end(); ++it) {
    os << ' ' << it->first << "=" << it->second;
  }
  return os;
}

MDPstate::MDPstate(const string &description) : name(description) { 
  prevAction = NULL;
  prevOutcome = NULL;
}
MDPstate::MDPstate(const string &description, const MDPaction *act, 
		   const PRUoutcome *out, const PRUstate &SV) : 
  name(description), stateVariables(SV) {
  prevAction = act;
  prevOutcome = out;
}
const string MDPstate::getPredicates() const {
  string prefix = "_context("+name;
  string res = ")"; // closing context
  for (PRUstate::const_iterator it=stateVariables.begin();
       it != stateVariables.end(); ++it) {
    if (it->second != "nil") // nil variables are not specified
      res += " & " + it->first+"("+it->second+")";
  }
  if (prevAction != NULL) {
    prefix += ","+prevAction->actionName+","+prevOutcome->name;
    if (prevOutcome->isFinal)
      res += " & _goal('" + prevOutcome->finalLabel + "')";
  }

  return prefix + res;
}
const string MDPstate::getPredicates(std::string stepPredicate) const {
  string prefix = "_context("+name;
  string res = ")"; // closing context
  for (PRUstate::const_iterator it=stateVariables.begin();
       it != stateVariables.end(); ++it) {
    if (it->second != "nil") // nil variables are not specified
      res += " & " + it->first+"("+it->second+")";
  }
  if (prevAction != NULL) {
    prefix += ","+prevAction->actionName+","+prevOutcome->name;
    if (prevOutcome->isFinal)
      res += " & _goal('" + prevOutcome->finalLabel + "')";
    else
      res += stepPredicate;
  }

  return prefix + res;
}

MDPstate::~MDPstate() {
#ifdef PRINT
  std::cout << "Destroying state " << name;
  if (prevAction != NULL)
    std::cout << ".(action " << prevAction->actionName
	      << " -> " << prevOutcome->name << ")";
  std::cout << std::endl;
#endif
}

std::ostream& operator<<(std::ostream& os, const MDPstate& state) {
  os << state.name;
  if (state.prevAction != NULL) {
    os << '.' << state.prevAction->actionName
       << '.' << state.prevOutcome->name;
    for (PRUstate::const_iterator itSV = state.stateVariables.begin();
	 itSV != state.stateVariables.end(); ++itSV) {
      os << '$' << itSV->first << '=' << itSV->second;
    } // for *itSV in stateVariables
  }
  return os;
};


MDP::~MDP() {
};

MDP::MDP(vector<MDPstate*> &_states) : states(_states) {
};

const vector<const MDPaction*> &MDP::getPolicy(int horizon) const {
  return policy[horizon];
}

void MDP::printPolicy(std::ostream& os) const {
  printPolicy(os, policy.size()-1);
}

void MDP::printPolicy(std::ostream& os, int horizon) const {
  const vector<const MDPaction*> &p = policy[horizon];
  for (vector<MDPstate*>::const_iterator itS = states.begin();
       itS != states.end(); ++itS) {
    MDPstate *s = *itS;
    const MDPaction *a = p[s->index];
    if (a!=NULL)
      os << *s << ": " << *a;
    else
      os << *s << ": ***No Action***";
    if (horizon >= ((int)policy.size())-2) {
      os << " (" << value[(horizon+1) & 1][s->index] << ')';
    }
    os << std::endl;
  } // for *itS in states
}

const MDPstate *MDP::getState(int idx) const {
  return states[idx];
}

const vector<const MDPstate *> &MDP::getStates() const {
  return (const vector<const MDPstate *>&) states;
}

