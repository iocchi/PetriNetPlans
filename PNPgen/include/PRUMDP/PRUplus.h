// Stores a PRU+ file in memory for later use
#ifndef __PRUplus__
#define __PRUplus__

#include <string>
#include <vector>
#include <map>
#include <set>

#include <iostream>

using std::string;
using std::map;
using std::vector;
using std::set;

typedef set<string> domain_type;
typedef map<string, string> PRUstate; // assignment of state variables

/** Utility function for printing a domain */
std::ostream& operator<<(std::ostream& os, const domain_type vec);
/** Utility function for printing a set or variables */
std::ostream& operator<<(std::ostream& os, const map<string, domain_type> vars);


class PRUmodule;
// Defines class xmlpp::TextReader so that libxml++ is necessary only for linking
namespace xmlpp {
  class TextReader;
}

/* The PRU function profile */
typedef float (*PRUfunction)(const PRUstate& fromState,
			  const PRUstate& toState,
			  const PRUstate& actionParam,
			  const string& kind,
			  float parameter);

/** Represents one of the outcomes of a PRUmodule.
 *  Any outcome is defined by a name that must be unique for a given module.
 */
class PRUoutcome {
 private:
  void initDefaultValues();

 public:
  string name;
  /** The estimated probability of this outcome. 
   * Must sum to 100% among all the outcomes of any module */
  float probability;

  /** Logic formula true iff this outcome is valid */
  string observable; 

  /** A function descriptor allowing for the computation of the expected quality of this outcome */
  string quality;
  float qualityParameter;
  float qualityConstant;

  /** A function descriptor allowing for the computation of the expected duration of this outcome */
  string duration;
  float durationParameter;
  float durationConstant;

  /** A list of updates for state variables.
   *  Each entry should be in the form X=V
   */
  vector<string> stateVariableUpdate;
  /** A list of modules that can be used after this outcome.
   *  Each entry should be a full-word regular expression.
   *  For example, advertise will match 1.advertise and 2.advertise but not 1.advertiseComplex
   */
  vector<string> nextModules;

  /** true iff this outcome completes the PRU (dead-end) */
  bool isFinal;
  /** empty or the label of the outcome of this PRU+ */
  string finalLabel; 

  PRUoutcome();
  PRUoutcome(xmlpp::TextReader &reader);
  ~PRUoutcome();
  /** Fills the specified domain with values from the SVUs of this outcome.
   *  Needs the action-parameters domain in case SVU depends on them. */
  void fillSVdomain (map<string, domain_type> &stateVariableDomain,
		     const map<string, domain_type> &parameters) const;

  /** Computes the reward for arriving in dest when coming from orig */
  float getQuality(const PRUstate &orig, const PRUstate &dest, const PRUstate &act) const;

  /** Computes the reward for arriving in dest when coming from orig */
  float getDuration(const PRUstate &orig, const PRUstate &dest, const PRUstate &act) const;

};

std::ostream& operator<<(std::ostream& os, const PRUoutcome& option);

/** Represents a module of a PRUlayer.
 *  A module is characterized by an actionName that must be unique in its layer.
 */
class PRUmodule {
 public:
  /** The action name. Will be used as an output of the MDP. */
  string actionName;
  /** The action parameters. Allow for action templates.
   *  For exemple, goto(X) with X={a,b,c}
   */
  map<string, domain_type> parameters;
  map<string, string> domains;

  /** The list of possible outcomes of this module */
  vector<PRUoutcome*> outcomes;

  PRUmodule() { }
  PRUmodule(xmlpp::TextReader &reader);
  ~PRUmodule();

  /** Fills the specified domain with values from the SVUs of all the outcomes of this module. */
  void fillSVdomain (map<string, domain_type> &stateVariableDomain) const;

  /** Updates the parameters with new domain values */
  void updateDomains();
};

std::ostream& operator<<(std::ostream& os, const PRUmodule& module);

/** Represents a layer from a PRUplus.
 *  A layer is characterized by a name that must be unique in each PRU.
 */
class PRUlayer {
 public:
  string name;
  /** A list of state variables that must be remembered all through this layer. */
  vector<string> stateVariables;
  /** The list of PRUmodules that can be used here */
  vector<PRUmodule*> modules;

  PRUlayer() { }
  PRUlayer(xmlpp::TextReader &reader);
  ~PRUlayer();

  /** Fills the specified domain with values from the SVUs of all the modules of this layer. */
  void fillSVdomain (map<string, domain_type> &stateVariableDomain) const;
  /** Updates the parameters with new domain values */
  void updateDomains();
};

std::ostream& operator<<(std::ostream& os, const PRUlayer& layer);


/** Converts a constrint part into a vector of strings to check for equality */
class PRUon;
/** Stores a PRU constraint */
class PRUconstraint {
 private:
  void readXML(xmlpp::TextReader &reader);
  /** Stores each matching tuple along with the number of times it has been matched */
  map<vector<string>,int> matched;
  int validIfNoLessThan,validIfEachNoLessThan,validIfAnyNoLessThan;
  int validIfNoMoreThan,validIfEachNoMoreThan,validIfAnyNoMoreThan;
 public:
  string name;
  float score;
  /** The elements this constraint evaluates. This is valid if all elements are valid */
  vector<const PRUon*> elements;
  PRUconstraint(xmlpp::TextReader &reader);
  ~PRUconstraint();
  //--- Policy-evaluation ---
  /** Reset the constraint, readying for a new evaluation */
  void reset();
  /** Updates the constraint, matching current action and state or not */
  void match(string &actionId, PRUstate &actionParam, PRUstate &stateVariables);
  /** Returns the score if the constraint is satisfied, 0 otherwise */
  float getScore() const;
};


/** Represents a PRU+. */
class PRUplus {
 private:
  void readXML(xmlpp::TextReader &reader);
 public:
  /** Stores the domain of any action parameter that can be specified outside of the XML file.*/
  static map<string, domain_type> actionsDomain;
  /** Stores the domain-dependant function used for computing reward and distances */
  static PRUfunction domainFunction;

  /** An identifier for any use */
  string id;

  /** Initial values of state variables. May be forgotten depending on layers... */
  vector<string> stateVariablesInitialAssignments;
  /** Available modules for starting the PRU. Usually first_layer.*
   *  Each entry should be a full-word regular expression.
   *  For example, advertise will match 1.advertise and 2.advertise but not 1.advertiseComplex
   */
  vector<string> firstEnabledModules;
  vector<PRUlayer*> layers;
  /** A list of valuated global-constraints to assess the plan quality */
  vector<PRUconstraint*> constraints;

  PRUplus();
  PRUplus(string xmlFileName);
  bool readXML(string xmlFileName);
  ~PRUplus() ;
  /** Fills the specified domain with values from the SVUs of all the modules of all the layers of this PRU. */  
  void fillSVdomain (map<string, domain_type> &stateVariableDomain) const;
  /** Updates the parameters with new domain values */
  void updateDomains();
};

std::ostream& operator<<(std::ostream& os, const PRUplus& pru);

#endif
