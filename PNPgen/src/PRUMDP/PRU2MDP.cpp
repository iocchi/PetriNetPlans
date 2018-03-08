// Compiles a PRU+ into a MDP

#include "PRUMDP/PRU2MDP.h"
#include <boost/algorithm/string.hpp>
#include <pcrecpp.h>

#undef PRINT
#include "PRUMDP/DEBUGprint.h"

void PRU2MDP::buildProgress(const PRUlayer *layer,
			    vector<string>::const_iterator itSV, 
			    map<string, string> &params){
  if (itSV == layer->stateVariables.end()) {
    // All variables are instanciated
    progress.push_back(new PRU2MDPprogress(layer,params, stateVariableDomain, states));
    progress.back()->registerActions(allActions);
  } else {
    // Some state variable is to be instanciated yet
    string var = *itSV;
    map<string, domain_type>::const_iterator itV = stateVariableDomain->find(var);
    if (itV == stateVariableDomain->end())
      std::cerr << "Unknown state variable " << var << std::endl;
    else {
      ++itSV;
      for (domain_type::const_iterator itD = itV->second.begin();
	   itD != itV->second.end(); ++itD) {
	params[var] = *itD;
	buildProgress(layer,itSV,params);
      } // for itD in variable domain
      --itSV;
      params.erase(var);
    } // if known state variable
  } // if some state variable left
} // buildProgress(*layer)

/** Inserts Actions listed in nextModules into s->availableActions **/
void PRU2MDP::matchActions(const vector<string> &nextModules, MDPstate *s) const {
  for(vector<string>::const_iterator itM = nextModules.begin();
      itM != nextModules.end(); ++itM) {
    string module = *itM;
    pcrecpp::RE expression("(^|\b)"+module+"([.,$]|$)"); // adds the word-boundary conditions
    if (module[0]=='-')
      expression = pcrecpp::RE("(^|\b)"+module.substr(1)+"([.,$]|$)");
    DEBUG("Matching " << module << std::endl);
    bool err = true;
    for (map<string, PRU2MDPactionDescriptor>::const_iterator it = allActions.begin();
	 it != allActions.end(); ++it) {
      if (it->second.progress->isMatching(s->stateVariables))
	if (expression.PartialMatch(it->first)) {
	  err = false;
	  if (module[0]=='-') {
    DEBUG(" < " << it->first << std::endl);
	    s->availableActions.erase(it->second.action);
	  } else {
    DEBUG(" > " << it->first << std::endl);
	    s->availableActions.insert(it->second.action);
	  }
	} // if match
    } // for *it in allActions
    if (err)
      std::cerr << " *** " << module << " matches no module!" << std::endl;
  } // for module=*itM in state.prevOutcome->nextModules
} // matchActions(&nextModules, *s)

PRU2MDP::~PRU2MDP() {
    DEBUG("Destroying PRU2MDP...\n");
  for (vector<PRU2MDPprogress*>::iterator it=progress.begin();
       it != progress.end(); ++it) {
    delete (*it);
  }
  delete init; // TEMPORARY
  delete stateVariableDomain; // TEMPORARY
}

PRU2MDP::PRU2MDP(const PRUplus &pru) {
  // First compute State Variables Domain
  stateVariableDomain = new map<string, domain_type>();
  pru.fillSVdomain(*stateVariableDomain);

  // Next builds progress (along with actions and states)
  for (vector<PRUlayer*>::const_iterator iLay = pru.layers.begin(); 
       iLay != pru.layers.end(); ++iLay) {
    PRUlayer *lay = *iLay;  
    map<string, string> params;
    buildProgress(lay, lay->stateVariables.begin(), params);
  } // for *iLay in pru.layers

#ifdef PRINT
  for (map<string,PRU2MDPactionDescriptor>::const_iterator it=allActions.begin();
       it != allActions.end(); ++it) {
    std::cout << it->first << std::endl;
  }
#endif

  // Matches first-modules with MDPaction in initial state
  init = new MDPstate("Init");
  for (vector<string>::const_iterator itSV = pru.stateVariablesInitialAssignments.begin();
       itSV != pru.stateVariablesInitialAssignments.end(); ++itSV) {
    vector<string> vec;
    boost::algorithm::split(vec, *itSV, boost::algorithm::is_any_of(":= "), 
			    boost::algorithm::token_compress_on );
    if (vec.size()!=2)
      std::cerr << "Unreadable initial SV : " << *itSV << std::endl;
    else {
      // res[vec[0]] = vec[1]; // but needs a pointer (no local string !)
      domain_type &dom = (*stateVariableDomain)[vec[0]];
      for (domain_type::const_iterator it = dom.begin();
	   it != dom.end(); ++it) {
	if (*it == vec[1]) {
	  init->stateVariables[vec[0]] = (*it);
	  break;
	}
      } // for *it in this SV domain
    } // if correct SVU element
  } // for itSV in pru.firstEnabledModules
  matchActions(pru.firstEnabledModules, init);

  // Finally matches next-module with MDPaction in each state
  for (vector<PRU2MDPprogress*>::iterator itP=progress.begin();
       itP != progress.end(); ++itP) {
    (*itP)->matchActions(this);
  } // for *itP in progress
} // PRU2MDP(&pru)

void PRU2MDP::getStates(vector<MDPstate*> &result) const {
  result.push_back(init);
  states.getStates(result);
}

MDP *PRU2MDP::getMDP() const {
  vector<MDPstate*> states;
  getStates(states);
  return new MDP(states);
}
