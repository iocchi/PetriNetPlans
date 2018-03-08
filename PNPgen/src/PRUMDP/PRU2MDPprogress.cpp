// Compile a PRU+ into a MDP

#include "PRUMDP/PRU2MDPprogress.h"
#include <boost/algorithm/string.hpp>

#include "PRUMDP/PRU2MDP.h"

#undef PRINT
#include "PRUMDP/DEBUGprint.h"

void PRU2MDPprogress::registerActions(map<string, PRU2MDPactionDescriptor> &allActions) const {
  string prefix = lay->name + ".";
  string suffix = ".";
  for (PRUstate::const_iterator it = stateVariables.begin();
       it != stateVariables.end(); ++it) 
      suffix += '$' + it->first + '=' + it->second;
  for (vector<MDPaction*>::const_iterator itA = actions.begin();
       itA != actions.end(); ++itA) {
    MDPaction *act = *itA;
    string infix = act->actionName;
    for (PRUstate::const_iterator it=act->parameters.begin();
       it != act->parameters.end(); ++it)
      infix += ',' + it->first + '=' + it->second;
    allActions.insert(std::pair<string,PRU2MDPactionDescriptor>
		      (prefix+infix+suffix, PRU2MDPactionDescriptor(this, act)));
  } // for *itA in actions
} // registerActions(&allActions)

void PRU2MDPprogress::buildActions(const PRUmodule *mod, 
				   map<string,domain_type>::const_iterator iParam,
				   const MDPaction &action, 
				   map<string, domain_type> *SVdomain,
				   PRU2MDPstateStore &states) {
  if (iParam == mod->parameters.end()) {
    // All parameters are instanciated
    MDPaction *act = new MDPaction(action);
    actions.push_back(act);
    
    PRUstate res(stateVariables); // makes a local copy of SV that will be updated with out SVUs
    for (vector<PRUoutcome*>::const_iterator itO = mod->outcomes.begin();
	 itO != mod->outcomes.end(); ++itO) {
      PRUoutcome *out = *itO;
      for (vector<string>::const_iterator itSVU = out->stateVariableUpdate.begin();
	   itSVU != out->stateVariableUpdate.end(); ++itSVU) {
	vector<string> vec;
	boost::algorithm::split(vec, *itSVU, boost::algorithm::is_any_of(":= "), 
				boost::algorithm::token_compress_on );
	if (vec.size()!=2)
	  std::cerr << "Unreadable SVU : " << *itSVU << std::endl;
	else {
	  if (vec[1][0] == '$') {
	    // this is an action parameter
	    string p = vec[1].substr(1);
	    const string &v = act->getParameter(p);
	    if (v == MDPaction::NIL)
	      std::cerr << "Unknown action parameter "<<vec[1]<<std::endl;
	    else
	      res[vec[0]] = v;
	  } else {
	    // res[vec[0]] = vec[1]; // but needs a pointer (no local string !)
	    domain_type &dom = (*SVdomain)[vec[0]];
	    for (domain_type::const_iterator it = dom.begin();
		 it != dom.end(); ++it) {
	      if (*it == vec[1]) {
		res[vec[0]] = (*it);
		break;
	      }
	    } // for *it in this SV domain
	  } // if not an action parameter
	} // if correct SVU element
      } // for itSVU in *itO SV Updates
      // Here, res contains the updated state variables
      MDPstate *s = states.getState(lay->name, act, out, res);
      act->outcomes.insert(s);
    } // for *itO in mod->outcomes
  } else {
    // Some action parameter is to be instanciated yet
    string name = iParam->first;
    domain_type::const_iterator it = iParam->second.begin();
    domain_type::const_iterator itEnd = iParam->second.end();
    ++iParam;
    for (;it != itEnd; ++it) {
      MDPaction ma(action,name,*it);
      buildActions(mod, iParam, ma, SVdomain, states);
    } // for *it in the current parameter's domain
    --iParam;
  } // if more parameters
} // buildActions(*mod, iParam)

PRU2MDPprogress::PRU2MDPprogress(const PRUlayer *l, const PRUstate &sv,
				 map<string, domain_type> *stateVariableDomain,
				 PRU2MDPstateStore &states) 
  : stateVariables(sv) {
  lay = l;
  for (vector<PRUmodule*>::const_iterator iMod = lay->modules.begin(); 
       iMod != lay->modules.end(); ++iMod) {
    PRUmodule *mod = *iMod;
    MDPaction act(mod->actionName); // template for building all those actions
    buildActions(mod, mod->parameters.begin(), act, stateVariableDomain, states);
  } // for *iMod in lay->modules
} // PRU2MDPprogress(*l,&sv)

PRU2MDPprogress::~PRU2MDPprogress(){
  for (vector<MDPaction*>::iterator itA = actions.begin();
       itA != actions.end(); ++itA) {
    DEBUG("Temporary destruction of " << **itA << std::endl);
    delete (*itA);
  }
}

bool PRU2MDPprogress::isMatching(const PRUstate &stateVariables) const {
  for (PRUstate::const_iterator itV = this->stateVariables.begin();
       itV != this->stateVariables.end(); ++itV) {
    PRUstate::const_iterator itF = stateVariables.find(itV->first);
    if (itF == stateVariables.end()) {
      if (itV->second != "nil")
	return false;
    } else if (itF->second != itV->second)
      return false;
  }
  return true; // layer name matches and any specified variable does...
} // isMatching(...)

std::ostream& operator<<(std::ostream& os, const PRU2MDPprogress& progress) {
  os << "Layer " << progress.lay->name << " [";
  for (PRUstate::const_iterator itSV = progress.stateVariables.begin();
       itSV != progress.stateVariables.end(); ++ itSV) {
    os << ' ' << itSV->first << '=' << (itSV->second);
  } // for *itSV in stateVariables
  os << " ]" << std::endl;
  for (vector<MDPaction*>::const_iterator itA = progress.actions.begin();
       itA != progress.actions.end(); ++itA) {
    os << " > " << **itA << std::endl;
  }
  return os;
}

void PRU2MDPprogress::matchActions(const PRU2MDP *model) {
  for(vector<MDPaction*>::iterator itA = actions.begin();
      itA != actions.end(); ++itA) {
    MDPaction *act = *itA;
    for(set<MDPstate*>::iterator itO = act->outcomes.begin();
	itO != act->outcomes.end(); ++itO) {
      MDPstate *s = *itO;
      model->matchActions(s->prevOutcome->nextModules, s);
    } // for s=*itO in act->outcomes
  } // for act=*itA in actions
} // matchActions(*model)
