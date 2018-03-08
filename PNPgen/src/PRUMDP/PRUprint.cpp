// Implements all the printing of PRUs
#include "PRUMDP/PRUplus.h"

std::ostream& operator<<(std::ostream& os, const PRUoutcome& option) {
  os << option.name << " (" << (option.probability*100) <<"%): "
     << option.observable
     << "\n      Q=" << option.quality << "(" << option.qualityParameter << ")+"
     << option.qualityConstant
     << "\n      D=" << option.duration << "(" << option.durationParameter << ")+"
     << option.durationConstant;
  os << "\n      SVU{";
  for (vector<string>::const_iterator it = option.stateVariableUpdate.begin();
       it != option.stateVariableUpdate.end(); ++it) {
    os << "\n       " << *it;
  } // for *it in option.SVU
  os << "\n      }";
  os << "\n      NEXT{";
  for (vector<string>::const_iterator it = option.nextModules.begin();
       it != option.nextModules.end(); ++it) {
    os << "\n       " << *it;
  } // for *it in pru.firstEnabledModules
  os << "\n      }";
  if (option.isFinal)
    os << "\n      FINAL Label=" << option.finalLabel;
  return os;
}

std::ostream& operator<<(std::ostream& os, const PRUmodule& module) {
  os << "  MODULE " << module.actionName << " {\n";
  os << "   PARAM{\n";
  for (map<string, domain_type>::const_iterator it = module.parameters.begin();
       it != module.parameters.end(); ++it) {
    os << "    " << it->first << " in {" ;
    for (domain_type::const_iterator it2 = it->second.begin();
	 it2 != it->second.end(); ++it2) {
      os << " " << *it2 ;
    } // for *it2 in parameter *it 's domain
    os << "   }\n";
  } // for *it in module.parameters
  os << "   }\n";
  for (vector<PRUoutcome*>::const_iterator it = module.outcomes.begin();
       it != module.outcomes.end(); ++it) {
    os << "   > " << **it << "\n";
  } // for *it in module.outcomes
  os << "  }\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PRUlayer& layer) {
  os << " LAYER " << layer.name << " {\n";
  os << "  VARS{";
  for (vector<string>::const_iterator it = layer.stateVariables.begin();
       it != layer.stateVariables.end(); ++it) {
    os << " " << *it ;
  } // for *it in layer.stateVariables
  os << " }\n";
  for (vector<PRUmodule*>::const_iterator it = layer.modules.begin();
       it != layer.modules.end(); ++it) {
    os << **it;
  } // for *it in layer.modules
  os << " }\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const PRUplus& pru) {
  os << "PRU{\n INIT{\n";
  for (vector<string>::const_iterator it = pru.stateVariablesInitialAssignments.begin();
       it != pru.stateVariablesInitialAssignments.end(); ++it) {
    os << "  " << *it << "\n";
  } // for *it in pru.stateVariablesInitialAssignments
  os << " }\n NEXT{\n";
  for (vector<string>::const_iterator it = pru.firstEnabledModules.begin();
       it != pru.firstEnabledModules.end(); ++it) {
    os << "  " << *it << "\n";
  } // for *it in pru.firstEnabledModules
  os << " }\n";
  for (vector<PRUlayer*>::const_iterator it = pru.layers.begin();
       it != pru.layers.end(); ++it) {
    os << **it;
  } // for *it in pru.layers
  os << "}\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const domain_type vec) {
  os << "{ ";
  for (domain_type::const_iterator it = vec.begin(); it != vec.end(); ++it) {
    os << *it << ' ';
  }
  os << "}";
  return os;
}

std::ostream& operator<<(std::ostream& os, const map<string, domain_type> vars) {
  os << "Variables:\n";
  for (map<string, domain_type>::const_iterator it = vars.begin(); 
       it != vars.end(); ++it) {
    os << " - " << it->first << " in " << it->second << "\n";
  }
  return os;
}
