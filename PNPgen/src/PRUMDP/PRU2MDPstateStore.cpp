// Stores states so that equivalent states are not duplicated
// a state is stored as <layer>.<latestAction>.<latestOutcome>$<var1>=<value1>$<var2>=<value2>$...
#include "PRUMDP/PRU2MDPstateStore.h"

MDPstate *PRU2MDPstateStore::getState(const string &layerName, const MDPaction *act, const PRUoutcome *out, const PRUstate &res) {
  string name = layerName;
  if (act != NULL)
    name += '.' + act->actionName + '.' + out->name;
  for (PRUstate::const_iterator it=res.begin(); it!=res.end(); ++it) {
    name += '$' + it->first + '=' + (it->second);
  }
  map<string,MDPstate*>::iterator exist = store.find(name);
  if (exist == store.end()) {
    // new state
    MDPstate *s = new MDPstate(layerName,act,out,res);
    store.insert(std::pair<string,MDPstate*>(name,s));
    return s;
  } else {
    // existing state
    return exist->second;
  }
}

void PRU2MDPstateStore::getStates(vector<MDPstate*> &result) const {
  for (map<string,MDPstate*>::const_iterator it=store.begin();
       it!=store.end(); ++it) {
    result.push_back(it->second);
  }
}

PRU2MDPstateStore::~PRU2MDPstateStore() {
  for (map<string,MDPstate*>::iterator it=store.begin();
       it!=store.end(); ++it)
    delete(it->second);
}

void PRU2MDPstateStore::print() const {
  for (map<string,MDPstate*>::const_iterator it=store.begin();
       it!=store.end(); ++it) {
    std::cout << it->first << std::endl;
  }
}

