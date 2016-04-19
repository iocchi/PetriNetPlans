// Stores states so that equivalent states are not duplicated

#include "PRUplus.h"
#include "MDP.h"

/** Stores states so that equivalent states are not duplicated */
class PRU2MDPstateStore {
 private:
  map<string,MDPstate*> store;
 public:
  /** Gets a state corresponding to the specified parameters or creates it if necessary. */
  MDPstate *getState(const string &layerName, const MDPaction *act, const PRUoutcome *out, const PRUstate &res);

  /** Fills the specified vector with all the MDP states */
  void getStates(vector<MDPstate*> &result) const;

  ~PRU2MDPstateStore();

  void print() const;
};
