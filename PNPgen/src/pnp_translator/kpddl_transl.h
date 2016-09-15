#include "condplan_translator.h"
#include <stack>
#include <algorithm>

class KPDDLTransl : public CondPlan_Translator{
private:
  vector<string> plan;
  
  string find_state(int num);
  ConditionalPlan make_state(string &state);

public:
  KPDDLTransl(string &path_to_file);
  void read_file();

};
