#include "condplan_translator.h"
#include "../pnpgenerator.h"
#include <map>

class DigraphTransl : public CondPlan_Translator {
private:
  string plan_name; //plan_#N of the digraph
//   map<string,string> state_action;
  void create_PNP(string& goal_name);
  void create_PNP(string& goal_name, vector<ConditionalPlan>& p);
  void write_pnml(vector<ConditionalPlan>& v);

public:
  DigraphTransl(string &path_to_file);
  DigraphTransl(string &path_to_file, string& pnml_out);
  void create_PNP();

  vector<ConditionalPlan> get_plan(){ 
    if(p.size() == 0 || p.size() == 1) return this->p; 
    else {
      cout << "Plan NOT defined" << endl;
      exit;
    }
  }
  string getPlanName() { return plan_name; }
};
