#include "condplan_translator.h"

class PDDLTransl : public CondPlan_Translator{
private:
  ConditionalPlan make_state(string &state);

public:
  PDDLTransl(string &path_to_file);
  void read_file();
  void write_plan(string file_to_write);
};