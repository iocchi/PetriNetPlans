#pragma once

#include "condplan_translator.h"

class CFFTransl : public CondPlan_Translator{
private:
  void parseline(string &line, set<string> &states, map<string,string> &state_action, map<string,string> &state_outcome);

public:
  CFFTransl(string &path_to_file);
  void read_file();
  void write_plan(string file_to_write);
};

