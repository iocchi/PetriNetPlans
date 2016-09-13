#include "condplan_translator.h"

class RDDLTransl : public CondPlan_Translator{
private:

public:

  void read_file(){
    string line, temp;
    vector<string> plan;
    ifstream solution(this->file.c_str());

    while(solution >> line && !solution.eof()){
    }
  }

};
