#include "condplan_translator.h"
#include "../pnpgenerator.h"
#include <sstream>

using namespace std;

class InLineTranslator : public CondPlan_Translator{
private:
//   string txt_file;
  void branch_r(string& line, vector<string>& branches);
  vector<string> tokenize(string s);
public:
  InLineTranslator(string& path_to_file);
  void generate_PNP(vector<string>& branches);
  void read_file();
  void write_pnml();
  void set_file(string& f);
  vector<ConditionalPlan> get_plan();
};
  