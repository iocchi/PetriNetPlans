#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;

struct Conditional{
  string true_case;
  string false_case;
  string node;
};

class DomainParser{
protected:
  vector<string> actions;
  vector<string> states;
  vector<Conditional> conditionals;
  string file_to_parse; //domain file
  
  static inline string &ltrim(string &s) {
    s.erase(s.begin(), find_if(s.begin(), s.end(),
            not1(ptr_fun<int, int>(isspace))));
    return s;
  }
  static inline string &rtrim(string &s){
      s.erase(find_if(s.rbegin(), s.rend(),
              not1(ptr_fun<int, int>(isspace))).base(), s.end());
      return s;
  }
  static inline string &trim(string &s) {
      return ltrim(rtrim(s));
  }
  
  int count_substr(const std::string& str, const std::string& sub){
    if (sub.length() == 0) return 0;
    int count = 0;
    for (size_t offset = str.find(sub); offset != std::string::npos;
	 offset = str.find(sub, offset + sub.length())){
        ++count;
    }
    return count;
  }
  
  void set_file(string& file){ this->file_to_parse = file; }
  
public:
  DomainParser(){}
  DomainParser(string& file){ this->file_to_parse = file; }
  bool find_conditionals();
  bool parse_actions();
  bool parse_states();
  vector<string> get_actions(){ return this->actions; }
  vector<string> get_states(){ return this->states; }
  vector<Conditional> get_conditionals(){ return this->conditionals; }
  
};