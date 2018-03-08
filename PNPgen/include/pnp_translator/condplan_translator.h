#ifndef __CONDITONALPLAN_TRANSLATOR_H_INCLUDED__
#define __CONDITONALPLAN_TRANSLATOR_H_INCLUDED__

#include "../conditionalplan.h"
#include <fstream>
#include <string>
#include <boost/regex.hpp>

using namespace std;

class CondPlan_Translator{
protected:
  vector<ConditionalPlan> p;
  string file; //plan file
  string pnml; //pnml file

  ConditionalPlan make_state(const string &s);

  /*** STRING UTILITIES ***/
  // trim from start
  static inline string &ltrim(string &s) {
    s.erase(s.begin(), find_if(s.begin(), s.end(),
            not1(ptr_fun<int, int>(isspace))));
    return s;
  }

  // trim from end
  static inline string &rtrim(string &s){
      s.erase(find_if(s.rbegin(), s.rend(),
              not1(ptr_fun<int, int>(isspace))).base(), s.end());
      return s;
  }

  // trim from both ends
  static inline string &trim(string &s) {
      return ltrim(rtrim(s));
  }

  // trim the string sub from s
  static inline string &trim_substr(string &s, string &sub){
    string::size_type i = s.find(sub);
    if( i != string::npos)
      s.erase(i,sub.length());
  }

  string to_lowercase(const string& str) {
    stringstream ss;
    int differ = 'A'-'a';
    int str_size = str.size();
    for(int i=0; i<str_size;i++){
      char ch = str.at(i);
      if(ch>='A' && ch<='Z')
	ch = ch-differ;
      ss << ch;
    }
    return ss.str();
  }

  void set_file(string& f){ this->file = f; }
  void set_pnml(string& pn){ this->pnml = pn;}

public:
  CondPlan_Translator() { }
  ConditionalPlan* read_file() { cerr << "ConditionalPlan::read_file() NOT IMPLEMENTED!!!" << endl; }
  // void write_plan(string out = "test/translator_out.txt");

  void write_plan(string file_to_write = "test/translator_out.txt"){
    ofstream out;
    out.open(file_to_write.c_str());

    out << "plan";
    out <<  string("{ \n\n");

    out << " n_states=";
    out << this->p.size();
    out << "\n\n";

    for(int i = 0; i < this->p.size();i++){
        out << " ";
        out << i;
        out << "[";
        out << "label=";
        out << p[i].state;
// 	if(!p[i].action.empty()){
	  out << ",actions=";
	  out << p[i].action;
// 	}
        out << "]\n";
    }
    out << "\n";

    for(int i = 0; i < this->p.size();i++){
	if(p[i].outcomes.size() == 0){
// 	  out << "\n";
// 	  cout << p[i].state << endl;
	  out << "\n";
// 	  break;
	}

	if(p[i].outcomes.size() == 1){
	    out << " \"";
	    out << p[i].state;
	    out << "\"";
	    out << " -> ";
	    out << "\"";
	    out << p[i].outcomes[0].successor->state;
	    out << "\"\n";

	}

	if(p[i].outcomes.size() > 1 ){
	  vector<ActionOutcome>::iterator it = p[i].outcomes.begin();
	  while(it != p[i].outcomes.end()){
	    string obs = (*it).observation;
	    out << " \"";
	    out << p[i].state;
	    out << "\" ";
	    out << "[";
	    out << obs;
	    out << "]";
	    out << " -> ";
	    out << "\"";
	    out << (*it).successor->state;
	    out << "\"";
	    ++it;
	    if(it != p[i].outcomes.end()) out << " ;";
	  }
	  out << "\n";
	}
    }

    out << "}";
    out.close();

  }

  void setCondPlan(vector<ConditionalPlan> cp){ this->p = cp; }
  vector<ConditionalPlan> getCondPlan(){ return this->p; }
};


#endif
