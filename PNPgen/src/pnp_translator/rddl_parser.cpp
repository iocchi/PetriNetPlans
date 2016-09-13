#include "domain_parser.h"
#include <map>

class RDDLParser : public DomainParser{
private:
  Conditional make_conditional(string ifline){
    Conditional cond;
    string tcase, fcase, node;
    
    if(ifline.find("if") != string::npos && ifline.find("then") != string::npos
      && ifline.find("else") != string::npos){
    //state block
    node = ifline.substr(ifline.find("(")+1,ifline.find(")") - ifline.find("(") - 1);
    cond.node = node;
    cout << "node " << node << endl;
	
    //then block
    tcase = ifline.substr(ifline.find("then")+5,ifline.find("else") - ifline.find("then") - 5);
    cond.true_case = tcase;
	
    //else block
    fcase = ifline.substr(ifline.find("else") + 5,ifline.find(";"));
    fcase = fcase.substr(0,fcase.length()-1);
    cond.false_case = fcase;
      
    }else if(ifline.find("if") != string::npos && ifline.find("then") != string::npos
      && ifline.find("else") == string::npos){
      
    }else if(ifline.find("if") != string::npos && ifline.find("then") == string::npos
      && ifline.find("else") == string::npos){
      
    }
    
    
    //state block
//     node = ifline.substr(ifline.find("(")+1,ifline.find(")") - ifline.find("(") - 1);
//     cond.node = node;
// 	
//     //then block
//     tcase = ifline.substr(ifline.find("then")+5,ifline.find("else") - ifline.find("then") - 5);
//     cond.true_case = tcase;
// 	
//     //else block
//     fcase = ifline.substr(ifline.find("else") + 5,ifline.find(";"));
//     fcase = fcase.substr(0,fcase.length()-1);
//     cond.false_case = fcase;

  }
  
public:
  RDDLParser(string& file){
    this->set_file(file);
  }
  
  bool parse_actions(){
    ifstream f(this->file_to_parse.c_str());
    if(!f.good()) return false;
    string line, action;
    while(getline(f,line) && !f.eof()){
      if(line.find("action-fluent")  != std::string::npos){
	line = this->trim(line);
	action = line.substr(0,line.find(":"));
	action = this->rtrim(action);
	this->actions.push_back(action);
      }
    }
    f.close();
    return true;
  }
  
  bool parse_states(){
    ifstream f(this->file_to_parse.c_str());
    if(!f.good()) return false;
    string line, state;
    while(getline(f,line) && !f.eof()){
      if(line.find("state-fluent")  != std::string::npos){
	line = this->trim(line);
	state = line.substr(0,line.find(":"));
	state = this->rtrim(state);
	this->states.push_back(state);
      }
    }
    f.close(); 
    return true;
  }
  
  bool find_conditionals(){
    ifstream f(this->file_to_parse.c_str());
    if(!f.good()) return false;
    string line,app;
    int index = 0;
    map<int,string> rows;
    
    getline(f,line);
    while(line.find("cpfs {") == string::npos && line.find("cdfs {") == string::npos
	  && line.find("cdfs{") == string::npos && line.find("cpfs{") == string::npos
	 ){
	    getline(f,line);
	  }
      
      while(line.find("};") == string::npos){
	getline(f,line);
	line = this->trim(line);
	if(line.find("//") != string::npos || 
	   line.empty() || line.find("};") != string::npos) continue;
	while(line.find(";") == string::npos){
	  getline(f,app);
	  app = this->trim(app);
	  if(app.find("//") != string::npos || 
	   app.empty() || app.find("};") != string::npos) continue;
	  line = line + " " + app;
	}
	
	rows.insert(make_pair(index,line));
	++index;
      }
      
      for(int i = 0; i < index; ++i){
	cout << i << " " << rows[i] << endl;
      }
      
    f.close();
    return true;
  }
  
};