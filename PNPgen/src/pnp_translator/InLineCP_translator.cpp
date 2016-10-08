#include "InLineCP_translator.h"

InLineTranslator::InLineTranslator(string& path_to_file)
{
  this->file = path_to_file;
}


pair<int,int> branch(string& line, pair<int,int>& p){

  int first, last;
  if(line.find('<') != string::npos){
    
    first = line.find_last_of("<");
    last = line.find_first_of(">",first);
    
    if(first >= 0 && last >= 0){
      line = line.substr(first+1, last - first - 1);
      pair<int,int> p = make_pair(first,last);
      branch(line,p);
    }
    
  }else 
    return p;
  
}

void InLineTranslator::branch_r(string& line, vector<string>& branches){

    int i = 0;
    while(line.find('<') != string::npos){
      string s = line;
      int first = 0;
      int last = s.size();
      pair<int,int> p = make_pair(first,last);
      p = branch(s,p);
      this->trim(s);
      branches.push_back(s);
      ostringstream convert;
      convert << i;
      string str2 = "(cp"+convert.str()+")";
      line.replace(p.first, p.second - p.first + 1,str2);
//       cout << "main: " << line << endl;
      ++i;
    }
    

}

void InLineTranslator::read_file(){
  string line;

  //open the file
  ifstream ff(this->file.c_str());
  if(!ff.good()){
    cout << "the file " << this->file.c_str() << "could not be opened" << endl ;
    exit;
  }
  
    // read all the branches
  vector<string> branches;
  while(!ff.eof()){ 
      getline(ff,line);
      if(line.empty()) break;
      branch_r(line,branches);
      branches.push_back(line);
  }
  
  generate_PNP(branches);
}

vector<string> InLineTranslator::tokenize(string s){
      size_t pos = 0;
      string token;
      vector<string> tokens;
      
      while ((pos = s.find(';')) != std::string::npos) {
	token = s.substr(0, pos);
	this->trim(token);
	tokens.push_back(token);
	s.erase(0, pos + 1);
      }
      token=s;
      this->trim(token);
      tokens.push_back(token);
      
      return tokens;
}

void InLineTranslator::generate_PNP(vector< string >& branches)
{
  
     //find the last layer and subdivide (the whole plan)
     vector<string> tokens = tokenize(branches[branches.size()-1]);
//      vector<ConditionalPlan> plan(tokens.size()+1);
     ConditionalPlan* pl;
     for(int i = 0; i < tokens.size(); ++i){
       string token = tokens[i];
       
       cout << token << endl;
       
//        ostringstream convert;
//        convert << i;
//        string state = "s"+convert.str();
//        plan[i].state = state;
//        plan[i].action = token;
//        plan[i].addOutcome(&plan[i+1]);
     }
//      plan[0].print();
     
//      int j = 0;
//      for(int i = 0; i < plan.size()-1; ++i){
//        if(plan[i].action.find('(') != string::npos){
// 	 int lev = atoi(plan[i].action.substr(3,1).c_str());
// 	 cout << lev << endl;
// 	 cout << branches[lev] << endl;
//        }
//      }     
      
}



void InLineTranslator::set_file(string& f)
{
  this->file = f;
}


vector< ConditionalPlan > InLineTranslator::get_plan()
{
    if(p.size() == 0 || p.size() == 1) return this->p; 
    else {
      cout << "Plan NOT defined" << endl;
      exit;
    }
}
