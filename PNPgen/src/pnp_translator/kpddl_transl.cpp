#include "pnp_translator/kpddl_transl.h"


  KPDDLTransl::KPDDLTransl(string &path_to_file){
    this->set_file(path_to_file);
  }

  string KPDDLTransl::find_state(int num){
    string res;
    for(int i = 0; i < this->plan.size(); i++){
      res = this->ltrim(this->plan[i]);
      if(atoi(res.substr(0,res.find(" ")).c_str()) == num){
        this->plan.erase(this->plan.begin()+i);
        return res;
      }
    }

    return "";
  }

  ConditionalPlan KPDDLTransl::make_state(string &state){
    
    vector<pair<string,int> > v;
    state = this->to_lowercase(state); //normalize the state
    boost::regex num(" [0-9]+");

    ConditionalPlan pl;
    pl.state = state.substr(0,state.find(" "));
    state = state.substr(state.find(":"),state.find("\n"));
    string totrim = ": ";
    string action;
    
    //check if conditional branching is involved
    if(state.find("(") != string::npos) action = state.substr(state.find(" "),state.find("-->")-5);
    else action = state.substr(state.find(" "),state.find("-->")-1);
   
    action = this->trim(action);
    pl.action = action;
    
    if(state.find("(") != string::npos){ //conditional transition
      string temp = state;
      while(!temp.empty()){
	string obs = temp.substr(temp.find("(")+1,temp.find(")") - temp.find("(")-1);
	temp = temp.substr(temp.find(")")+1,temp.find("\n"));
	boost::sregex_token_iterator it(temp.begin(), temp.end(), num);
	string t = *it;
	int to = atoi(t.c_str());
	if(to == 0) pl.addOutcome(ActionOutcome(obs,&this->p[this->p.size()-1]));
	else pl.addOutcome(ActionOutcome(obs,&this->p[to]));

	temp = temp.substr(temp.find("   ")+1,temp.find("\n")-temp.find("   "));
	temp = this->trim(temp);
      }
    }
    else{	//no conditional transition
      string app = state.substr(state.find(">")+1,state.find("\n") - state.find(">"));
      app = this->trim(app);
      int succ = atoi(app.c_str());

      if(succ == 0) pl.addOutcome(&this->p[this->p.size()-1]);
      else pl.addOutcome(&this->p[succ]);
      
    }
    
    return pl;
  }
 

  void KPDDLTransl::read_file(){
    string line, init, goal;
    vector<string> plan;
    ifstream solution(this->file.c_str());

    while(getline(solution,line) && !line.empty()){
      this->plan.push_back(line);
    }
    
    //initialize p with blank ConditionalPlan
    for(int i = 0; i < this->plan.size()-1; i++){ //-1: due to GOAL and INIT
      ConditionalPlan temp;
      this->p.push_back(temp);
    }
        
    //make the initial state
    init = this->plan[0];
    int num = atoi(init.substr(init.find(" "),init.find("\n")).c_str());
    init = find_state(num);
    this->plan.erase(this->plan.begin());
    ConditionalPlan in = make_state(init);
    this->p[0] = in;
    
    //make the goal state
    goal = this->plan[this->plan.size()-1];
    goal = this->ltrim(goal);
    int num2 = atoi(goal.substr(0,1).c_str());
    goal = find_state(num2);
    ConditionalPlan go = ConditionalPlan("GOAL","");
    this->p[this->p.size()-1] = go;
    
    for(int i = 0, j = 1; i < this->plan.size(); i++, j++){
      ConditionalPlan pl = make_state(this->plan[i]);
      this->p[j] = pl;
    }

  }

