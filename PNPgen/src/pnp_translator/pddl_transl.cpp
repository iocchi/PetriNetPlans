#include "pnp_translator/pddl_transl.h"

  PDDLTransl::PDDLTransl(string &path_to_file){
    this->set_file(path_to_file);
  }

  ConditionalPlan PDDLTransl::make_state(string &state){
    ConditionalPlan cp;
    string state_name = state.substr(0, state.find(" ")-1);
    state_name = this->to_lowercase(state_name);
    cp.state = state_name;
    state_name = state_name + ": ";
    state = this->trim_substr(state,state_name);

    int i = state.find(" ");
    string action = state.substr(0,i);
    string params;
    
    state = this->trim_substr(state,action);
    state = this->ltrim(state);
    
    //get all the parameters
    while(!state.empty()){
      if(state.find(" ") != string::npos) params = state.substr(0,state.find(" "));
      else params = state.substr(0,state.find("\n"));
      action = action + "_" + this->trim(params);
      state = this->trim_substr(state,params);
      state = this->trim(state);
    }
    
    
    action = this->to_lowercase(action);
    cp.action = action;

    return cp;
  }

  void PDDLTransl::read_file(){
    string line, temp;
    vector<string> plan;

    ifstream solution(this->file.c_str());
    if(!solution.good()){
      cout << "PDDL_TRANSL: file does not exist!" << endl;
      return;
    } 

    //Read the output of FF and search the solution
    while(solution >> line && !solution.eof()){
      if(line == "step"){
        getline(solution,line);
        while(!line.empty() && !solution.eof()){
          temp = line + "\n";
          this->trim(temp);
          plan.push_back(temp);
          getline(solution,line);
        }
      }
    }
    plan.pop_back(); //eliminate the last empty line
    solution.close();

    //Create the states from each line of solution
    ConditionalPlan ps[plan.size()];
    unsigned int i = 0;
    vector<string>::iterator it = plan.begin();
    while(it != plan.end()){
      string att = *it;
      ConditionalPlan state = make_state(att);
      state.addOutcome(&ps[i+1]);
      ps[i] = state;
      // cout << ps[i].state << " " << ps[i].action << endl;
      ++i; ++it;
    }

    vector<ConditionalPlan> app(ps,ps+plan.size());
    this->p=app;

  }

  void PDDLTransl::write_plan(string file_to_write){
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
        out << ",actions=";
        out << p[i].action;
        out << "]\n";
    }
    out << "\n";
  
    for(int i = 0; i < this->p.size()-1;i++){
        out << " \"";
        out << p[i].state;
        out << "\"";
        out << " -> ";
        out << "\"";
        out << p[i+1].state;
        out << "\"\n";
    }
  
    out << "}";
    out.close();
  
  }
