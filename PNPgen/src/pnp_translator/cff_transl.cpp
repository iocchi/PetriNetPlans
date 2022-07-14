#include "pnp_translator/cff_transl.h"

  CFFTransl::CFFTransl(string &path_to_file){
    this->set_file(path_to_file);
  }

  void CFFTransl::parseline(string &line, set<string> &states, map<string,string> &state_action, map<string,string> &state_outcome){

    // cout << " -- add_stateaction(" << line << ")" << endl;
   
    string state = line.substr(0, line.find(" "));
    state = this->to_lowercase(state);
    line = this->trim_substr(line,state);
    line = this->trim(line);
    line = line.substr(4);

    std::replace(state.begin(), state.end(), '|', 'x');
    // cout << " -- state = " << state << endl;
    states.insert(state);

    // cout << "    == [" << line << "]" << endl;

    int i = line.find(" ");
    string action = line.substr(0,i);

    // cout << " -- action name = " << action << endl;

    string params;
    
    line = this->trim_substr(line,action);
    line = this->ltrim(line);
    // cout << "      == [" << line << "]" << endl;
    
    //get parameters
    i = line.find("---");
    params = line.substr(0,i);
    params = this->trim(params);

    // cout << " -- params = " << params << endl;
    
    line = line.substr(i+4);
    // cout << "        == [" << line << "]" << endl;

    std::replace(action.begin(), action.end(), '_', '-');
    std::replace(params.begin(), params.end(), ' ', '_');

    if (params!="")
        action = action + "_" + params;
    action = this->to_lowercase(action);

    state_action[state] = action;

    // cout << " -- action = " << action << endl;

    char f = line.c_str()[0];
    if (f=='S') { // single outcome
        string outstate;
        int j = line.find(':');
        outstate = line.substr(j+1);
        outstate = this->trim(outstate);
        std::replace(outstate.begin(), outstate.end(), '|', 'x');
        states.insert(outstate);
        // cout << " -- outcome: " << outstate << endl;
        state_outcome[state] = outstate;
    }
    else if (f=='T') { // true/false outcome
        string outstate_t, outstate_f;
        int j = line.find(':');
        int k = line.substr(j+2).find(' ');
        outstate_t = line.substr(j+1,k+1);
        outstate_t = this->trim(outstate_t);
        std::replace(outstate_t.begin(), outstate_t.end(), '|', 'x');
        states.insert(outstate_t);
        line = line.substr(j+1);
        j = line.find(':');
        outstate_f = line.substr(j+1);
        outstate_f = this->trim(outstate_f);
        std::replace(outstate_f.begin(), outstate_f.end(), '|', 'x');
        states.insert(outstate_f);
        state_outcome[state] = outstate_t + " " + outstate_f;
        // cout << " -- outcomes: T -> " << outstate_t << " | F -> " << outstate_f << endl;
    }


  }

  void CFFTransl::read_file(){
    string line, temp;
    set<string> states;
    map<string,string> state_action;
    map<string,string> state_outcome;
    map<string,int> state_cpidx;

    ifstream solution(this->file.c_str());
    if(!solution.good()){
      cout << "PDDL_TRANSL: file does not exist!" << endl;
      return;
    } 

    cout << "Input from file " << this->file << endl;

    //Read the output of FF and search the solution
    
    while(!solution.eof()){
    
      getline(solution,line);    // ignore

      if(line == "ff: found plan as follows"){
        getline(solution,line);

        while(!line.empty() && !solution.eof()){

          if (line.c_str()[0]!='-') {
            cout << line << endl;
            temp = line + "\n";
            this->trim(temp);
            this->parseline(temp,states,state_action,state_outcome);
          }
          getline(solution,line);
        }
      }
    }
    //plan.pop_back(); //eliminate the last empty line
    solution.close();

    cout << "Solution closed" << endl;

    cout << "Number of states found: " << states.size() << endl;

    // create states and actions
    cout << "States and actions" << endl;
    this->p.resize(states.size());
    unsigned int i = 0;
    set<string>::iterator it = states.begin();
    while(it != states.end()){
      this->p[i].state = *it;
      this->p[i].action = state_action[*it];
      state_cpidx[*it] = i;
      cout << "  "  << i << ": " << this->p[i].state << " " << this->p[i].action << endl;
      ++i; ++it;
    }

    // add all outcomes    
    cout << "Outcomes" << endl;
    it = states.begin();
    while(it != states.end()){
      ConditionalPlan *cpstate = &(this->p[state_cpidx[*it]]);
      string outstate = state_outcome[*it];
      if (outstate!="") {
          int p = outstate.find(" ");
          if (p>0) {
            string ot = outstate.substr(0,p);
            string of = outstate.substr(p+1);
            cout << "  " << *it << " ->  T " << ot << " F " << of << endl;
            string cond = cpstate->action;
            if (cpstate->action.substr(0,6)=="sense-")
                cond = cpstate->action.substr(6); // ASSUMPTION action name starts with "sense-"
            cpstate->addOutcome(cond,&(this->p[state_cpidx[ot]]));
            cpstate->addOutcome("(not "+cond+")",&(this->p[state_cpidx[of]]));
          }
          else {
            cout << "  " << *it << " -> " << outstate << endl;
            ConditionalPlan *cpstateout = &(this->p[state_cpidx[outstate]]);
            cpstate->addOutcome(cpstateout);
          }
      }


      ++it;
    }
    
    //this->p[0].print();

  }

  void CFFTransl::write_plan(string file_to_write){
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
  
    out << "}\n";
    out.close();
  
  }

