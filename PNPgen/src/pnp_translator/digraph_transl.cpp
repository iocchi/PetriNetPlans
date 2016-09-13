#include "digraph_transl.h"

  DigraphTransl::DigraphTransl(string& path_to_file, string& pnml_out)
  {
    this->set_file(path_to_file);
    this->set_pnml(pnml_out);
  }


  void DigraphTransl::create_PNP(string& plan_name){
    map<string,pair<string,vector<ActionOutcome> > > state_action_out;
    ConditionalPlan p[this->p.size()];
    int i = 0;
    for(vector<ConditionalPlan>::iterator it = this->p.begin(); it != this->p.end(); ++it){
      p[i] = *it;
      state_action_out.insert(make_pair(it->state,make_pair(it->action,it->outcomes)));
      ++i;
    }

    PNPGenerator pnpgen(plan_name);

    // generate the PNP from the conditional plan
    bool r=pnpgen.genFromConditionalPlan_loop(p[0],p[this->p.size()-1],state_action_out);
//     bool r=pnpgen.genFromConditionalPlan(p[0]);

    if(r){
      string pnpoutfilename = this->pnml + plan_name+".pnml";
      pnpgen.save(pnpoutfilename.c_str());
      cout << "Saved PNP file " << pnpoutfilename << endl;
    }else {
      cout << "PNP not generated!!!" << endl;
    }
}

void DigraphTransl::create_PNP(string& plan_name, vector<ConditionalPlan>& v){
  map<string,pair<string,vector<ActionOutcome> > > state_action_out;
  ConditionalPlan p[v.size()];
  int i = 0;
  for(vector<ConditionalPlan>::iterator it = v.begin(); it != v.end(); ++it){
    p[i] = *it;
    state_action_out.insert(make_pair(it->state,make_pair(it->action,it->outcomes)));
    ++i;
  }

  PNPGenerator pnpgen(plan_name);

  // generate the PNP from the conditional plan
  bool r=pnpgen.genFromConditionalPlan_loop(p[0],p[v.size()-1],state_action_out);
//     bool r=pnpgen.genFromConditionalPlan(p[0]);

  if(r){
    string pnpoutfilename = this->pnml + plan_name+".pnml";
    pnpgen.save(pnpoutfilename.c_str());
    cout << "Saved PNP file " << pnpoutfilename << endl;
  }else {
    cout << "PNP not generated!!!" << endl;
  }
}

  void DigraphTransl::read_file(){
    string line;
    ifstream f(this->file.c_str());
    cout << this->file.c_str() << endl;
    if(!f.good()){
      cout << "DIGRAPH_TRANSL: file does not exist!" << endl;
      return;
    }

    getline(f,line); //get rid of the first line
    this->plan_name = line.substr(line.find(" "),line.find("{")-line.find(" "));
    this->plan_name = this->trim(this->plan_name);

      //reads all the states
     boost::regex exp("\"([A-Za-z_-])+\"");
     getline(f,line);
     while(line.substr(0,1) != "\""){
       string state = line.substr(0,line.find("["));
       boost::sregex_token_iterator it(line.begin(), line.end(), exp, 0);
       string action = *it;
       action = action.substr(1,action.size()-2);
       pair<string,string> sa = make_pair(state,action);
       state_action.insert(sa);
       getline(f,line);
     }

     ConditionalPlan plan[state_action.size()];
     map<string,string>::iterator it;
     int i = 0;
     for(it = state_action.begin(); it != state_action.end(); ++it, ++i){
       ConditionalPlan p;
       p.state = it->first;
       p.action = it->second;
       plan[i] = p;
     }

      //reads all the transition
      boost::regex exp2("\"[0-9]+\"");
      boost::regex exp3("\"\\((.*?)\\)\""); // "\((.*?)\)"
//       getline(f,line);
      while(line != "}"){
      	boost::sregex_token_iterator it(line.begin(), line.end(), exp2, 0);
      	string source = *it;
      	source = source.substr(1,source.size()-2);

      	++it;
      	string destination = *it;
      	destination = destination.substr(1,destination.size()-2);

      	string obs;
      	if(line.find("[") != string::npos){
      	  boost::sregex_token_iterator it2(line.begin(), line.end(), exp3, 0);
      	  obs = *it2;
      	  obs = obs.substr(1,obs.size()-2);
      	}
      	else obs = "";

      	int so = atoi(source.c_str());
      	int dest = atoi(destination.c_str());
      	plan[so].addOutcome(ActionOutcome(obs,&plan[dest]));

      	getline(f,line);
      }

      ConditionalPlan ps("goal","");
      plan[state_action.size()-1].addOutcome(&ps);

      vector<ConditionalPlan> v(plan,plan+state_action.size());
      this->p=v;
      this->p[0].print();

    f.close();
    this->create_PNP(this->plan_name,this->p);
  }

  void DigraphTransl::write_pnml(vector<ConditionalPlan> &v){
    this->create_PNP(this->plan_name,v);
  }
