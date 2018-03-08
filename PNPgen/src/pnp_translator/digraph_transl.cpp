#include "pnp_translator/digraph_transl.h"
#include <boost/algorithm/string.hpp> 

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

    string erfile = "/home/viki/src/robocupathome_pnp/plans/default.er";

    if (erfile!="") {
        // apply the execution rules
        cout << "Applying Execution Rules..." << endl;
        pnpgen.readERFile(erfile);
        pnpgen.applyExecutionRules();
    }


    string pnpoutfilename = this->pnml + plan_name+".pnml";
    pnpgen.save(pnpoutfilename.c_str());
    cout << "Saved PNP file " << pnpoutfilename << endl;
  }else {
    cout << "PNP not generated!!!" << endl;
  }
}


string extract_action(string &app) {
    string act = "";
    int i = 0;
    char ci = app.at(i);
    while(ci != '\"'){
      if (ci!=' ')
        act += app.at(i); 
      else
        act += '_';
      ++i;
      ci = app.at(i);
    }
    if (act.at(i-1)=='_') act = act.substr(0,i-1); // remove last space

    boost::replace_all(act, "__var__", "@");

    return act;
}


void DigraphTransl::create_PNP(){
    string line;
    ifstream f(this->file.c_str());
//     cout << this->file.c_str() << endl;
    if(!f.good()){
      cout << "DIGRAPH_TRANSL: file does not exist!" << endl;
      return;
    }

    getline(f,line); //get rid of the first line
    this->plan_name = line.substr(line.find(" "),line.find("{")-line.find(" "));
    this->plan_name = this->trim(this->plan_name);

      //reads all the states
//      boost::regex exp("\"([A-Za-z0-9_-])+\"");
     boost::regex exp("\"([^\"]*)\"");
     getline(f,line);
     vector<pair<string,string> > vect;
     while(line.substr(0,1) != "\""){
       string state = line.substr(0,line.find("["));
       string app = line.substr(line.find("\"")+1,line.find("\n")-line.find("\""));
       string action = extract_action(app);


       pair<string,string> sa = make_pair(state,action);
//        state_action.insert(sa);
       vect.push_back(sa);
//        cout << "state_action " << state << " - " << action << endl;
       getline(f,line);
     }

//      ConditionalPlan plan[state_action.size()];
//      map<string,string>::iterator it;
     ConditionalPlan plan[vect.size()];
     vector<pair<string,string> >::iterator it;
     int i = 0;
     
//      for(it = state_action.begin(); it != state_action.end(); ++it, ++i){
//        ConditionalPlan p;
//        p.state = it->first;
//        p.action = it->second;
//        cout << "p.state " << p.state << endl;
//        cout << "p.action " << p.action << endl;
//        cout << endl;
//        cout << "vect[" <<i<< "] " << vect[i].first << " - " << vect[i].second << endl;
//        cout << endl;
//        plan[i] = p;
//      }
     for(it = vect.begin(); it != vect.end(); ++it, ++i){
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
	string source, destination;
	boost::sregex_token_iterator it(line.begin(), line.end(), exp2, 0);
	source = *it;
	source = source.substr(1,source.size()-2);
// 	cout << "source " << source << endl;

	++it;
	destination = *it;
	destination = destination.substr(1,destination.size()-2);
// 	cout << "destination " << destination << endl;

      	string obs;
      	if(line.find("[") != string::npos){
	  
// 	  boost::sregex_token_iterator it2(line.begin(), line.end(), exp3, 0);
// 	  obs = *it2;
//       	  obs = obs.substr(1,obs.size()-2);
	  string app = line.substr(line.find("=")+2,line.find("\n")-line.find("\""));
	  string act;
	  int i = 0;
	  while(app.at(i) != '\"'){
	    act += app.at(i); 
	    ++i;
	  }
	  obs = act;
	  boost::algorithm::trim(obs);
	  
      	}
      	else obs = "";
// 	cout << "with observation " << obs << endl;
// 	cout << endl;

      	int so = atoi(source.c_str());
      	int dest = atoi(destination.c_str());
      	if(obs != "") plan[so].addOutcome(ActionOutcome(obs,&plan[dest]));
	    else plan[so].addOutcome(&plan[dest]);

      	getline(f,line);
      }

      ConditionalPlan ps("goal","");
//       plan[state_action.size()-1].addOutcome(&ps);
      plan[vect.size()-1].addOutcome(&ps);
//       vector<ConditionalPlan> v(plan,plan+state_action.size());
      vector<ConditionalPlan> v(plan,plan+vect.size());
      this->p=v;
//       this->p[0].print();
    f.close();
    
//     this->write_plan("condplan.txt");
    string pl_name = "AUTOGEN_"+this->plan_name;
    this->create_PNP(pl_name,this->p);
  }



  void DigraphTransl::write_pnml(vector<ConditionalPlan> &v){
    this->create_PNP(this->plan_name,v);
  }
