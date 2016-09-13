// Standalone version of PNP generation from conditional plan

#include <iostream>
#include <stack>

#include "conditionalplan.h"
#include "pnpgenerator.h"

// #include "pnp_translator/pddl_transl.cpp"
// #include "pnp_translator/asp_transl.cpp"
#include "pnp_translator/rddl_transl.cpp"
// #include "pnp_translator/kpddl_transl.cpp"
// #include "pnp_translator/digraph_transl.cpp"

#include "pnp_translator/rddl_parser.cpp"



using namespace std;

void create_PNP_from_KPDDL(vector<ConditionalPlan> v, string goal_name){
    map<string,pair<string,vector<ActionOutcome> > > state_action_out;
    ConditionalPlan p[v.size()];
    int i = 0;
    for(vector<ConditionalPlan>::iterator it = v.begin(); it != v.end(); ++it){
      p[i] = *it;
      state_action_out.insert(make_pair(it->state,make_pair(it->action,it->outcomes)));
      ++i;
    }

    PNPGenerator pnpgen(goal_name);

    // generate the PNP from the conditional plan
    bool r=pnpgen.genFromConditionalPlan_loop(p[0],p[v.size()-1],state_action_out);
//     bool r=pnpgen.genFromConditionalPlan(p[0]);

  if(r){
    string pnpoutfilename = goal_name+".pnml";
    pnpgen.save(pnpoutfilename.c_str());
    cout << "Saved PNP file " << pnpoutfilename << endl;
  }else {
    cout << "PNP not generated!!!" << endl;
  }
}


void create_PNP_from_PDDL(const string& file){
  ifstream f(file.c_str());
  string line;

  for(int i = 0; i < 3; i++) getline(f,line);
  string n_states =  line.substr(line.find("=")+1,line.find("\n"));
  int n = atoi(n_states.c_str());

  ConditionalPlan p[n];

  getline(f,line); //get rid of a blank line
  boost::regex st("[0-9]+");
  for(int i = 0; i < n; i++){
    getline(f,line);
    ConditionalPlan pl;
    boost::sregex_token_iterator it(line.begin(), line.end(), st);
    string state = *it;
    int end = line.find("\n")-2;
    int begin = line.find("s=")+2;
    string action = line.substr(begin,end);
    action = action.substr(0,action.size()-1);
    pl.state = state;
    pl.action = action;
    p[i]=pl;
    if(i <= n-2) p[i].addOutcome(&p[i+1]);

    it++;
  }

  p[n-1].addOutcome(ActionOutcome("",new ConditionalPlan("GOAL","")));
//   p[0].print();

  string goal_name="SimpleSerialPlan";
  cout << "Serial plan name: " << goal_name << endl;
  PNPGenerator pnpgen(goal_name);

  // generate the PNP from the conditional plan
  bool r=pnpgen.genFromConditionalPlan(p[0]);

  if(r){
    string pnpoutfilename = goal_name+".pnml";
    pnpgen.save(pnpoutfilename.c_str());
    cout << "Saved PNP file " << pnpoutfilename << endl;
  }else {
    cout << "PNP not generated!!!" << endl;
  }

}

void create_PNP_from_conditionalplan(const string& erfile) {

    ConditionalPlan p[10];
    string goal_name = "SimpleConditionalPlan";

    p[0].state = "S0"; p[0].action = "a1"; ;
    p[0].addOutcome(&p[1]);

    p[1].state = "S1"; p[1].action = "a2";
    p[1].addOutcome(ActionOutcome("X_true",&p[2]));
    p[1].addOutcome(ActionOutcome("X_false",&p[3]));
    p[1].addOutcome(ActionOutcome("X_maybe",&p[4]));

    p[2].state = "S2"; p[2].action = "a_true";
    p[2].addOutcome(&p[5]);

    p[3].state = "S3"; p[3].action = "a_false";
    p[3].addOutcome(&p[6]);

    p[4].state = "S4"; p[4].action = "a_maybe";
    p[4].addOutcome(&p[7]);

    p[5].state = "goal";
    p[6].state = "fail";
    p[7].state = "maybe";

    p[0].print();


    // Generates the PNP

    cout << "Conditional plan name: " << goal_name << endl;
    PNPGenerator pnpgen(goal_name);

    // generate the PNP from the conditional plan
    bool r=pnpgen.genFromConditionalPlan(p[0]);

    if (r) {

        if (erfile!="") {
            // apply the execution rules
            pnpgen.readERFile(erfile);
            pnpgen.applyExecutionRules();
        }

        string pnpoutfilename = goal_name+".pnml";
        pnpgen.save(pnpoutfilename.c_str());
        cout << "Saved PNP file " << pnpoutfilename << endl;
    }
    else {
        cout << "PNP not generated!!!" << endl;
    }

}


int main(int argc, char** argv) {

//     string erfile="";
//     if (argc==2)
//         erfile = string(argv[1]);
//     
//     create_PNP_from_conditionalplan(erfile);


    //=============== PDDL TRANSLATION ===============
//     string path = "/home/valerio/thesis/PDDL/ff_output.txt";
//     string write_to = "test/pddl_to_condplan.txt";
//     PDDLTransl pd(path);
//     
//     pd.read_file();
//     cout << "file read" << endl;
//     pd.write_plan(write_to); //write the translated plan to 'write_to'
//     cout << "plan write" << endl;
//     create_PNP_from_PDDL(write_to); //generate the PNP from the file 'write_to'

    //=============== KPDDL TRANSLATION ===============
    //COACHES
//     string path = "/home/valerio/thesis/PDDL/planners/KPlanner/plans/coaches.txt";
//     string write_to = "test/kpddl_to_condplan";

    
  //LIGHTS
//     string path = "/home/valerio/thesis/PDDL/planners/KPlanner/plans/lights.txt";
//     string write_to = "test/kpddl_to_condplan_lights.txt";
  
    //CLEANING
//     string path = "/home/valerio/thesis/PDDL/planners/KPlanner/plans/cleaning.txt";
//     string write_to = "test/kpddl_to_condplan_cleaning.txt";

//     KPDDLTransl kpd(path);
//     kpd.read_file();
//     kpd.write_plan(write_to); //write the translated plan to 'write_to'
// 
// 
//     vector<ConditionalPlan> v = kpd.getCondPlan();
//     string goal_name="SimpleKPDDLCondPlan_coaches";
// //     string goal_name="SimpleKPDDLCondPlan_cleaning";
//     cout << "KPDDL Conditional Plan name: " << goal_name << endl;
//     create_PNP_from_KPDDL(v,goal_name);


    //=============== ASP TRANSLATION ===============
    // string path2 = "/home/valerio/thesis/ASP/coaches/generated/clothes_4step.txt";
    // ASPTransl asp(path2);
    // asp.read_file();
    
    //=============== RDDL TRANSLATION ===============
//     string rddl_file = "/home/valerio/thesis/MDP/SPUDD/rddlsimSPUDD/files/rddl/test/dbn_prop.rddl";
//     string rddl_file = "/home/valerio/thesis/MDP/SPUDD/rddlsimSPUDD/files/rddl/test/pizza.rddl";
//   string rddl_file = "/home/valerio/thesis/MDP/SPUDD/rddlsimSPUDD/files/rddl/test/game_of_life_stoch.rddl";
  
    string rddl_file = "/home/valerio/thesis/MDP/SPUDD/rddlsimSPUDD/files/final_comp/rddl/elevators_mdp.rddl";
    RDDLParser parser(rddl_file);
    bool res;
    res = parser.parse_actions();
    if(res) res = parser.parse_states();
    if(res) res = parser.find_conditionals();
    else return -1;
    
    vector<Conditional> v = parser.get_conditionals();
    for(vector<Conditional>::iterator it = v.begin(); it != v.end(); ++it)
      cout << "node: " << (*it).node << " [t] " << (*it).true_case << " [f] " << (*it).false_case << endl;
  
  //=============== DIGRAPH TRANSLATION ===============
//   string path = "/home/valerio/thesis/ROSPlan/plan_graph_lights.dot";
//   DigraphTransl tr(path);
//   tr.read_file();
//   tr.create_PNP("DigraphCondPlan");
//   vector<ConditionalPlan> v = tr.getCondPlan();
//   v[0].print();

}
