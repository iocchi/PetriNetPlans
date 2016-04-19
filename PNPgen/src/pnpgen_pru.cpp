// Standalone version of PNP generation from policy

#include <iostream>
#include <stack>
#include <locale.h>
#include <boost/format.hpp>


#include "policy.h"
#include "pnpgenerator.h"
#include "PRU2MDP.h"


using namespace std;


float funcEmpty(const PRUstate& fromState, const PRUstate& toState, const PRUstate& actionParam, const string& kind, float parameter) {
  return 0;
}

void create_PNP_from_PRU(const string& prufile, const string& erfile) {

    std::locale::global(std::locale());
    setlocale(LC_NUMERIC,"C");
    string path = "unspecified";
    string domain = "Empty";
    int horizon = 50;
    double gamma = 0.99;
    double epsilon = 0;

    PRUplus::domainFunction = &funcEmpty;

    // Loading the PRU from file
    std::cout << "Reading PRU " << prufile << std::endl;
    PRUplus pru(prufile);
    std::cout << pru <<std::endl;

    // Building the MDP from the PRU
    std::cout << "\nBuilding MDP from PRU" << std::endl;
    PRU2MDP p2m(pru);

    MDP *mdp = p2m.getMDP();
    mdp->initVI();
    std::cout << "\nNb states : " << mdp->getStates().size() << std::endl;
    p2m.states.print();

    // Solving the MDP
    std::cout << "\nSolving the MDP with Horizon " << horizon << ", Gamma " << gamma << " and Epsilon " << epsilon << std::endl;
    int h;
    int effectiveHorizon=0;
    float delta = 1 + epsilon;
    for (h=horizon; (h>0) && (delta>epsilon); --h) {
        std::cout << "Horizon "<<h<<": " << (delta = mdp->iterate((float)gamma)) << std::endl;
        ++effectiveHorizon;
    }
    mdp->printPolicy(std::cout);


    // Computing the policy
    std::cout << "\nOptimal plan is \n";
    Policy policy;

    // Setting the goal name
    policy.goal_name  = prufile;

    size_t p = string(prufile).find_last_of ('.');
    if (p!=string::npos) {
        policy.goal_name = prufile.substr(0,p);
    }


    std::set<int> current_states,next_states;
    current_states.insert(0); // init
    MDPstate* goalState = NULL;
    bool prevOnlyGoals = false;
    for (int steps=effectiveHorizon-1; steps>=0; --steps) {
      std::string cstep = str(boost::format(" & _step( %d )") %steps);
      std::string nstep = str(boost::format(" & _step( %d )") %(steps>0?steps-1:0));
      const vector<const MDPaction*> &pol = mdp->getPolicy(steps);
      next_states.clear();
      bool onlyGoals = true;
      for (std::set<int>::iterator itS=current_states.begin(); itS!=current_states.end(); ++itS) {
        int s = *itS;
        string s_st = mdp->getState(s)->getPredicates(cstep);
        std::string s_act;
        vector<StateOutcome> s_outcome;
        std::cout << "state= " << s_st << std::endl;
        //stateAction.state = mdp->getState(s)->getPredicates(cstep);

        const MDPaction *a = pol[s];
        if (a != NULL) {
          //stateAction.action = a->actionName;
          s_act = a->actionName;
          std::cout << " action= " << a->actionName;
          if (! a->parameters.empty()) {
            //stateAction.action += "( ";
            s_act += "_";
            bool notFirst = false;
            for (PRUstate::const_iterator it = a->parameters.begin();it != a->parameters.end(); ++it) {
              std::cout << "_" << it->second;
              if (notFirst)
                //stateAction.action += ", ";
                s_act += "_";
              else
                notFirst = true;
              //stateAction.action += it->second;
              s_act += it->second;
            } // for it in a's parameters
                //stateAction.action += " )";
            s_act += "";
          } // if parametric action
          std::cout << std::endl;
          for (std::set<MDPstate*>::const_iterator it = a->outcomes.begin(); it!= a->outcomes.end(); ++it) {
            std::cout << " observation= " << (*it)->prevOutcome->observable << std::endl;
            std::cout << "  => " << (*it)->getPredicates(nstep) << std::endl;
            if ((*it)->prevOutcome->isFinal)
              goalState = *it;
            else
              onlyGoals = false;
            next_states.insert((*it)->index);
            s_outcome.push_back(StateOutcome((*it)->prevOutcome->observable,(*it)->getPredicates(nstep)));
          } // for it in outcomes of a
        } else {
          std::cerr << "No action defined for MDP state "<<s_st<<"!"<<std::endl;
          //stateAction.action = "none";
          s_act = "none";
        } // if a != null
        //pol.push_back(stateAction);
        policy.addStatePolicy(s_st,s_act,s_outcome);
      } //for s in current_states
      current_states = next_states;
      if (prevOnlyGoals && onlyGoals)
        break; // No need to go further, all branches are at goal states.
      prevOnlyGoals = onlyGoals;
    } // for steps

    policy.initial_state = mdp->getState(0)->getPredicates("");
    if (goalState != NULL) {
        std::cout << "final_state= " << goalState->getPredicates() << std::endl;
        //plan.final_state = goalState->getPredicates();
        policy.final_state = goalState->getPredicates();
    }

    policy.print();


    // Generating the PNP

    PNPGenerator pnpgen(policy.goal_name);

    // generate the PNP from the policy
    bool r=pnpgen.genFromPolicy(policy);

    if (r) {

        if (erfile!="") {
            // apply the execution rules
            pnpgen.readERFile(erfile);
            pnpgen.applyExecutionRules();
        }
        
        string pnpoutfilename = policy.goal_name+".pnml";
        pnpgen.save(pnpoutfilename.c_str());
        cout << "Saved PNP file " << pnpoutfilename << endl;
    }
    else {
        cout << "PNP not generated!!!" << endl;
    }

    // memory release
    delete mdp;
}


int main(int argc, char** argv) {


    if (argc<2) {
        cout << "    Use: " << argv[0] << " <PRU file> [<erfile>]" << endl;
        cout << "Example: " << argv[0] << " demo1.pru DIAG_printer.er" << endl;
        return -1;
    }

    string prufile = string(argv[1]);

    string erfile="";
    if (argc==3)
        erfile = string(argv[2]);

    create_PNP_from_PRU(prufile,erfile);
    
}
