// Standalone version of PNP generation from policy

#include <iostream>
#include <stack>
#include <locale.h>
#include <boost/format.hpp>


#include "policy.h"
#include "pnpgenerator.h"
#include "PRUMDP/PRU2MDP.h"


using namespace std;


float funcEmpty(const PRUstate& fromState, const PRUstate& toState, const PRUstate& actionParam, const string& kind, float parameter) {
  return 0;
}

void create_PNP_from_PRU(const string& prufile, const string& erfile, int horizon, double gamma, double epsilon) {

    std::locale::global(std::locale());
    setlocale(LC_NUMERIC,"C");

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

void usage(char *filename) {
  cout << "Usage: "<<filename<<" PRU.xml [horizon [gamma [epsilon]]] [rules_file]\n";
  cout << " horizon is the maximal number (>0) of steps to compute the policy over. (default=50)\n";
  cout << " gamma is the actuation factor (>0, <=1, default=0.99)\n";
  cout << " epsilon is the stop criterion (>0) if expected value is stable enough. (default=0)\n";
  cout << " rules_file is the name of the file containing execution rules to apply. (default none)\n" << endl;
}

int main(int argc, char** argv) {
    string pruFile = "unspecified";
    string erFile="";
    int horizon = 50;
    double gamma = 0.99;
    double epsilon = 0;

    if (argc>1)
      pruFile = string(argv[1]);
    else {
      usage(argv[0]);
      return 1;
    }
    int next = 2;
    if (argc>next) {
      char * end = NULL;
      int tmp = strtol(argv[next], &end, 10);
      if (end != argv[next]) {
        horizon = tmp;
        if ((horizon <= 0) || (end == NULL) || (*end != 0)) {
          cout << "*** Horizon "<<argv[next]<<" is invalid. Must be positive integer!\n";
          usage(argv[0]);
          return 1;
        }
        ++next;
      } // if starts like a number
    } // if horizon
    if (argc>next) {
      char * end = NULL;
      double tmp = strtod(argv[next], &end);
      if (end != argv[next]) {
        gamma = tmp;
        if ((gamma <= 0) || (gamma > 1) || (end == NULL) || (*end != 0)) {
          cout << "*** Gamma "<<argv[next]<<" is invalid. Must be >0 and <=1!\n";
          usage(argv[0]);
          return 1;
        }
        ++next;
      } // if starts like a number
    } // if gamma
    if (argc>next) {
      char * end = NULL;
      double tmp = strtod(argv[next], &end);
      if (end != argv[next]) {
        epsilon = tmp;
        if ((epsilon < 0) || (end == NULL) || (*end != 0)) {
          cout << "*** Epsilon "<<argv[next]<<" is invalid. Must be >=0!\n";
          usage(argv[0]);
          return 1;
        }
        ++next;
      } // if starts like a number
    } // if epsilon
    if (argc>next) {
      erFile = string(argv[next]);
      ++next;
    } // if last argument not read yet


  create_PNP_from_PRU(pruFile,erFile,horizon,gamma,epsilon);
    
}
