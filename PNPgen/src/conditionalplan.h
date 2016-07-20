#pragma once

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <sstream>

using std::cout;
using std::endl;

struct ConditionalPlan;

struct ActionOutcome { // Represents one possible outcome of an action used in a state
    std::string observation; // a formula describing what should be observed when this outcome is active
    struct ConditionalPlan *successor;  // the next state that will be reached

    // makes a copy of all data
    ActionOutcome(std::string o, struct ConditionalPlan *s) : observation(o), successor(s) {}
};

struct ConditionalPlan { 
    std::string state;     // the description of this state
    std::string action;    // the action to execute in this state

    std::vector<ActionOutcome> outcomes;   // the list of possible outcomes of this action when used in this state

    ConditionalPlan(): state(""), action("") {}
    ConditionalPlan(std::string s, std::string a) : state(s), action(a) {}
    ConditionalPlan(std::string s, std::string a, std::vector<ActionOutcome> o) : state(s), action(a), outcomes(o) {}

    void addOutcome(ActionOutcome o) { outcomes.push_back(o); }
    void addOutcome(ConditionalPlan *s) { outcomes.push_back(ActionOutcome("",s)); }

    std::string tab(int level) {
        std::stringstream o;
        for (int i=0; i<level; i++) o << "  ";
        return o.str();
    }

    void print(int level=0) {
        cout << tab(level) << "State: " << state << endl;
        cout << tab(level) << "Action: " << action << endl;
        cout << tab(level) << "Successors: { " << endl;

        std::vector<ActionOutcome>::iterator it = outcomes.begin();
        while (it!=outcomes.end()) {
            cout << tab(level+1) << "[" << it->observation << "]" << endl;
            it->successor->print(level+1);
            it++;
        }

        cout << tab(level) << "} " << endl;
    }
};

#if 0                
struct ConditionalPlan { // represents the policy of a PRU for going to somewhere and activate a task
    std::string goal_name;      // the name of the goal this policy solves
    std::string initial_state;  // the name of the state to start this PRU
    std::string final_state;    // the name of the state to end this PRU
    std::vector<StatePolicy> policy;   // best policy computed

    std::map<std::string, std::string> state_action; // state action map for quick access
    std::map< std::pair<std::string, std::string>, std::vector<ActionOutcome> > state_action_outcomes; // <state,action> outcomes map for quick access

    void addStatePolicy(std::string s, std::string a, std::vector<ActionOutcome> o) {
        // makes a copy of all data
        policy.push_back(StatePolicy(s,a,o));
        state_action[s]=a;
        state_action_outcomes[std::make_pair(s,a)]=o;
    }

    // returns the action associated to this state
    std::string getActionForState(std::string s) {
        return state_action[s];
    }

};
#endif

