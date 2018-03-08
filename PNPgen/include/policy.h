#pragma once

#include <vector>
#include <map>
#include <string>
#include <iostream>

using std::cout;
using std::endl;

struct StateOutcome { // Represents one possible outcome of an action used in a state
    std::string observation; // a formula describing what should be observed when this outcome is active
    std::string successor; // the next state that will be reached

    // makes a copy of all data
    StateOutcome(std::string o, std::string s) : observation(o), successor(s) {}
};

struct StatePolicy { // represents a pair of (state, action) for MDP policies
    std::string state;           // the description of this state
    std::string action;           // the action to implement in this state
    std::vector<StateOutcome> outcomes;   // the list of possible outcomes of this action when used in this state

    // makes a copy of all data
    StatePolicy(std::string s, std::string a, std::vector<StateOutcome> o) : state(s), action(a), outcomes(o) {}
};
                     
struct Policy { // represents the policy of a PRU for going to somewhere and activate a task
    std::string goal_name;      // the name of the goal this policy solves
    std::string initial_state;  // the name of the state to start this PRU
    std::string final_state;    // the name of the state to end this PRU
    std::vector<StatePolicy> policy;   // best policy computed

    std::map<std::string, std::string> state_action; // state action map for quick access
    std::map< std::pair<std::string, std::string>, std::vector<StateOutcome> > state_action_outcomes; // <state,action> outcomes map for quick access

    void addStatePolicy(std::string s, std::string a, std::vector<StateOutcome> o) {
        // makes a copy of all data
        policy.push_back(StatePolicy(s,a,o));
        state_action[s]=a;
        state_action_outcomes[std::make_pair(s,a)]=o;
    }

    // returns the action associated to this state
    std::string getActionForState(std::string s) {
        return state_action[s];
    }

    // get the outcomes for this stae action pair
    std::vector<StateOutcome> & getOutcomes(std::string s, std::string a) {
        return state_action_outcomes[std::make_pair(s,a)];
    }

    void print() {
        cout << "Policy name: " << goal_name << endl;
        cout << "Init: " << initial_state << endl;
        cout << "Final: " << final_state << endl;


        std::vector<StatePolicy>::iterator it = policy.begin();
        while (it!=policy.end()) {
            cout << it->state << " : " << it->action << " -> { ";
            std::vector<StateOutcome>::iterator jt = it->outcomes.begin();
            while (jt<it->outcomes.end()) {
                cout << "[" << jt->observation << "] " << jt->successor;
                jt++;
                if (jt<it->outcomes.end()) cout << ", ";
            }
            cout << " } " << endl;
            it++;
        }
    }

};
