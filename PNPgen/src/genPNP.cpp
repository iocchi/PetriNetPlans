// Standalone version of PNP generation from policy

#include<iostream>
#include <stack>

#include "policy.h"
#include "pnpgenerator.h"

using namespace std;

int main(int argc, char** argv) {
    Policy p;
    // ...
    p.goal_name = "Test";
    p.initial_state = "S0";
    p.final_state = "S2";
    // ...

    vector<StateOutcome> v0; v0.push_back(StateOutcome("","S1"));
    p.addStatePolicy("S0","a",v0);
    
    vector<StateOutcome> v1; v1.push_back(StateOutcome("o1","S2")); v1.push_back(StateOutcome("not o1","S3"));
    p.addStatePolicy("S1","b",v1);

    vector<StateOutcome> v2; v2.push_back(StateOutcome("","S0"));
    p.addStatePolicy("S3","c",v2);


    p.print();


    // Generates the PNP
    bool PNPgen_error = false; // check if errors occur during generation

    cout << "Policy name: " << p.goal_name << endl;
    PNPGenerator pnpgen(p.goal_name);

    cout << "Init: " << p.initial_state << endl;
    Place *p0 = pnpgen.pnp.addPlace("init"); p0->setInitialMarking();
    string current_state = p.initial_state;
    // add a first fixed transition from the init place to the initial state
    pair<Transition*,Place*> pa = pnpgen.pnp.addCondition("[]",p0);
    Place *p1 = pa.second;
    p1->setName(current_state);


    cout << "Final: " << p.final_state << endl;
    //string final_state_str = transformState(final_state);

    // Map of visited states (with associated places)
    std::map<string,Place*> visited;

    // Initialization of the stack
    stack< pair<string, Place*> > SK; SK.push(make_pair(current_state,p1));

    while (!SK.empty()) {

        // get top element from stack
        current_state=SK.top().first; Place* current_place = SK.top().second;
        SK.pop();

        std::cout << "PNPgen::  " << current_state << " : ";
        string action = p.getActionForState(current_state);

        if (action=="") {
            std::cerr << "PNPgen Warning: No action found for state " << current_state << std::endl;
            continue;
        }
        std::cout << action << " -> ";

        vector<StateOutcome> vo = p.getOutcomes(current_state,action);

        if (vo.size()==0) {
            std::cerr << "PNPgen ERROR: No successor state found for state " << current_state << " and action " << action  << std::endl;
            PNPgen_error = true;
            break;
        }

        Place *pe = pnpgen.addAction(action,current_place);  // pe: end place of action

        // y coordinate of Petri Net layout
        int dy=0;

        vector<StateOutcome>::iterator io;
        for (io = vo.begin(); io!=vo.end(); io++) {
            string succ_state = io->successor;
            string cond = io->observation;

            cout << "[" << cond << "] " << succ_state << "    ";

            Place *ps = visited[succ_state];


            // check if succ_state is already visited
            if (ps==NULL) { // if not visited
                pair<Transition*,Place*> pa = pnpgen.pnp.addCondition(cond,pe,dy); dy++;
                Place* pc = pa.second;
                SK.push(make_pair(succ_state,pc));
                visited[succ_state]=pc;
                pc->setName(succ_state);
                if (succ_state==p.final_state)
                    pc->setName("goal");
            }
            else { // if already visited
                pnpgen.pnp.addConditionBack(cond,pe, ps, dy); dy++;
            }

        } // for io

        std::cout << std::endl;

    }


    if (!PNPgen_error) {
        string pnpoutfilename = p.goal_name+".pnml";
        pnpgen.save(pnpoutfilename.c_str());
        cout << "Saved PNP file " << pnpoutfilename << endl;
    }
    else {
        cout << "PNP not generated!!!" << endl;
    }


    

}
