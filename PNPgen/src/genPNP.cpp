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

    cout << "Policy name: " << p.goal_name << endl;
    PNPGenerator pnpgen(p.goal_name);

    bool r=pnpgen.genFromPolicy(p);

    if (r) {
        string pnpoutfilename = p.goal_name+".pnml";
        pnpgen.save(pnpoutfilename.c_str());
        cout << "Saved PNP file " << pnpoutfilename << endl;
    }
    else {
        cout << "PNP not generated!!!" << endl;
    }

}
