// Standalone version of PNP generation from policy

#include<iostream>
#include <stack>

#include "policy.h"
#include "pnpgenerator.h"

using namespace std;



void create_PNP_from_policy(const char* erfile) {
    Policy p;
    // ...
    p.goal_name = "SimplePolicy";
    p.initial_state = "S0";
    p.final_state = "S3";
    // ...

    vector<StateOutcome> v0; v0.push_back(StateOutcome("personhere","S1")); v0.push_back(StateOutcome("not personhere","S2"));
    p.addStatePolicy("S0","goto_printer",v0);
    
    vector<StateOutcome> v1; v1.push_back(StateOutcome("","S2"));
    p.addStatePolicy("S1","say_hello",v1);

    vector<StateOutcome> v2; v2.push_back(StateOutcome("","S3"));
    p.addStatePolicy("S2","goto_home",v2);


    p.print();


    // Generates the PNP

    cout << "Policy name: " << p.goal_name << endl;
    PNPGenerator pnpgen(p.goal_name);

    // generate the PNP from the policy
    bool r=pnpgen.genFromPolicy(p);

    if (r) {

        if (erfile!=NULL) {
            // apply the execution rules
            pnpgen.readERFile(erfile);
            pnpgen.applyExecutionRules();
        }
        
        string pnpoutfilename = p.goal_name+".pnml";
        pnpgen.save(pnpoutfilename.c_str());
        cout << "Saved PNP file " << pnpoutfilename << endl;
    }
    else {
        cout << "PNP not generated!!!" << endl;
    }
}


int main(int argc, char** argv) {

/*
    if (argc<2) {
        cout << "    Use: " << argv[0] << " [<erfile>]" << endl;
        cout << "Example: " << argv[0] << " DIAG_printer.er" << endl;
        return -1;
    }
  */  
    const char* erfile = NULL;
    if (argc==2)
        erfile = argv[1];
    

    create_PNP_from_policy(erfile);
    
}
