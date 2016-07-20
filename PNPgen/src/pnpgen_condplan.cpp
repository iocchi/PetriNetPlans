// Standalone version of PNP generation from conditional plan

#include<iostream>
#include<stack>

#include "conditionalplan.h"
#include "pnpgenerator.h"

using namespace std;



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

    string erfile="";
    if (argc==2)
        erfile = string(argv[1]);

    create_PNP_from_conditionalplan(erfile);
    
}

