#include <fstream>
#include <iostream>
#include <map>
#include <boost/algorithm/string.hpp>

#include "pnpgenerator.h"

using namespace std;


void create_PNP_from_linear_plan(const string& planfile, const string& erfile) {
    
    // vector<string> v; boost::split(v,planfile,boost::is_any_of("."),boost::token_compress_on);
    // if (v.size()>0) planname = v[0];

    // Setting the goal name
    string goal_name = planfile;
    size_t p = string(planfile).find_last_of ('.');
    if (p!=string::npos) {
        goal_name = planfile.substr(0,p);
    }


    PNPGenerator pnpgen(goal_name);
    
    // read the linear plan from file
    string main_plan;

    if (boost::ends_with(planfile, ".plan"))
        pnpgen.readPlanFile(planfile, main_plan);
    else
        pnpgen.readPDDLOutputFile(planfile, main_plan);
    
    cout << "Plan: " << main_plan << endl;
    // generate the main PNP from the linear plan
    pnpgen.setMainLinearPlan(main_plan);
    
    if (erfile!="") {
        // apply the execution rules
        pnpgen.readERFile(erfile);
        pnpgen.applyExecutionRules();
    }
    
    // save the PNP
    pnpgen.save();
    
}


int main(int argc, char **argv)
{    
    if (argc<2) {
        cout << "    Use: " << argv[0] << " <planfile> [<erfile>]" << endl;
        cout << "Example: " << argv[0] << " DIAG_printer.plan DIAG_printer.er" << endl;
        cout << "Example: " << argv[0] << " DIAG_printer.pddlout DIAG_printer.er" << endl;
        return -1;
    }
    
    string planfile = string(argv[1]);

    string erfile="";
    if (argc==3)
        erfile = string(argv[2]);


    create_PNP_from_linear_plan(planfile,erfile);

    return 0;
}

