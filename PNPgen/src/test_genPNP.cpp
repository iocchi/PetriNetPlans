#include <fstream>
#include <iostream>
#include <cstdlib>
#include <map>

#include "pnpgenerator.h"

using namespace std;


void generate(string name, vector<string> &plan, vector<pair<string,string> > &socialrules)
{
    PNPGenerator g(name);
    g.genLinear(plan);
    g.applyRules(socialrules);
    g.save();
}


void generateH(string name, vector<pair<string,string> > &socialrules)
{
    PNPGenerator g(name);
    if (name=="HloadPaper")
        g.genHumanAction("say_getPapers","say_printerThanks","say_sorry","objectPickedUp");
    else if (name=="HopenDoor")
        g.genHumanAction("say_openDoor","say_doorThanks","say_sorry","doorOpen");
    g.applyRules(socialrules);
    g.save();
}

int main(int argc, char **argv)
{
    vector<pair<string,string> > socialrules;
    
    socialrules.push_back(make_pair("after findHuman","approach"));
    socialrules.push_back(make_pair("after findHuman","say_greet"));
    socialrules.push_back(make_pair("after findHuman","explain_help"));
    socialrules.push_back(make_pair("after HloadPaper","say_thank"));
    socialrules.push_back(make_pair("after HopenDoor","say_thank"));
    socialrules.push_back(make_pair("during approach","explain_approach"));
    socialrules.push_back(make_pair("during say","face"));
    socialrules.push_back(make_pair("during say","display_text"));
    
    vector<string> plan;

    // main plan
    plan.push_back("goto_printer");
    //plan.push_back("findHuman"); 
    plan.push_back("HloadPaper"); 
    plan.push_back("goto_door"); 
    //plan.push_back("findHuman"); 
    plan.push_back("HopenDoor"); 
    plan.push_back("enter_door");

    generate("main",plan,socialrules);

    // H_loadPaper
    generateH("HloadPaper",socialrules);

    // H_openDoor
    generateH("HopenDoor",socialrules);

    return 0;
}

