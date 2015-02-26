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


int main(int argc, char **argv)
{
    vector<pair<string,string> > socialrules;
    socialrules.push_back(make_pair("before HloadPaper","Sapproach"));
    socialrules.push_back(make_pair("during Sapproach","Sexplainapproach"));
    socialrules.push_back(make_pair("after Sapproach","Ssayplease"));
    socialrules.push_back(make_pair("after Sapproach","Sexplainhelp"));
    socialrules.push_back(make_pair("after HloadPaper","Sthank"));
    socialrules.push_back(make_pair("before HopenDoor","Sapproach"));
    socialrules.push_back(make_pair("after HopenDoor","Sthank"));
    socialrules.push_back(make_pair("during say","S_face"));
    socialrules.push_back(make_pair("during say","S_display_text"));


    vector<string> plan;

    // main plan
    plan.push_back("gotoPrinter"); plan.push_back("findHuman");
    plan.push_back("HloadPaper"); plan.push_back("gotoDoor"); plan.push_back("HopenDoor");
    plan.push_back("passDoor");

    generate("main",plan,socialrules);

    // H_loadPaper
    plan.clear(); plan.push_back("preLoadPaper"); plan.push_back("say_loadPaper");
    plan.push_back("percLoadPaper");

    generate("HloadPaper",plan,socialrules);

    // H_openDoor
    plan.clear(); plan.push_back("preOpenDoor"); plan.push_back("say_openDoor");
    plan.push_back("percOpenDoor");

    generate("HopenDoor",plan,socialrules);

    return 0;
}

