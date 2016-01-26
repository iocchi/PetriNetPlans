#include <fstream>
#include <iostream>
#include <cstdlib>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/regex.hpp>

#include "pnpgenerator.h"

using namespace std;

#if 0
void generate(string name, string plan, TSocialRules &socialrules, TExecutionRules &executionrules)
{
   PNPGenerator g(name);

    g.setMainLinearPlan(plan);
    g.applySocialRules(socialrules);
    g.applyExecutionRules(executionrules);
    g.save(); 
}


void generateH(string name, TSocialRules &socialrules, TExecutionRules &executionrules)
{
    PNPGenerator g(name);
    if (name=="HloadPaper")
        g.genHumanAction("say_getPapers","say_printerThanks","turn_0_ABS","say_sorry","objectPickedUp");
    else if (name=="HopenDoor")
        g.genHumanAction("say_openDoor","say_doorThanks","turn_90_ABS","say_sorry","doorOpen");
    g.applySocialRules(socialrules);
    g.applyExecutionRules(executionrules);
    g.save();
}



void gen_IROS15_example() {
    TSocialRules socialrules;

    TExecutionRules executionrules;

    // V1
    socialrules.push_back(make_pair("after findHuman","approach"));

    // V3
    socialrules.push_back(make_pair("after findHuman","say_greet"));

    // V4
    // socialrules.push_back(make_pair("after findHuman","explain_help"));

    // True
    socialrules.push_back(make_pair("after HloadPaper","say_thank"));

    // True
    socialrules.push_back(make_pair("after HopenDoor","say_thank"));

    // False
    // socialrules.push_back(make_pair("during approach","explain_approach"));

    // False
    // socialrules.push_back(make_pair("during say","face"));

    // V2
    // socialrules.push_back(make_pair("during say","display_text"));

    executionrules.add("approach","(not personfound)","restart_plan");
    executionrules.add("explain","(not personfound)","restart_plan");

    string main_plan = "goto_printer; HloadPaper; goto_door; HopenDoor; enter_door";

    generate("IROS15",main_plan,socialrules,executionrules);

    // H_loadPaper
    generateH("HloadPaper",socialrules,executionrules);

    // H_openDoor
    generateH("HopenDoor",socialrules,executionrules);
}





void gen_ICAPS16_example() {
    
    TSocialRules socialrules;
    TExecutionRules executionrules;

    executionrules.add("goto","(and personhere closetotarget)","skip_action");
    executionrules.add("goto","(and personhere (not closetotarget))","say_MoveAway; waitfor_freespace; restart_action");
    executionrules.add("goto","obstacle", "lookForPerson; say_MoveObstacle; waitfor_freespace; restart_action");
    executionrules.add("goto","lowbattery","goto_rechargeStation; fail_plan");
    executionrules.add("say","(not personhere)","lookForPerson*; restart_action");
    executionrules.add("lookForPerson","timeout_lookForPerson","goto_home; restart_plan");

    string main_plan = "goto_printer; say_hello; goto_home";

    generate("ICAPS16",main_plan,socialrules,executionrules);

}
#endif

void genPNP(const char* planfile, const char* erfile, const char* planname) {
    string main_plan; 

    PNPGenerator g(planname);
    g.readPlanFile(planfile, main_plan);
    g.setMainLinearPlan(main_plan);
    //g.applySocialRules(socialrules);
    g.readERFile(erfile);
    g.applyExecutionRules();
    g.save();
    
}


int main(int argc, char **argv)
{
    // gen_IROS15_example();

    // gen_ICAPS16_example();
    
    if (argc<4) {
        cout << "    Use: " << argv[0] << " <planfile> <erfile> <planname>" << endl;
        cout << "Example: " << argv[0] << " icaps16_1.plan icaps16_1.er ICAPS16" << endl;
        exit(-1);
    }
    
    genPNP(argv[1],argv[2],argv[3]);

    return 0;
}

