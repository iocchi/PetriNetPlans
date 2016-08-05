#ifndef PNPGENERATOR_H
#define PNPGENERATOR_H

#include <string>
#include <sstream>
#include <vector>
#include <stack>
#include <map>
#include "policy.h"
#include "conditionalplan.h"

using std::string;
using std::stringstream;
using std::vector;
using std::stack;
using std::pair;
using std::map;

static int dx=60, dy=100, offx=20, offy=200;  // draw parameters

extern int node_id;  // unique id of the nodes
extern int arc_id;   // unique id of the arcs


class Node {
protected:


    int X,Y; // virtual position
    int posx,posy; // real position

    string name; // name of the node
    string sid; // string version of the id

public:
    Node(string _name) : name(_name) {
        stringstream ss; ss << "n" << node_id; sid = ss.str();
        X=node_id; Y=0; setX(X); setY(Y); node_id++;
    }

    int getX() {
       return X;
    }

    void addX(int dx) {
        setX(X+dx);
    }

    void setX(int x) {
        X=x; posx = offx + X * dx;
    }

    int getY() {
        return Y;
    }

    void addY(int dy) {
        setY(Y+dy);
    }


    void setY(int y) {
        Y=y; posy = offy + Y * dy;
    }

    string getID() {
        return sid;
    }

    void setName(string _name) {
        name=_name;
    }
    
    string getName() { return name; }
};


class Place : public Node {
    int marking;
public:
    Place(string _name) : Node(_name), marking(0) { }

    void setInitialMarking() {
        marking=1;
    }

    friend std::ostream& operator<< (std::ostream& stream, const Place& place);

};

class Transition : public Node {
public:
    Transition(string _name) : Node(_name) { }

    friend std::ostream& operator<< (std::ostream& stream, const Transition& transition);
};


class Arc {

    string sid, source, target;
public:
    Arc(string _source, string _target) {
        stringstream ss; ss << "a" << arc_id; sid = ss.str(); arc_id++;
        source=_source; target=_target;
    }

    string getID() {
        return sid;
    }

    friend std::ostream& operator<< (std::ostream& stream, const Arc& arc);
};

class Edge {
    Node *n1, *n2;
    Arc a;
public:

    Edge(Node* _n1, Node* _n2) : n1(_n1), n2(_n2), a(n1->getID(),n2->getID()) {
    }

    Node* first() { return n1; }
    Node* second() { return n2; }

    friend bool operator== (const Edge& e1, const Edge& e2);
    friend std::ostream& operator<< (std::ostream& stream, const Edge& edge);
};

class PNP {

protected:
    string name;  // name of the plan
    vector<Place*> P;
    vector<Transition*> T;
    vector<Edge*> E;
    int nactions;
    // ??? map<string, vector<Place *> > initPlace; // initial place for each instance of an action    

public:

    Place *pinit; // init place of the plan
    map<string, Place*> timed_action_wait_exec_place; // exec place of corresponding wait action for timed actions
    map<string, Place*> timed_action_fork_place; // fork place of timed actions

    PNP(string _name);
    string getName() { return name; }

    Node *next(Node *n);
    void connect(Node* n1, Node* n2);
    void disconnect(Node* n1, Node* n2);
    Node* disconnect(Node* p);

    Place* addPlace(string name);
    Place* addPlace(string name, int n); // n=-1: use node_id
    Transition* addTransition(string name);

    std::pair<Transition*,Place*> addCondition(string name, Place* p0, int dy=0);
    void addConditionBack(string name, Place* pfrom, Place *pto, int dy=0);
    Place* addAction(string name, Place* p0);
    Place* addAction(string name, Node* p0);

    // sensing action with multiple outcomes
    std::vector<Place*> addSensingAction(string name, Place* p0, vector<string> outcomes);

    Place* addTimedAction(string name, Place *p0, int timevalue, Place **p0action);
    Transition* addInterrupt(Place *pi, string condition, Place *po);
    Transition* addFail(Place *pi, Place *po);
    void connectActionToPlace(Place *pi, Place *po); // connect the action and the place with an empty transition
    void connectPlaces(Place *pi, Place *po); // connect the two places with an empty transition

    Place* execPlaceOf(Place *pi); // returns the exec place of action starting in pi
    Place* endPlaceOf(Place *pi); // returns the end place of action starting in pi

    std::string stats();

    friend std::ostream& operator<< (std::ostream& stream, const PNP& pnp);
};

struct TExecutionRule
{
    string action;
    string condition;
    string recoveryplan;

    TExecutionRule(string a, string c, string r) : action(a), condition(c), recoveryplan(r) { }
};

struct TExecutionRules {
    vector<TExecutionRule> v;

    void add(string action, string condition, string recoveryplan) {
        v.push_back(TExecutionRule(action,condition,recoveryplan));
    }
};

typedef vector<pair<string,string> > TSocialRules;


class PNPGenerator
{
private:

    stack< pair<string, Place*> > ASS;  // action, Place* map - stack of actions to be analized for applying the social rules
    stack< pair<string, Place*> > ASE;  // action, Place* map - stack of actions to be analized for applying the execution rules

    void addActionToStacks(string a, Place *p) {
        ASS.push(make_pair(a,p)); ASE.push(make_pair(a,p));
    }
    
    bool parseERline(const string line, string &action, string &cond, string &plan);

public:
    PNP pnp;
    TSocialRules socialrules;
    TExecutionRules executionrules;

    PNPGenerator(string name);

    Place *genLinearPlan(Place *pi, string plan, bool allinstack=true); // string format = action1; action2; ...; actionn - returns output place
                                                                        // allinstack = true -> all actions are added in the stack for further processing
                                                                        // allinstack = false -> only actions trerminating with '*' are added in the stack for further processing

    // Generation of PNP
    bool genFromPolicy(Policy &p);
    bool genFromConditionalPlan(ConditionalPlan &plan);
    bool genFromConditionalPlan_r(ConditionalPlan *plan, Place *place);


    void setMainLinearPlan(string plan); // set this plan as main plan for this generation
    void readPlanFile(const char*filename, string &plan);
    void readPlanFile(const string& filename, string &plan) {
        readPlanFile(filename.c_str(), plan);
    }

    void genHumanAction(string say_ask, string say_do, string action_do, string say_dont, string condition);
    void applySocialRules();
    void applyExecutionRules();
        
    void readERFile(const char* filename);
    void readERFile(const string& filename) { readERFile(filename.c_str()); }

    Place * add_before(PNP &pnp, string b, string current_action, Place* current_place);
    Place * add_after(PNP &pnp, string b, string current_action, Place* current_place);
    
    Place* addAction(string action, Place *place) {
        addActionToStacks(action,place);
        return pnp.addAction(action,place);
    }

    vector<Place*> addSensingAction(string action, Place *place, vector<string> outcomes) {
        addActionToStacks(action,place);
        return pnp.addSensingAction(action,place,outcomes);
    }

    void save(const char* filename=NULL); // if NULL it uses the name of the plan as file name

};

#endif // PNPGENERATOR_H
