#include "pnpgenerator.h"
#include <fstream>
#include <iostream>
#include <boost/algorithm/string.hpp>

int node_id=0;  // unique id of the nodes
int arc_id=0;   // unique id of the arcs

using namespace std;

static string place_color_str =
    "  <toolspecific tool=\"JARP\" version=\"1.2\">\n"
    "    <FrameColor>\n"
    "      <value>java.awt.Color[r=0,g=0,b=0]</value>\n"
    "    </FrameColor>\n"
    "    <FillColor>\n"
    "      <value>java.awt.Color[r=255,g=255,b=255]</value>\n"
    "    </FillColor>\n"
    "  </toolspecific>\n";


static string transition_color_str =
    "  <toolspecific tool=\"JARP\" version=\"1.2\">\n"
    "    <FrameColor>\n"
    "      <value>java.awt.Color[r=0,g=0,b=0]</value>\n"
    "    </FrameColor>\n"
    "    <FillColor>\n"
    "      <value>java.awt.Color[r=200,g=200,b=220]</value>\n"
    "    </FillColor>\n"
    "  </toolspecific>\n";

static string arc_color_str =
    "  <toolspecific tool=\"JARP\" version=\"1.2\">\n"
    "    <FrameColor>\n"
    "      <value>java.awt.Color[r=128,g=128,b=128]</value>\n"
    "    </FrameColor>\n"
    "    <ArrowMode>\n"
    "      <value>2</value>\n"
    "    </ArrowMode>\n"
    "  </toolspecific>\n";


std::ostream& operator<< (std::ostream& stream, const Place& place) {
    stream << "<place id=\"" << place.sid << "\">\n"
            << "  <graphics>\n"
            << "    <position x=\"" << place.posx << "\" y=\"" << place.posy << "\" />"
            << "    <size width=\"32\" height=\"32\" />"
            << "  </graphics>\n"
            << "  <name>\n"
            << "    <value>" << place.name << "</value>\n"
            << "    <graphics>\n"
            << "      <offset x=\"0\" y=\"40\" />\n"
            << "    </graphics>\n"
            << "  </name>\n"
            << "  <initialMarking>\n"
            << "    <value>" << place.marking << "</value>\n"
            << "  </initialMarking>\n"
            << place_color_str
            << "</place>\n";
    return stream;
}



std::ostream& operator<< (std::ostream& stream, const Transition& transition) {
    stream << "<transition id=\"" << transition.sid << "\">\n"
            << "  <graphics>\n"
            << "    <position x=\"" << transition.posx << "\" y=\"" << transition.posy << "\" />"
            << "    <size width=\"8\" height=\"32\" />"
            << "  </graphics>\n"
            << "  <name>\n"
            << "    <value>" << transition.name << "</value>\n"
            << "    <graphics>\n"
            << "      <offset x=\"0\" y=\"-20\" />\n"
            << "    </graphics>\n"
            << "  </name>\n"
            << transition_color_str
            << "</transition>\n";
    return stream;
}


std::ostream& operator<< (std::ostream& stream, const Arc& arc) {
    stream << "<arc id=\"" << arc.sid << "\" source=\"" << arc.source << "\" target=\"" << arc.target << "\" >\n"
            << "  <inscription>\n"
            << "    <value>1</value>\n"
            << "    <graphics>\n"
            << "      <offset x=\"0\" y=\"40\" />\n"
            << "    </graphics>\n"
            << "  </inscription>\n"
            << arc_color_str
            << "</arc>\n";
    return stream;
}

std::ostream& operator<< (std::ostream& stream, const Edge& edge) {
    stream << edge.a;
    return stream;
}

bool operator== (const Edge& e1, const Edge& e2)
{
    return (e1.n1==e2.n1 && e1.n2==e2.n2);
}

std::ostream& operator<< (std::ostream& stream, const PNP& pnp) {

    stream << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n\n"
            << "<pnml>\n"
            << "  <!--generator: PNPgenerator (Luca Iocchi 2015)-->\n"
            << "  <net id=\"n1\" type=\"PTNet\">\n"
            << "    <name>\n"
            << "      <value>" << pnp.name << "</value>\n"
            << "    </name>\n";

    vector<Place*>::const_iterator ip = pnp.P.begin();
    while (ip!=pnp.P.end()) {
        Place *p = *ip;
        stream << *p; ip++;
    }
    vector<Transition*>::const_iterator it = pnp.T.begin();
    while (it!=pnp.T.end()) {
        Transition *t = *it;
        stream << *t; it++;
    }
    vector<Edge*>::const_iterator ie = pnp.E.begin();
    while (ie!=pnp.E.end()) {
        Edge *e = *ie;
        stream << *e; ie++;
    }

    stream << "  </net>\n"
            << "</pnml>\n";
    return stream;
}

PNP::PNP(string _name) : name(_name) {
    node_id=0;  arc_id=0; P.clear(); T.clear(); E.clear(); nactions=0;
    pinit = addPlace("init"); pinit->setInitialMarking(); pinit->setX(3);
}

Place* PNP::addPlace(string name) {
    Place* p = new Place(name);
    P.push_back(p);
    return p;
}

Place* PNP::addPlace(string name, int n) {
    stringstream ss; ss << name;
    if (n==-1) ss << node_id;
    else ss << n;
    Place* p = new Place(ss.str());
    P.push_back(p);
    return p;
}


Transition* PNP::addTransition(string name) {
    Transition* t = new Transition(name);
    T.push_back(t);
    return t;
}


void PNP::connect(Node* n1, Node* n2) {
    Edge *e = new Edge(n1,n2);
    E.push_back(e);
}

void PNP::disconnect(Node* n1, Node* n2) {
    vector<Edge*>::iterator it = E.begin();
    Edge* ee = new Edge(n1,n2);
    while (it!=E.end()) {
        if (ee==*it) break;
        it++;
    }
    if (it!=E.end()) E.erase(it);
}

Node* PNP::disconnect(Node* n) {
    // search for connected Nodes
    Node* nn;
    vector<Edge*>::iterator it = E.begin();
    while (it!=E.end()) {
        Edge *e=*it;
        if (e->first()==n) {
            nn = e->second(); break;
        }
        it++;
    }
    if (it!=E.end()) E.erase(it);
    return nn;
}

Node* PNP::next(Node *n) {
    // search for connected Nodes
    Node* nn;
    vector<Edge*>::iterator it = E.begin();
    while (it!=E.end()) {
        Edge *e=*it;
        if (e->first()==n) {
            nn = e->second(); break;
        }
        it++;
    }
    return nn;
}


std::pair<Transition*,Place*> PNP::addCondition(string name, Place* p0, int dy) {
    Transition *t = addTransition(name);
    Place *p1 = addPlace("X",-1);
    t->setY(p0->getY()+dy); p1->setY(p0->getY()+dy); // same line as p0 + dy
    t->setX(p0->getX()+1);  p1->setX(p0->getX()+2);  // X pos after p0
    connect(p0,t); connect(t,p1);
    return std::make_pair(t,p1);
}

void PNP::addConditionBack(string name, Place* pfrom, Place *pto, int dy) {
    Transition *t = addTransition(name);
    t->setY(pfrom->getY()+dy); // same line as p0 + dy
    t->setX(pfrom->getX()+1);  // X pos after p0
    connect(pfrom,t); connect(t,pto);
}

Place* PNP::addAction(string name, Node *p0) {
    Transition *ts = addTransition(name+".start");
    Place *pe = addPlace(name+".exec");
    Transition *te = addTransition(name+".end");
    Place *pf = addPlace("X",-1);
    ts->setY(p0->getY()); pe->setY(p0->getY()); // same line as p0
    te->setY(p0->getY()); pf->setY(p0->getY());
    ts->setX(p0->getX()+1);  pe->setX(p0->getX()+2); // X pos after p0
    te->setX(p0->getX()+3);  pf->setX(p0->getX()+4);
    connect(p0,ts); connect(ts,pe); connect(pe,te); connect(te,pf);
    nactions++;
    return pf;
}

void PNP::connectActionToPlace(Place *pi, Place *po) { // add po after end place of action starting in pi
    Node *pe = next(next(next(next(pi)))); // end place of action
    Transition *ts = addTransition("");
    ts->setY(pe->getY()); ts->setX(pe->getX()+1); // same line as pe
    connect(pe,ts); connect(ts,po);
}

void PNP::connectPlaces(Place *pi, Place *po) { // add po after pi
    Transition *ts = addTransition(" ");
    ts->setY(pi->getY()); ts->setX(pi->getX()+1); // same line as pi
    connect(pi,ts); connect(ts,po);
}

void PNP::addInterrupt(Place *pi, string condition, Place *po) {
    Node *pe = next(next(pi)); // exec place
    string ae=pe->getName();
    std::size_t pos = ae.find(".");      // position of "." in str
    std::string a = ae.substr(0,pos);
    Transition *ts = addTransition(a+".interrupt ["+condition+"]");
    ts->setY(pe->getY()-1); ts->setX(pe->getX()); // upper line wrt pe
    connect(pe,ts); connect(ts,po);
}

Place* PNP::addAction(string name, Place *p0) {
    Transition *ts = addTransition(name+".start");
    Place *pe = addPlace(name+".exec");
    Transition *te = addTransition(name+".end");
    Place *pf = addPlace("X",-1);
    ts->setY(p0->getY()); pe->setY(p0->getY()); // same line as p0
    te->setY(p0->getY()); pf->setY(p0->getY());
    ts->setX(p0->getX()+1);  pe->setX(p0->getX()+2); // X pos after p0
    te->setX(p0->getX()+3);  pf->setX(p0->getX()+4);
    connect(p0,ts); connect(ts,pe); connect(pe,te); connect(te,pf);
    nactions++;
    //initPlace[name].push_back(p0);
    return pf;
}

Place* PNP::execPlaceOf(Place *pi) { // returns the exec place of action starting in pi
    Node *n = next(next(pi));
    return static_cast<Place *>(n);
}

Place* PNP::endPlaceOf(Place *pi) { // returns the end place of action starting in pi
    Node *n = next(next(next(next(pi))));
    return static_cast<Place *>(n);
}

std::string PNP::stats()
{
    stringstream ss;
    ss << "Actions: " << nactions << " Places: " <<
        P.size() << " Transitions: " << T.size() << " Edges: " << E.size();
    return ss.str();
}



PNPGenerator::PNPGenerator(string name) : pnp(name) {
    cout << endl << "Generation of PNP '" << name << "'" << endl;
}


void PNPGenerator::save() {
    stringstream ss; ss << pnp.getName() << ".pnml";
    std::ofstream of(ss.str().c_str());
    of << pnp;
    of.close();
    std::cout << "PNP '" << pnp.getName() << "' saved." << std::endl;
    std::cout << "PNP stats: '" << pnp.stats() << std::endl;

}

void PNPGenerator::setMainLinearPlan(string plan) {
    Place *p = genLinearPlan(pnp.pinit,plan); p->setName("goal");
}


Place *PNPGenerator::genLinearPlan(Place *pi, string plan, bool allinstack)
{
    boost::trim(plan);
    vector<string> v; boost::split(v,plan,boost::is_any_of("; "),boost::token_compress_on);

    vector<string>::iterator i = v.begin();
    Place *p = pi;
    while (i!=v.end()) {
        string a = *i++;
        if (a!="") {
            bool addstack = false;
            if (a[a.size()-1]=='*') {
                addstack = true;
                a = a.substr(0,a.size()-1);
            }
            if (allinstack || addstack) addActionToStacks(a,p);
            p = pnp.addAction(a,p);
        }
    }

    return p;
}


void PNPGenerator::genHumanAction(string say_ask, string say_do, string action_do, string say_dont, string condition)
{
    Place *p1 = pnp.addAction("findHuman",pnp.pinit); addActionToStacks("findHuman",pnp.pinit);
    Place *p2 = pnp.addAction(say_ask,p1); addActionToStacks(say_ask,p1);
    pair<Transition*,Place*> ptg = pnp.addCondition(" ",p2);
    Place *p = ptg.second;
    
    Transition* t1 = pnp.addTransition("["+condition+"]"); t1->setY(p->getY()-1); t1->setX(p->getX()+1);
    Transition* t2 = pnp.addTransition("[saidYes]"); t2->setY(p->getY()+0); t2->setX(p->getX()+1);
    Transition* t3 = pnp.addTransition("[saidNo]");  t3->setY(p->getY()+1); t3->setX(p->getX()+1);
    Transition* t4 = pnp.addTransition("[timeout]"); t4->setY(p->getY()+2); t4->setX(p->getX()+1);
    pnp.connect(p,t1); pnp.connect(p,t2); pnp.connect(p,t3); pnp.connect(p,t4);
    Place *py = pnp.addPlace("Y",-1); py->setY(t2->getY()); py->setX(t2->getX()+1);
    pnp.connect(t2,py);
    Place *pn = pnp.addPlace("N",-1); pn->setY(t3->getY()); pn->setX(t3->getX()+1);
    pnp.connect(t3,pn);

    // say_do, action_do
    Place *pd1 = pnp.addAction(say_do,py); addActionToStacks(say_do,py);
    Place *pd2 = pnp.addAction(action_do,pd1); addActionToStacks(action_do,pd1);
    ptg = pnp.addCondition("["+condition+"]",pd2);
    Place *pg = ptg.second;
    pg->setName("goal");

    // say_dont
    Place *pf1 = pnp.addAction(say_dont,pn); addActionToStacks(say_dont,pn);
    Transition* tf1 = pnp.addTransition("[not humanDetected]"); tf1->setY(pf1->getY()); tf1->setX(pf1->getX()+1); 
    pnp.connect(pf1,tf1); pnp.connect(tf1,pnp.pinit);

    pnp.connect(t1,pg); pnp.connect(t4,pnp.pinit);
    
    pnp.addInterrupt(pnp.pinit, condition, pg);

}

Place *PNPGenerator::add_before(PNP &pnp, string b, string current_action, Place* current_place) {
    // add b before this action
    cout << "-- add " << b << " before " << current_action << endl;
    Node* p2 = pnp.disconnect(current_place);
    current_place->addX(-2); current_place->addY(1);
    Place* p1 = pnp.addAction(b,current_place); addActionToStacks(b,current_place);
    //p1->addY(-1); p1->addX(-2);
    pnp.connect(p1,p2);
    return p1;
}

Place *PNPGenerator::add_after(PNP &pnp, string b, string current_action, Place* current_place) {
    // add b after this action
    cout << "-- add " << b << " after " << current_action << endl;
    Node* n = current_place;
    for (int k=0; k<4; k++)
	n = pnp.next(n);
    Node* p2 = pnp.disconnect(n);
    n->addX(-2); n->addY(1);
    Place *pn = static_cast<Place*>(n);
    Place* p1 = pnp.addAction(b,n); addActionToStacks(b,pn);
    //cout << "  ** push " << b << ","  << n->getName() << endl;
    pnp.connect(p1,p2);
    return pn;
}


void PNPGenerator::applySocialRules(TSocialRules &socialrules) {
    pair<string, Place*> current; Place* noplace=NULL;
    while (!ASS.empty()) {
        current=ASS.top(); ASS.pop();
        string current_action_param = current.first;
        vector<string> tk; boost::split(tk,current_action_param,boost::is_any_of("_"));
        string current_action = tk[0]; // current action (without parameters)
        Place* current_place = current.second; // init place of this action

        // before
        vector<pair<string,string> >::iterator sit = socialrules.begin();
        while (sit!=socialrules.end()) {
            string a = sit->first, b = sit->second; sit++;
            vector<string> tk; boost::split(tk,a,boost::is_any_of(" "));
            if (tk[0]=="before" && tk[1]==current_action) {
                current_place=add_before(pnp,b,current_action,current_place);
/*  - Execution rules
                if (b=="approach")
                    pnp.addInterrupt(current_place,"not humanDetected",pnp.pinit);
                if (b=="explain_approach")
                    pnp.addInterrupt(current_place,"not humanDetected",pnp.pinit);
*/
            }
        }

        // after
        sit = socialrules.end();
        while (sit!=socialrules.begin()) {
            sit--;
            string a = sit->first, b = sit->second;
            vector<string> tk; boost::split(tk,a,boost::is_any_of(" "));
            if (tk[0]=="after" && tk[1]==current_action) {
                Place *pp = add_after(pnp,b,current_action,current_place);

/*  - Execution rules
	       if (b=="approach")
                pnp.addInterrupt(pp,"not humanDetected",pnp.pinit);  // not humanDetected
           if (b=="explain_approach")
                    pnp.addInterrupt(current_place,"not humanDetected",pnp.pinit);
*/
            }

        }

        // during
        sit = socialrules.begin();
        while (sit!=socialrules.end()) {
            string a = sit->first, b = sit->second; sit++;
            vector<string> tk; boost::split(tk,a,boost::is_any_of(" "));
            if (tk[0]=="during" && tk[1]==current_action) {
                // add b during this action
                cout << "-- add " << b << " in parallel with " << current_action << endl;

                // search for action after place current_place
                // it may be different when after and before add actions around...

                Node* nn = current_place;
                bool found=false;
                while (!found) {
                  string naction = pnp.next(nn)->getName();
                  found = naction.substr(0,current_action.size())==current_action;
                  // cout << naction << "=" << current_action << "  " << found << endl;
                  if (!found)
                    nn = pnp.next(nn);
                }
		
                Node* t1 = pnp.disconnect(nn);
                Transition* tf = pnp.addTransition(" "); tf->setX(nn->getX()+1); tf->setY(nn->getY());
                pnp.connect(nn,tf);
                Place* p1 = pnp.addPlace("YA",-1); p1->setX(tf->getX()+1); p1->setY(tf->getY());
                pnp.connect(tf,p1); pnp.connect(p1,t1); current_place=p1;
                Place* p2 = pnp.addPlace("YB",-1); p2->setX(tf->getX()+1); p2->setY(tf->getY()+1);
                pnp.connect(tf,p2);
                Place* p2e = pnp.addAction(b,p2); addActionToStacks(b,p2); // new action (b, p2) to be analyzed

                Transition* tj = pnp.addTransition(" "); tj->setX(p2e->getX()+1); tj->setY(p2e->getY());
                pnp.connect(p2e,tj);

                for (int k=0; k<3; k++)
                    t1 = pnp.next(t1);
                Node* t3 = pnp.disconnect(t1);
                pnp.connect(t1,tj);

                Place *po = pnp.addPlace("Z",-1); po->setX(tj->getX()+2); po->setY(tj->getY());
                pnp.connect(tj,po);

                pnp.connect(po,t3);

            }
        }


    }
}

void PNPGenerator::applyExecutionRules(TExecutionRules &executionrules) {

    pair<string, Place*> current; Place* noplace=NULL;
    while (!ASE.empty()) {
        current=ASE.top(); ASE.pop();
        string current_action_param = current.first;
        vector<string> tk; boost::split(tk,current_action_param,boost::is_any_of("_"));
        string current_action = tk[0]; // current action (without parameters)
        Place* current_place = current.second; // init place of this action

        cout << "Applying execution rules for action " << current_action << endl;

        vector<TExecutionRule>::iterator eit;
        eit = executionrules.v.begin();
        while (eit!=executionrules.v.end()) {
            if (eit->action==current_action) {
                cout << "    " << eit->condition << " -> " << eit->recoveryplan << endl;

                boost::trim(eit->recoveryplan);
                vector<string> v; boost::split(v,eit->recoveryplan,boost::is_any_of("; "),boost::token_compress_on);

                Place *po = NULL; string R="";
                Place *pi = pnp.addPlace("I",-1); pi->setY(current_place->getY()-1); pi->setX(current_place->getX()+3);

                if (v.size()>1) {
                    // build actual plan without recovery specification
                    string plan = ""; int i=0;
                    for (i=0; i<v.size()-2; i++)
                        plan = plan + v[i] + ";";
                    plan = plan + v[i];

                    po = genLinearPlan(pi,plan,false); // output place of linear plan

                    R = v[v.size()-1];
                }
                else {
                    R = eit->recoveryplan;
                    po = pi;
                }

                pnp.addInterrupt(current_place,eit->condition,pi);

                if (R=="fail_plan") {
                    po->setName("fail");
                }
                else if (R=="restart_plan") {
                    pnp.connectPlaces(po,pnp.pinit);
                }
                else if (R=="restart_action") {
                    pnp.connectPlaces(po,current_place);
                }
                else if (R=="skip_action") {
                    pnp.connectPlaces(po,pnp.endPlaceOf(current_place));
                }

            }
            eit++;
        }

    }
}
