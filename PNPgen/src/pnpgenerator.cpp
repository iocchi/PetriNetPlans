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



Place* PNP::addPlace(string name) {
    Place* p = new Place(name);
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
    Place *p1 = addPlace("X");
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
    Place *pf = addPlace("X");
    ts->setY(p0->getY()); pe->setY(p0->getY()); // same line as p0
    te->setY(p0->getY()); pf->setY(p0->getY());
    ts->setX(p0->getX()+1);  pe->setX(p0->getX()+2); // X pos after p0
    te->setX(p0->getX()+3);  pf->setX(p0->getX()+4);
    connect(p0,ts); connect(ts,pe); connect(pe,te); connect(te,pf);
    return pf;
}

Place* PNP::addAction(string name, Place *p0) {
    Transition *ts = addTransition(name+".start");
    Place *pe = addPlace(name+".exec");
    Transition *te = addTransition(name+".end");
    Place *pf = addPlace("X");
    ts->setY(p0->getY()); pe->setY(p0->getY()); // same line as p0
    te->setY(p0->getY()); pf->setY(p0->getY());
    ts->setX(p0->getX()+1);  pe->setX(p0->getX()+2); // X pos after p0
    te->setX(p0->getX()+3);  pf->setX(p0->getX()+4);
    connect(p0,ts); connect(ts,pe); connect(pe,te); connect(te,pf);
    return pf;
}



PNPGenerator::PNPGenerator(string name) : pnp(name) {
    pinit = pnp.addPlace("init"); pinit->setInitialMarking();
}

void PNPGenerator::save() {
    stringstream ss; ss << pnp.getName() << ".pnml";
    std::ofstream of(ss.str().c_str());
    of << pnp;
    of.close();
    std::cout << "PNP '" << pnp.getName() << "' saved." << std::endl;
}


void PNPGenerator::genLinear(vector<string> v)
{
    vector<string>::iterator i = v.begin();
    Place *p = pinit;
    while (i!=v.end()) {
        string a = *i++;
        SK.push(make_pair(a,p));
        p = pnp.addAction(a,p);
    }
    p->setName("goal");
}


void PNPGenerator::genHumanAction(string say_ask, string say_do, string say_dont, string condition)
{
    Place *p0 = pnp.addAction(say_ask,pinit); SK.push(make_pair(say_ask,pinit));
    pair<Transition*,Place*> ptg = pnp.addCondition("true",p0);
    Place *p = ptg.second;

    stringstream ss; ss << "[" << condition << "]"; condition=ss.str();
    Transition* t1 = pnp.addTransition(condition); t1->setY(p->getY()-1); t1->setX(p->getX()+1);
    Transition* t2 = pnp.addTransition("[saidYes]"); t2->setY(p->getY()+0); t2->setX(p->getX()+1);
    Transition* t3 = pnp.addTransition("[saidNo]");  t3->setY(p->getY()+1); t3->setX(p->getX()+1);
    Transition* t4 = pnp.addTransition("[timeout]"); t4->setY(p->getY()+2); t4->setX(p->getX()+1);
    pnp.connect(p,t1); pnp.connect(p,t2); pnp.connect(p,t3); pnp.connect(p,t4);
    Place *py = pnp.addPlace("Y"); py->setY(t2->getY()); py->setX(t2->getX()+1);
    pnp.connect(t2,py);
    Place *pn = pnp.addPlace("N"); pn->setY(t3->getY()); pn->setX(t3->getX()+1);
    pnp.connect(t3,pn);

    Place *pd = pnp.addAction(say_do,py); SK.push(make_pair(say_do,py));
    ptg = pnp.addCondition(condition,pd);
    Place *pg = ptg.second;
    pg->setName("goal");

    Place *pf1 = pnp.addAction(say_dont,pn); SK.push(make_pair(say_dont,pn));
    ptg = pnp.addCondition("[true]",pf1);
    Place* pf2=ptg.second;
    pf2->setName("fail");

    pnp.connect(t1,pg); pnp.connect(t4,pf2);

}




void PNPGenerator::applyRules(vector<pair<string,string> > socialrules) {
    pair<string, Node*> current; Node* noplace=NULL;
    while (!SK.empty()) {
        current=SK.top(); SK.pop();
        string current_action_param = current.first;
        vector<string> tk; boost::split(tk,current_action_param,boost::is_any_of("_"));
        string current_action = tk[0];
        Node* current_place = current.second;

        // before
        vector<pair<string,string> >::iterator sit = socialrules.begin();
        while (sit!=socialrules.end()) {
            string a = sit->first, b = sit->second; sit++;
            vector<string> tk; boost::split(tk,a,boost::is_any_of(" "));
            if (tk[0]=="before" && tk[1]==current_action) {
                // add b before this action
                cout << "-- add " << b << " before " << current_action << endl;
                Node* p2 = pnp.disconnect(current_place);
                current_place->addX(-2); current_place->addY(1);
                Place* p1 = pnp.addAction(b,current_place); SK.push(make_pair(b,current_place));
                //p1->addY(-1); p1->addX(-2);
                pnp.connect(p1,p2);
                current_place=p1;
            }
        }

        // after
        sit = socialrules.end();
        while (sit!=socialrules.begin()) {
            sit--;
            string a = sit->first, b = sit->second;
            vector<string> tk; boost::split(tk,a,boost::is_any_of(" "));
            if (tk[0]=="after" && tk[1]==current_action) {
                // add b after this action
                cout << "-- add " << b << " after " << current_action << endl;

                Node* n = current_place;
                for (int k=0; k<4; k++)
                    n = pnp.next(n);

                Node* p2 = pnp.disconnect(n);
                n->addX(-2); n->addY(1);
                Place* p1 = pnp.addAction(b,n); SK.push(make_pair(b,n));
                pnp.connect(p1,p2);

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

                Node* t1 = pnp.disconnect(current_place);
                Transition* tf = pnp.addTransition("[true]"); tf->setX(current_place->getX()+1); tf->setY(current_place->getY());
                pnp.connect(current_place,tf);
                Place* p1 = pnp.addPlace("Y1"); p1->setX(tf->getX()+1); p1->setY(tf->getY());
                pnp.connect(tf,p1); pnp.connect(p1,t1); current_place=p1;
                Place* p2 = pnp.addPlace("Y2"); p2->setX(tf->getX()+1); p2->setY(tf->getY()+1);
                pnp.connect(tf,p2);
                Place* p2e = pnp.addAction(b,p2); SK.push(make_pair(b,p2));

                Transition* tj = pnp.addTransition("[true]"); tj->setX(p2e->getX()+1); tj->setY(p2e->getY());
                pnp.connect(p2e,tj);

                for (int k=0; k<3; k++)
                    t1 = pnp.next(t1);
                Node* t3 = pnp.disconnect(t1);
                pnp.connect(t1,tj);

                Place *po = pnp.addPlace("Z"); po->setX(tj->getX()+2); po->setY(tj->getY());
                pnp.connect(tj,po);

                pnp.connect(po,t3);

            }
        }


    }
}
