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

Place* PNP::addAction(string name, Place* p0) {
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
        p = pnp.addAction(a,p);
        //stringstream ss; ss << "S_" << a;
        SK.push(make_pair(a,p));
    }
}

void PNPGenerator::applyRules(vector<pair<string,string> > socialrules) {
    pair<string, Place*> current;
    while (!SK.empty()) {
        current=SK.top(); SK.pop();
        string current_action = current.first;
        Place* current_place = current.second;

        // before
        vector<pair<string,string> >::iterator sit = socialrules.begin();
        while (sit!=socialrules.end()) {
            string a = sit->first, b = sit->second; sit++;
            vector<string> tk; boost::split(tk,a,boost::is_any_of(" "));
            if (tk[0]=="before" && tk[1]==current_action) {
                // add b before this action
                cout << "-- add " << b << " before " << current_action << endl;
            }
        }

    }
}
