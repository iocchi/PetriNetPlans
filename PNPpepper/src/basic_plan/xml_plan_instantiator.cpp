
#include <pnp/basic_plan/xml_plan_instantiator.h>
#include <pnp/utils.h>
#include <pnp/pnp_plan.h>

#include <list>
#include <iterator>
#include <functional>
#include <set>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

using namespace PetriNetPlans;

using namespace std;

struct hideParams {
	hideParams ( const xmlDocPtr& doc, const xmlNodePtr& cur ) : doc ( doc ), cur ( cur ) {}

	hideParams ( const xmlDocPtr& doc ) : doc ( doc ), cur ( NULL ) {}

	hideParams ( const xmlNodePtr& cur ) : doc ( NULL ), cur ( cur ) {}

	xmlDocPtr doc;
	xmlNodePtr cur;
};

void XMLPnpPlanInstantiator::loadFromPNML ( const string& filePath, PnpPlan* plan )
throw ( runtime_error ) {

	xmlDocPtr doc;

	doc = xmlParseFile ( filePath.c_str() );

	xmlNodePtr cur = checkDocument ( doc, filePath ).cur;

	placesPnmlLookup.clear();
	transitionsPnmlLookup.clear();
	plan->clearAll();

	plan->setPlanName ( extractPlanNameFromPath ( filePath ) );


	cur = cur->children;

	while ( cur && xmlStrcmp ( cur->name, ( const xmlChar* ) "net" ) != 0 )
		cur = cur->next;


	if ( cur ) {
		for ( cur = cur->children; cur; cur = cur->next ) {

			if ( cur->type == XML_ELEMENT_NODE ) {
				if ( xmlStrcmp ( cur->name, ( const xmlChar* ) "place" ) == 0 ) {
					parsePlace ( hideParams ( doc, cur ), plan );
				} else
					if ( xmlStrcmp ( cur->name, ( const xmlChar* ) "transition" ) == 0 ) {
						parseTransition ( hideParams ( doc, cur ), plan );
					} else
						if ( xmlStrcmp ( cur->name, ( const xmlChar* ) "arc" ) == 0 ) {
							parseArc ( hideParams ( doc, cur ), plan );
						} else
							if ( xmlStrcmp ( cur->name, ( const xmlChar* ) "comment" ) == 0 ) {
								parseComment ( hideParams ( doc, cur ), plan );
							}
			}
		}
	}

	xmlFreeDoc ( doc );

	// resolve implicit function names
	PNP_OUT ( "Expliciting function names:" );

	for ( set<PnpTransition*>::iterator itt = plan->transitions.begin();
	        itt != plan->transitions.end(); ++itt ) {

		PnpTransition& t = **itt;

		PNP_OUT ( "  Transition "<<t.nodeId<<" \""<<t.pnmlString<<"\"" );

		vector<string> explicitedFunctions;

		for ( vector<string>::iterator itf = t.functionsToCall.begin();
		        itf != t.functionsToCall.end(); ) {


			string& f = *itf;
			PNP_OUT ( "    Function '"<<f<<"'" );

			if ( f[0] == '*' && f.size() > 2 ) {
				string fn = f.substr ( 2 );

				bool lookFanOut = false;
				bool atLeastOne = false;

				if ( fn == "start" || fn == "resume" )
					lookFanOut = true;

				pair<multimap<PnpTransition*, EdgePlace<PnpPlace> >::iterator,
				multimap<PnpTransition*, EdgePlace<PnpPlace> >::iterator >
				boundaries;

				if ( lookFanOut )
					boundaries = plan->outputPlaces.equal_range ( *itt );
				else
					boundaries = plan->inputPlaces.equal_range ( *itt );

				for ( multimap<PnpTransition*, EdgePlace<PnpPlace> >::iterator
				        itp = boundaries.first;
				        itp != boundaries.second; ++itp ) {


					string placeFn = itp->second.p->functionToCall;

					if ( placeFn != "" ) {
						string act = placeFn.substr ( 0, placeFn.find_first_of ( "." ) );
						string newFn = act + "." + fn;
						explicitedFunctions.push_back ( newFn );
						PNP_OUT ( "      Explicited in '"<<newFn<<"'" );
						atLeastOne = true;
					}
				}

				if ( !atLeastOne ) {
					PNP_ERR ( "I cannot explicit action function in transition "<<t.nodeId<<" \""
					          <<t.pnmlString<<"\"" );
				}

				itf = t.functionsToCall.erase ( itf );
			}

			else
				++itf;

		}

		t.functionsToCall.insert ( t.functionsToCall.end(),

		                           explicitedFunctions.begin(), explicitedFunctions.end() );

		// adding [action.finished()] to end transitions and [action.failed] to fail transitions

		for ( vector<string>::iterator itf = t.functionsToCall.begin();
		        itf != t.functionsToCall.end(); ++itf ) {
			string actName = itf->substr ( 0, itf->find_last_of ( "." ) );
			string fnName = itf->substr ( itf->find_last_of ( "." ) +1 );
			PNP_OUT ( "    Action '"<<actName<<"', function '"<<fnName<<"'" );

			if ( fnName == "end" ) {
				PNP_OUT ( "      Guard condition '"<<t.guardCondition<<"'" );

				if ( t.guardCondition == "" )
					t.guardCondition = actName + ".finished";

				PNP_OUT ( "      New guard condition '"<<t.guardCondition<<"'" );
			} else
				if ( fnName == "fail" ) {
					PNP_OUT ( "      Guard condition '"<<t.guardCondition<<"'" );

					if ( t.guardCondition == "" )
						t.guardCondition = actName + ".failed ";

					PNP_OUT ( "      New guard condition '"<<t.guardCondition<<"'" );
				}
		}
	}

//        for (set<string>::iterator it = symbolNames.begin(); it != symbolNames.end(); ++it) {
//          PNP_OUT("Symbol (action or plan): "<<*it);
//        }

}


std::string XMLPnpPlanInstantiator::extractPlanNameFromPath ( const std::string& filePath ) {

	string::size_type a = filePath.find_last_of ( "/\\" );
	string n;

	if ( a != string::npos )
		n = filePath.substr ( a+1 );
	else
		n = filePath;

	n = n.substr ( 0, n.find_first_of ( "." ) );

	return n;

}

hideParams XMLPnpPlanInstantiator::checkDocument ( const hideParams &pars, const string& filePath ) {
	xmlDocPtr document = pars.doc;

	if ( !document ) {
		PNP_OUT ( "Document '" << filePath << "' not parsed successfully." );
		throw runtime_error ( "Document '" + filePath + "' not parsed successfully." );
	}

	xmlNodePtr current = xmlDocGetRootElement ( document );

	if ( !current ) {
		PNP_OUT ( "Document '" << filePath << "' is empty" );
		xmlFreeDoc ( document );
		throw runtime_error ( "Document '" + filePath + "' is empty" );
	}

	if ( xmlStrcmp ( current->name, ( const xmlChar* ) "pnml" ) != 0 ) {
		PNP_OUT ( "Document is not a pnml file." );
		xmlFreeDoc ( document );
		throw runtime_error ( "Document is not a pnml file." );
	}

	return current;
}




xmlNodePtr getFirstChildByName ( xmlNodePtr cur, const char* name ) {
	for ( cur = cur->children; cur; cur = cur->next ) {
		if ( xmlStrcmp ( cur->name, ( const xmlChar* ) name ) == 0 )
			break;
	}

	return cur;
}

string getFirstChildContentByName ( xmlDocPtr doc, xmlNodePtr cur, const char* name ) {
	string ret = "";
	xmlNodePtr nodePtr = getFirstChildByName ( cur, name );

	if ( nodePtr ) {
		xmlChar* text = xmlNodeListGetString ( doc, nodePtr->children, 1 );
		ret = string ( ( const char* ) text );
		xmlFree ( text );
	}

	return ret;
}


void XMLPnpPlanInstantiator::parsePlace ( const hideParams& pars, PnpPlan *plan ) {

	xmlDocPtr doc = pars.doc;
	xmlNodePtr cur = pars.cur;

	set<string> symbolNames; //FIXME MATTEO check what the hell this guy is needed for.
	string id;
	xmlChar* xid = xmlGetProp ( cur, ( const xmlChar* ) "id" );
	PNP_OUT ( "Place, id: " << xid );
	id = string ( ( const char* ) xid );
	xmlFree ( xid );
	string placeString = "";
	int initialMarking = 0;

	for ( xmlNodePtr cur2 = cur->children; cur2; cur2 = cur2->next ) {
		if ( xmlStrcmp ( cur2->name, ( const xmlChar* ) "name" ) == 0 ) {
			placeString = getFirstChildContentByName ( doc, cur2, "value" );
			PNP_OUT ( "  String: '" << placeString<<"'" );
		} else
			if ( xmlStrcmp ( cur2->name, ( const xmlChar* ) "initialMarking" ) == 0 ) {
				string v = getFirstChildContentByName ( doc, cur2, "value" );
				vector<string> vv = tokenize ( v, "," );

				if ( vv.size() == 1 )
					v = vv[0];
				else
					if ( vv.size() == 2 )
						v = vv[1];

				PNP_OUT ( "  Initial marking: '" << v<<"'" );

				initialMarking = atoi ( v.c_str() );
			}
	}

	string functionToCall;

	int goalMarking, failMarking;
	parsePlaceString ( placeString, functionToCall, goalMarking, failMarking );

	PNP_OUT ( "  Parsed: function = '"<<functionToCall<<"', goalMarking = "
	          <<goalMarking<<", failMarking = "<<failMarking );
	PnpPlace* p = new PnpPlace ( id, functionToCall, initialMarking, goalMarking, failMarking );
	p->pnmlString = placeString;
	placesPnmlLookup.insert ( make_pair ( id, p ) );
	plan->places.insert ( p );

	if ( functionToCall != "" ) {
		//  vector<string> v;	// = tokenize(functionToCall, ".");
		size_t q = functionToCall.find_last_of ( "." );

		if ( q != string::npos ) {
			symbolNames.insert ( functionToCall.substr ( 0, q ) );
		}

//    if (v.size() == 2 && v[0] != "*") symbolNames.insert(v[0]);
	}
}


void XMLPnpPlanInstantiator::parseComment ( const hideParams& pars, PnpPlan *plan ) {

	xmlDocPtr doc = pars.doc;
	xmlNodePtr cur = pars.cur;

	string id;
	xmlChar* xid = xmlGetProp ( cur, ( const xmlChar* ) "id" );
	PNP_OUT ( "Comment, id: " << xid );
	id = string ( ( const char* ) xid );
	xmlFree ( xid );
	xmlNodePtr textNodePtr = getFirstChildByName ( cur, "text" );
	string transitionString = "";

	if ( textNodePtr ) {
		// XXX mi sfugge perch�ci siano pi nodi "text" di cui solo uno contiene un value
		// ma probabilmente ho scritto una scemenza da qualche parte nelle righe qui intorno
		// in ogni caso, funziona, quindi ok ;-)
		for ( xmlNodePtr cur2 = textNodePtr; cur2; cur2 = cur2->next ) {
			if ( xmlStrcmp ( cur2->name, ( const xmlChar* ) "text" ) == 0 ) {
				string value = getFirstChildContentByName ( doc, cur2, "value" );

				if ( value != "" ) {
					PNP_OUT ( "  Text value: '" << value << "'" );

					if ( value.substr ( 0, 10 ) == "PLAN NAME:" ) {
						string planName = value.substr ( 10 );
						planName = trim ( planName );
						plan->setPlanName ( planName );
						PNP_OUT ( "  Plan name: '" << planName<<"'" );
					}
				}
			}
		}
	}
}


void XMLPnpPlanInstantiator::parseTransition ( const hideParams& pars, PnpPlan* plan ) {
	xmlDocPtr doc = pars.doc;
	xmlNodePtr cur = pars.cur;

	set<string> symbolNames; //FIXME MATTEO I still don't know what this is here for.
	string id;
	xmlChar* xid = xmlGetProp ( cur, ( const xmlChar* ) "id" );
	PNP_OUT ( "Transition, id: " << xid );
	id = string ( ( const char* ) xid );
	xmlFree ( xid );
	xmlNodePtr nameNodePtr = getFirstChildByName ( cur, "name" );
	string transitionString = "";

	if ( nameNodePtr ) {
		transitionString = getFirstChildContentByName ( doc, nameNodePtr, "value" );
		PNP_OUT ( "  Transition string: '" << transitionString<<"'" );
	}

	string guardCondition;

	vector<string> functionsToCall;

	parseTransitionString ( transitionString, guardCondition, functionsToCall );
	PNP_OUT ( "  Parsed: guard = '"<<guardCondition<<"'" );

	for ( size_t i = 0; i < functionsToCall.size(); i++ ) {
		PNP_OUT ( "    Function to call '"<<functionsToCall[i]<<"'" );
	}

	PnpTransition* t = new PnpTransition ( id, functionsToCall, guardCondition );

	t->pnmlString = transitionString;
	transitionsPnmlLookup.insert ( make_pair ( id, t ) );
	plan->transitions.insert ( t );

	for ( size_t i = 0; i < functionsToCall.size(); i++ ) {
		string& functionToCall = functionsToCall[i];
		//vector<string> v;// = tokenize(functionToCall, ".");
		size_t q = functionToCall.find_last_of ( "." );

		if ( q != string::npos ) {
			symbolNames.insert ( functionToCall.substr ( 0, q ) );
		}

		//if (v.size() == 2 && v[0] != "*") symbolNames.insert(v[0]);
	}
}


void XMLPnpPlanInstantiator::parseArc ( const hideParams& pars, PnpPlan *plan ) {
	xmlDocPtr doc = pars.doc;
	xmlNodePtr cur = pars.cur;

	string id, srcid, targid;
	xmlChar* xid = xmlGetProp ( cur, ( const xmlChar* ) "id" );
	xmlChar* xsrc = xmlGetProp ( cur, ( const xmlChar* ) "source" );
	xmlChar* xtarg = xmlGetProp ( cur, ( const xmlChar* ) "target" );
	PNP_OUT ( "Arc, id: " << xid << ", Source: " << xsrc << ", Target: " << xtarg );
	id = string ( ( const char* ) xid );
	srcid = string ( ( const char* ) xsrc );
	targid = string ( ( const char* ) xtarg );
	xmlFree ( xid );
	xmlFree ( xsrc );
	xmlFree ( xtarg );
	xmlNodePtr inscriptionNode = getFirstChildByName ( cur, "inscription" );
	int arcValue = 1;

	if ( inscriptionNode ) {
		string value = getFirstChildContentByName ( doc, inscriptionNode, "value" );
		size_t q = value.find_first_of ( "," );

		if ( q != string::npos )
			value = value.substr ( q+1 );

		PNP_OUT ( "  Arc value: " << value );

		arcValue = atoi ( value.c_str() );
	}

	if ( placesPnmlLookup.find ( srcid ) != placesPnmlLookup.end()
	        && transitionsPnmlLookup.find ( targid ) != transitionsPnmlLookup.end() ) {
		// arc from Place to Transition
		plan->addEdge ( placesPnmlLookup[srcid], transitionsPnmlLookup[targid], arcValue );
	} else
		if ( transitionsPnmlLookup.find ( srcid ) != transitionsPnmlLookup.end()
		        && placesPnmlLookup.find ( targid ) != placesPnmlLookup.end() ) {
			// arc from Transition to Place
			plan->addEdge ( transitionsPnmlLookup[srcid], placesPnmlLookup[targid], arcValue );
		} else {
			PNP_OUT ( "  Malformed arc" );
		}
}

XMLPnpPlanInstantiator::~XMLPnpPlanInstantiator() {}


