#include <pnp/utils.h>
#include <cstdlib>
#include <cstdio>

namespace PetriNetPlans {

using namespace std;

vector<string> tokenize(string s, const string& seps)
{
	vector<string> r;
	while (s.length()) {
		string::size_type a = s.find_first_of(seps);
		if (a != string::npos) {
			r.push_back(s.substr(0, a));
			s = s.substr(s.find_first_not_of(seps, a));
		}
		else {
			r.push_back(s);
			s = "";
		}
	}
	return r;
}

vector<string> tokenizeWithQuotes(string stt, const string& seps)
{
	vector<string> toks;
	string s;
	int pars = 0;
	for (size_t i = 0; i < stt.size(); i++) {
		char c = stt[i];
		if (c == '"') {
			if (pars == 0) pars = 1;
			else pars = 0;
		}
		// XXX odio le std::string
		string cs = "";
		cs += c;
		if (!pars && seps.find_first_of(cs) != string::npos) {
			toks.push_back(s);
			s = "";
		}
		if (pars || seps.find_first_of(cs) == string::npos) s += cs;
	}
	if (s != "") toks.push_back(s);
	return toks;
}

string trim(const string& s)
{
	if (s == "") return "";
	string r = s.substr(s.find_first_not_of(" "));
	r = r.substr(0, r.find_last_not_of(" ") + 1);
	return r;
}

vector<string> tokenize2(string stt)
{
	vector<string> toks;
	string s;
	int pars = 0;
	for (size_t i = 0; i < stt.size(); i++) {
		char c = stt[i];
		if (c == '(' || c == '[') pars++;
		else if (c == ')' || c == ']') pars--;
		if (pars < 0) {
			PNP_ERR("Malformed Place string: '"<<stt<<"'\n");
			return vector<string>();
		}
		if (!pars && c == ' ' && s!="") {//ML adding '&& s!="", same at the end
			toks.push_back(s);
			s = "";
		}
		if (pars || c != ' ') s += c; 
	}
 	if(s!="")
		toks.push_back(s);
	return toks;
}

// format: <actionName>.{"ex"|"exec"} or anything else (e.g. "P1"),
// can be followed by "goal" or "goal(n)" or "fail" or "fail(n)"

// old legged format: ex.<actionName>[.goal[(n)]] or anything not starting with "ex."

bool parsePlaceString(const string& PlaceString, string& functionToCall, int& goalMarking, int& failMarking)
{
	functionToCall = "";	// no function to call
	goalMarking = -1;		// don't care
	failMarking = -1;		// don't care

	vector<string> toks = tokenize2(PlaceString);
/*	for (size_t i = 0; i < toks.size(); i++) {
		printf("..%s", toks[i].c_str());
	}
	printf("\n");*/

	bool functionFound = false;
	for (size_t i = 0; i < toks.size(); i++) {
		vector<string> v = tokenize(toks[i], ".");
		if (v.size() > 1) {
			size_t q = toks[i].find_last_of(".");
			if (q != string::npos) {
				v.clear();
				v.push_back(toks[i].substr(0, q));
				v.push_back(toks[i].substr(q+1));
			}
		}
		if (v.size() > 0 && v[0] == "ex") {
			// old legged format
			functionToCall = v[1] + "." + "exec";
			if (v.size() == 3) {	// goal?
				if (v[2].substr(0, 4) == "goal") {
					if (v[2].size() == 4) goalMarking = 1;
					else {
						if (v[2][4] != '(' || v[2][v.size()-1] != ')') {
							PNP_ERR("Malformed Place string (goal): '"<<PlaceString<<"'\n");
							return false;
						}
						goalMarking = atoi(v[2].substr(5, v[2].size()-6).c_str());
					}
				}
			}
		}
		else if (v.size() > 1 && (v[1] == "ex" || v[1] == "exec")) {
			if (functionFound) {
				PNP_ERR("We can have only one function call in place strings (for now ;-))\n");
			}
			else {
				if (v[1] == "ex") v[1] = "exec";
				functionToCall = v[0] + "." + v[1];
				functionFound = true;
			}
		}
		if (toks[i].substr(0, 4) == "goal") {
			if (toks[i].size() == 4) goalMarking = 1;
			else {
				if (toks[i][4] != '(' || toks[i][toks[i].size()-1] != ')') {
					PNP_ERR("Malformed Place string (goal): '"<<PlaceString<<"'\n");
					return false;
				}
				goalMarking = atoi(toks[i].substr(5, toks[i].size()-6).c_str());
			}
		}
		else if (toks[i].substr(0, 4) == "fail") {
			if (toks[i].size() == 4) failMarking = 1;
			else {
				if (toks[i][4] != '(' || toks[i][toks[i].size()-1] != ')') {
					PNP_ERR("Malformed Place string (fail): '"<<PlaceString<<"'\n");
					return false;
				}
				failMarking = atoi(toks[i].substr(5, toks[i].size()-6).c_str());
			}
		}
	}
	return true;
}


// legged format: {"start"|"end"|"resume"|"res"|"interrupt"|"interr"}.<actionName>[.<condition>] or T1, T2, etc.
// rescue format: {"start"|"end"|"resume"|"interrupt"}.<actionName>\[<condition>\] or anything else

// condition legged format: <condition>
//	<condition> ::= [<predicate>|"("<not><condition>")"]
//	<
// condition rescue format: "["<condition>"]"
//    <not> ::= "not" | "!"
//    <and> ::= "and" | "&&"
//    <or>  ::= "or | "||"
//    <condition> ::= [<not> <condition> | <condition>]

string normalizeTransitionFunction(const string& s)
{
	if (s == "start") return "start";
	if (s == "resume" || s == "res") return "resume";
	if (s == "end") return "end";
	if (s == "interrupt" || s == "interr") return "interrupt";
	if (s == "fail") return "fail";
	return "";
}

bool parseTransitionString(const string& transitionString, string& guardCondition,
	vector<string>& functionsToCall)
{
	vector<string> toks = tokenize2(transitionString);
	functionsToCall.clear();	// non si sa mai

	bool guardFound = false;
	for (size_t i = 0; i < toks.size(); i++) {
		//printf("Token '%s'\n", toks[i].c_str());
		string ntransfunc = normalizeTransitionFunction(toks[i]);
		if (ntransfunc != "") functionsToCall.push_back("*." + ntransfunc);
		else {
			if (toks[i][0] == '[') {
				// it is the guard condition
				if (guardFound) {
					PNP_ERR("You cannot put more than one guard on the same Transition '"<<transitionString<<"'\n");
					return false;
				}
				else {
					guardCondition = toks[i].substr(1, toks[i].size() - 2);
					guardFound = true;
				}
			}
			else {
				size_t q = toks[i].find_last_of(".");
				if (q == string::npos) continue; //ML: was return true.
				/*With return true it failed if there was a token that didn't have a function
				 call, such as an empty string. For instance if the transition was
				 Action.start  [Condition]
				 it was parsed into 'Action.start' '' '[Condition]
				 The second token is an empty string, and the 'return true' made the function
				 terminate here, without parsing the condition.
				 I've also changed tokenize2() to avoid empty strings, but I still think this can
				 be useful with labels that are not actions, such as in
				 T [Condition]
				 that would be parsed in 'T' '[Condition]' and T would just be ignored, while the
				 condition still used.*/
				string first = toks[i].substr(0, q);
				string second = toks[i].substr(q + 1);
				ntransfunc = normalizeTransitionFunction(second);
				if (ntransfunc != "") {
					functionsToCall.push_back(first + "." + ntransfunc);
				}
				else {
					PNP_ERR("Unknown function '" << second << "'\n");
					return false;
				}
			}
#if 0
			else if (v[0] == "start" || v[0] == "end") {
				// old legged format
				if (v.size() < 2) {
					PNP_ERR("Malformed Transition string '"<<transitionString<<"' (token '"<<toks[i]<<"')\n");
					return false;
				}
				else {
					functionsToCall.push_back(v[1] + "." + v[0]);
					if (v.size() == 3) {
						// there is also the guard condition
						guardCondition = v[2];
					}
				}
			}
#endif
		}
	}
	return true;
}

} // namespace
