#include <pnp/pnp_action_loader_txt.h>
#include <pnp/utils.h>

namespace PetriNetPlans {

using namespace std;

bool PnpActionLoaderTxt::openFile(const string& filename)
{
	ifs.open(filename.c_str(), ios::in);
	return ifs.good();
}

void PnpActionLoaderTxt::closeFile()
{
	ifs.close();
}

bool PnpActionLoaderTxt::getParamValues(const string& actionName, string& actionClassName,
		vector<string>& params)
{
	string pp;
	if (!getParamValues(actionName, actionClassName, pp)) return false;
	else {
		params = tokenizeWithQuotes(pp, PARAMS_SEPARATORS);
		return true;
	}
}

PnpActionLoaderTxt::~PnpActionLoaderTxt()
{
	//PNP_OUT("Someone's deleting actionLoader");
}

bool PnpActionLoaderTxt::getParamValues(const string& actionName, string& actionClassName,
		string& params)
{
	ifs.clear();
	ifs.seekg(0, ios::beg);
	if (!ifs.good()) {
		PNP_OUT("Actions file not open!");
		return false;
	}
	string line;
	while (getline(ifs, line)) {
		line = trim(line);
		string::size_type a = line.find_first_of(FIELD_SEPARATORS);
		string aname = line.substr(0, a);
		if (aname == actionName) {
			string::size_type b = line.find_first_not_of(FIELD_SEPARATORS, a);
			string::size_type c = line.find_first_of(FIELD_SEPARATORS, b);
			actionClassName = line.substr(b, c-b);
			string::size_type d = line.find_first_not_of(FIELD_SEPARATORS, c+1);
			params = "";
			if (c != string::npos && d != string::npos) params = line.substr(d);
			return true;
		}
	}
	return false;
}

} // namespace
