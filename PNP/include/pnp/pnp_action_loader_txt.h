#ifndef PNP_ACTION_LOADER_TXT
#define PNP_ACTION_LOADER_TXT

#include <fstream>
#include <string>
#include <vector>

#define PARAMS_SEPARATORS ";,"
#define COMMENT_CHAR "#"
#define FIELD_SEPARATORS " \t"

namespace PetriNetPlans {

class PnpActionLoaderTxt {
public:
	~PnpActionLoaderTxt();
	bool openFile(const std::string& filename);
	void closeFile();

	bool getParamValues(const std::string& actionClassName, std::string& actionName,
		std::vector<std::string>& params);
	bool getParamValues(const std::string& actionClassName, std::string& actionName,
		std::string& params);

protected:
	std::ifstream ifs;
};

} // namespace

#endif
