#include <pnp/learning_plan/algo/LoggingController.h>

#include <pnp/learning_plan/Learner.h>

#include <iostream>
#include <fstream>

using namespace std;

namespace learnpnp {
  
   PlaceLogger::PlaceLogger(const std::string& filepath, const std::set<std::string>& places) :
	  filepath(filepath), places(places){}
  
  void PlaceLogger::initialize() {
	std::ofstream file(filepath.c_str());

	file << "#";
	
	set<string>::iterator plIt = places.begin();
	  
	  for(bool first=true; plIt != places.end(); ++plIt, first = false) {
		file << ((first)?"":"\t") << *plIt;
	  }
	  
	file << endl;
	file.close();
  }
  
  void PlaceLogger::afterUpdate(const Marking &current) {
	if( places.find(current.getId()) != places.end() &&	
	    markings.find(current.getId()) == markings.end())
	  markings.insert(make_pair(current.getId(),current));
  }
  
  void PlaceLogger::saving(Learner *V) {

	  std::ofstream file(filepath.c_str(), ios_base::app);

	  set<string>::iterator plIt = places.begin();
	  
	  for(bool first = true; plIt != places.end(); ++plIt, first = false) {
		
		if(markings.find(*plIt) != markings.end()) {
			double value = V->valueOf(markings[*plIt]);
		  file << ((first)?"":"\t") << value;
		}
		else
		  file << ((first)?"":"\t") << "?";
	  }
	  
	  file << endl;

 	  file.close();
  }
  
}
