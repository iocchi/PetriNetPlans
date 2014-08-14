#ifndef PetriNetPlans_XMLPnpPlanInstantiator_guard
#define PetriNetPlans_XMLPnpPlanInstantiator_guard


#include <string>
#include <stdexcept>
#include <map>

#include "basic_plan.h"

//I use this declaration, defined  in the .cpp,
 //to remove the includes of libxml from this file.
 struct hideParams;

namespace PetriNetPlans {

	/**
	 * \brief A xml plan loader for basic PnpPlans
	 *
	 * This class parses .pnml files to fill plan objects. Just instantiate the class
	 * and call loadFromPNML().
	 * */
  class XMLPnpPlanInstantiator {
    public:

	/**
	* \brief Loads the plan from a file
	*
	* \param filePath the path of the .pnml file to load
	* \param plan the plan object that will receive the net specified in
	* 				\p filePath
	* \exception std::runtime_error thrown whenever the parsing fails
	*/
      void loadFromPNML(const std::string& filePath,  PnpPlan* plan) throw(std::runtime_error);

	  /**
	  *\brief dtor
	  */
      virtual ~XMLPnpPlanInstantiator();

    private:

      void parseArc(const hideParams&, PnpPlan*);
      void parseComment(const hideParams&, PnpPlan*);
      void parseTransition(const hideParams&, PnpPlan*);
      void parsePlace(const hideParams&, PnpPlan*);
      std::map<std::string, PnpPlace*> placesPnmlLookup;
      std::map<std::string, PnpTransition*> transitionsPnmlLookup;
      
      std::string extractPlanNameFromPath(const std::string& filePath);
      hideParams checkDocument(const hideParams &, const std::string& filePath);

  };
}

#endif
