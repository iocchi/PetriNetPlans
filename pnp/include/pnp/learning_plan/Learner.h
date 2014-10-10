#ifndef learnpnp_Learner_h__guard
#define learnpnp_Learner_h__guard


#include <pnp/learning_plan/Marking.h>

#include <map>
#include <string>

 #include <fstream>
 #include <iterator>
 #include <sstream>
 #include <stdexcept> 

namespace learnpnp {

/**
*\brief A generic algorithm based on a value function
*
* This class takes care of storing and loading a V function from a file, and can be
* sub-classed to implement update rules.
* 
* The value of the states are either initialised from the aforementioned file
* or set to \p defaultV when the value is requested for the first time. A value is not
* stored until the corresponding state is visited.
*
*\author Matteo Leonetti
*/

class Learner  {

public:
	
	/**
	*\brief ctor
	*
	* Loads the value function from a file. If the file is empty or does not
	* exist, it will be created and initialised to \p defaultV
	* the first time it is requested.
	*
	*\param filePath the file to retrieve and store the value function.
	*\param defaultV the default value for the markings
	*/
	explicit Learner(const std::string &filePath, double defaultV = 0.0);

	/**
	*\brief Stores the value function in the file specified in the constructor
	*/
	virtual void saveState();

	/**
	*\brief Returns the value of a specific state
	*
	*
	*\param s the state whose value must be returned.
	*/
	virtual double valueOf(const Marking& s);

	/**
	 * \brief Returns the value of a specific state, by id
	 *
	 *
	 *\param s the state whose value must be returned.
	 *\throw std::runtime_error if there is no marking with id \p s
	 */
	virtual double valueOfId(const std::string& s) throw(std::runtime_error);

	/**
	*\brief Updates the value function according to a transition between two states
	*
	* This method first makes sure that both current and next marking have been
	* initialised, then calls updateV()
	*
	*\param current the state before the transition occurred
	*\param reward the reward accumulated since the last transition happened
	*\param next the state after the transition occurred
	*/
	void update(const Marking &current, double reward,const Marking &next);

	/**
	*\brief Performs the actual update to the value function according to a transition between two states
	*
	*It is called by update() after i has made sure that the marking used (current and next) have an initialised
	*value.
	*
	*\param current the state before the transition occurred
	*\param reward the reward accumulated since the last transition happened
	*\param next the state after the transition occurred
	*/
	virtual void updateV(const Marking &current, double reward,const Marking &next) = 0;


	/**
	*\brief Notifies the learner that one time step has passed although the marking has not changed
	*/
	virtual void tick() = 0;

	void finalize();

	/**
	*\brief Virtual dtor.
	*/
	virtual ~Learner();


/**
*\brief Performs final computations before saving the value function
**/
virtual void finalizeV() = 0;


private:
	std::map<Marking, double> V;
	std::string file;
	bool finalized;
	double defaultValue;

protected:
	void setValueOf(const Marking&, double value);
	bool isSet(const Marking&);
	std::map<Marking, double>::iterator begin();
	std::map<Marking, double>::const_iterator begin() const;

	std::map<Marking, double>::iterator end();
	std::map<Marking, double>::const_iterator end() const;

};


/**
*\brief Reads the value function from a text file
*
*\param path is the file from which the value function will be read
*\param stateMap is the map in which the V function will be stored
*
*\relates Learner
*/
template<typename State>
void readVFunctionFromTxt(const std::string &path, std::map<State, double> &stateMap) {
	std::ifstream file(path.c_str());

	if(!file.is_open()) return; //fail, leave the map empty
	
	std::string line;
	while (! file.eof() )
    {
      getline (file,line);
      size_t startpos = line.find_first_not_of(" \t"); // Find the first character position after excluding leading blank spaces  
      size_t endpos = line.find_last_not_of(" \t"); // Find the first character position from reverse af  

	  if(( std::string::npos == startpos ) || ( std::string::npos == endpos))
	  	continue; //if the string is empty skip it;
     
      line = line.substr( startpos, endpos-startpos+1 ); 

      std::pair<State, double> mapPair;
      std::istringstream str(line);

      str >> mapPair;
		  
      stateMap.insert(mapPair);

    }

//  	copy(std::istream_iterator< std::pair<State, double> >(file), 
//  		std::istream_iterator< std::pair<State, double> >(),
//  		std::inserter(stateMap, stateMap.begin()));

	file.close();
}

/**
*\brief Writes the value function to a text file
*
*\param path is the file to which the value function will be written
*\param stateMap is the map from which the V function will be extracted
*
*\relates Learner
*/
template<typename State>
void writeVFunctionToTxt(const std::string &path, std::map<State, double> &stateMap) {

	std::ofstream file(path.c_str());

	copy(stateMap.begin(), stateMap.end(), std::ostream_iterator< std::pair<State, double> >(file));


	file.close();
}

}

#endif
