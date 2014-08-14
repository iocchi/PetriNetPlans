#ifndef learnpnp_Marking_h__guard
#define learnpnp_Marking_h__guard

#include <vector>
#include <map>
#include <string>
#include <iostream>


namespace learnpnp {

/**
*\brief A state of a Petri Net
*
*\author Matteo Leonetti
*/
class Marking {

public:
	/**
	*\brief Initializes an empty marking
	*
	*An empty marking is one with zero states.
	*/
	Marking();
// 	static int count1;
// 	static int count2;
	/**
	*\brief Initializes the marking from a map
	*
	*The map must have the id of the state as the key and the number
	*of tokens in the state as the value. The states are identified by
	*their position in the map. Recall that iterators of std::map traverse
	*the structure in the order imposed by the keys.
	*
	*\param inmap is the map from which initialize the marking
	*/
	Marking (const std::map<std::string, int>& inmap);
	
	/**
	*\brief Initializes the marking from a vector
	*
	*The states are identified by their position in the vector. Each element
	*of the vector is the number of tokens for that state.
	*
	*\param vec is the vector from which initialize the marking
	*\param id is an id you can associate to the marking to recognize it in
	*the file in which it is stored.
	*/
	Marking (const std::vector<int>& vec, std::string id = "unknown");
	
	/**
	*\brief Returns the id of this marking
	*
	*\return the id of this marking
	*/
	std::string getId() const;

private:
	std::vector<int> marking;
	std::string id;
	//position of the first place with non-zero marking
	int lastNonZero; //this variable speeds up operator< which profiling revealed quite time consuming

	int findLastNonZero();
	
	friend bool operator<(const Marking& a,const  Marking& b	);
	friend std::ostream &operator<<(std::ostream& stream, const std::pair<Marking, double>& markPair);
	friend bool operator ==(const Marking& a, const Marking& b);
	friend bool operator !=(const Marking& a,const Marking& b);
};

/**
*\brief Imposes a total order upon markings
*
*\p a < \p b iff \p a has less states than \p b or they have the same
*number of states and there exists a state \p s such that all the states
*that precedes \p s in the internal ordering of the markings are less then
*or equal to their corresponding state in \p b and \p s is strictly less then
*it corresponding state in \p b.
*
*\param a is a marking
*\param b is a marking
*\return \c true iff a precedes b in the order described.
*/
bool operator< (const Marking& a,const  Marking& b);

/**
*\brief Checks whether the two marking are the same
*
*Two markings are the same if the have the same number of states and 
*the states in the same positions have the same number of tokens.
*
*\return \c true iff a equals b
*/
bool operator ==(const Marking& a,const Marking& b);

/**
*\brief Returns !(a == b)
*/
bool operator !=(const Marking& a, const Marking& b);

/**
*\brief Deserializes a marking from a stream
*
*\param stream is the stream from which to read the marking
*\param p is a pair that will store the marking and its value in the value function
*\return \p stream
*/
std::istream &operator>>(std::istream& stream, std::pair<Marking, double>& p);

/**
*\brief Serializes a marking into a stream
*
*\param stream is the stream to which to serialize the marking
*\param markPair is the pair that contains a marking and its value in the value function
*\return \p stream
*/
std::ostream &operator<<(std::ostream& stream,const std::pair<Marking, double>& markPair);


}

#endif
