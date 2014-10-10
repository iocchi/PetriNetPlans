#ifndef learnpnp_LoggingController_h__guard
#define learnpnp_LoggingController_h__guard

#include <pnp/learning_plan/Controller.h>
#include <pnp/learning_plan/Marking.h>

#include <string>
#include <set>

namespace learnpnp {

class Learner;
class ExpPolicy;


/**
 * \brief Keeps track of the value of specified markings, and writes them on a file
 *
 * Given a set of Markings' ids, it records which ones are visited on a given episode
 * and logs their value on a file.
 * The set of Markings the logger will take into account is specified to the constructor.
 * Before the first episode, the method initialize() must be called, in order to
 * initialise the log file.
 * Then, through the method afterUpdate(), this PlaceLogger keeps track of the markings
 * actually visited. Finally, when saving() is called, it stores the values
 * of the specified markings at that time, or a '?' (question mark) if that marking has
 * not been visited yet.
 *
 * The markings that must be logged are specified through their id. A marking id is given
 * by the ids of each place with a number of token higher than zero, separated by an '-' (hyphen)
 * and in lexicographical order. For instance, let the marking we want to log have a non-zero
 * number of tokens in the places p1, p2, p14, p32. Its marking would be "p1-p14-p2-p32".
 * Please note the lexicographical order.
 *
 * \see LoggingController for an example of usage.
 *
 * \author Matteo Leonetti
 *
 * */
class PlaceLogger {
	
	
public:

	/**
	 * \brief Creates a PlaceLogger with a given set of markings and a file to store their values
	 *
	 *\param filepath the path to the file to store markings' values
	 *\param places a set of markings' ids, see the class description for more information
	 */
	PlaceLogger(const std::string& filepath, const std::set<std::string>& places);

	/**
	 * \brief Resets (creates and empties) the file with logged values
	 */
	void initialize();

	/**
	 * \brief notifies the place logger that a marking has been visited
	 *
	 * The place logger logs only the value of visited markings, and stores a
	 * '?' for the ones not visited
	 */
	void afterUpdate(const Marking &current);

	/**
	 * \brief Stores the values of the specified markings taken from a given Learner
	 *
	 * \param V the learner whose value function will be logged
	 */
	void saving(Learner *V);
	
private:
	
	std::string filepath;
	std::set<std::string> places;
	std::map<std::string, Marking> markings;
	
	
};



/**
*\brief A Controller that can log markings' values at the end of each episode
*
* The actual logging is performed by PlaceLogger s, that must be registered
* with the LoggingController
*
* here is an example of pseudocode:
\code

set<string> interestingPlaces;

//insert every marking you want to log
interestingPlaces.insert("p1");

//create the place logger
PlaceLogger logger("File_to_save_logging.txt",interestingPlaces);

//if it is the first episode, initialise logger
if(firstEpisode)
	logger.initialize()

//instantiate the controller
//use the sub-class of LoggingController of your choice
LoggingController *controller = new LoggingController();

//register the logger with the controller
controller->addLogger(logger);
\endcode
* 
* You may use multiple loggers with the same controller.
*
* When the controller is destroyed,
* all registered place loggers output the value of the specified markings on the file,
* for each of those marking that have been visited during the current episode. Otherwise
* the output is a '?' (question mark).
*
*\author Matteo Leonetti
*/

class LoggingController : public Controller {

public:

	/**
	 *\brief Created a LoggingController from a given learner
	 *
	 * The \p learner is used to read values from its value function
	 *
	 * \param learner a Learner whose value function must be logged/
	 */
	explicit LoggingController(Learner *learner);

	/**
	 * \brief virtual dtor
	 */
	virtual ~LoggingController();

	/**
	 * \brief Notifies this controller that a Marking has been visited
	 *
	 * This function  lets the loggers know
	 * whether a Marking has been visited or not.
	 * Loggers only save the Marking's value if it has been visited during
	 * that episode. Must be called by the sub-classes for logging to take place.
	 *
	 * \param current the Marking that has been visited
	 * */
	void visited(const Marking& current);

	/**
	 * \brief Registers a given PlaceLogger with this controller
	 *
	 * The registered PlaceLogger \p pl will be notified when a Marking has been visited,
	 * and when this controller is being destroyed, and it must write markings' values
	 * to the file.
	 *
	 * Remember to initialise the place logger on the first episode to reset the file.
	 *
	 * \param pl the PlaceLogger to be register
	 * */
	void addLogger(const PlaceLogger& pl);

private:
	void saveState();
	std::vector<PlaceLogger> loggers;

protected:
	//owns
	Learner  *learner;



};

}

#endif // LOGGINGLEARNER_H
