#include <pnp/learning_plan/algo/LoggingController.h>

#include <pnp/learning_plan/Learner.h>

#include <boost/bind.hpp>

#include <algorithm>


using boost::bind;

using namespace std;

namespace learnpnp {

LoggingController::LoggingController(Learner *learner) : learner(learner) {}

LoggingController::~LoggingController() {
	if (learning) {
		learner->finalize();
		learner->saveState();
	}

	this->saveState();

	delete learner;
}

void LoggingController::visited(const Marking& current) {
	for_each(loggers.begin(),loggers.end(),bind(&PlaceLogger::afterUpdate,_1,current));
}

void LoggingController::addLogger(const PlaceLogger& pl) {
	loggers.push_back(pl);
}

void LoggingController::saveState() {
	for_each(loggers.begin(), loggers.end(),bind(&PlaceLogger::saving,_1,learner));
}

}
