#include <pnp/learning_plan/Learner.h>

namespace learnpnp {



Learner::Learner(const std::string &filePath, double defaultV) : V(), file(filePath),finalized(false),defaultValue(defaultV) {
		readVFunctionFromTxt(filePath, V);
}


void Learner::saveState() {
  finalize();
  if(file != "")
	writeVFunctionToTxt(file, V);
}

void Learner::finalize() {
	if(!finalized) {
		finalized = true;
		finalizeV();
	}
}

void Learner::update(const Marking &current, double reward,const Marking &next) {
	if(V.find(current) == V.end())
		V[current] = defaultValue;

	if(V.find(next) == V.end())
		V[next] = defaultValue;

	this->updateV(current,reward,next);
}


double Learner::valueOf(const Marking& s)  {
	if(V.find(s) == V.end())
		V[s] = defaultValue;

	return V[s];
}

double Learner::valueOfId(const std::string& s) throw(std::runtime_error) {
	std::map<Marking,double>::const_iterator it = V.begin();

	for(; it != V.end(); ++it) {
		if(it->first.getId() == s)
			return it->second;
	}
	
	throw std::runtime_error("No Marking with id " + s);
}

bool Learner::isSet(const Marking& s) {
		return !(V.find(s) == V.end());
}

void Learner::setValueOf(const Marking& s, double value) {
	V[s] = value;
}

std::map<Marking, double>::iterator Learner::begin() {
	return V.begin();
}
std::map<Marking, double>::const_iterator Learner::begin() const {
	return V.begin();
}

std::map<Marking, double>::iterator Learner::end() {
	return V.end();
}
std::map<Marking, double>::const_iterator Learner::end() const {
	return V.end();
}


Learner::~Learner() {}

}
