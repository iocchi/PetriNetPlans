#ifndef learnpnp_MaxLearner__guard
#define learnpnp_MaxLearner__guard

#include <pnp/learning_plan/algo/TDParams.h>
#include <pnp/learning_plan/Marking.h>

#include <boost/bind.hpp>

#include <algorithm>

namespace learnpnp {

class ExpPolicy;

template <typename LearnerType>
class MaxLearner  : public LearnerType {

	private:
		typename LearnerType::ParamType par;
	
	public:
		
	MaxLearner(const std::string &filePath, const typename LearnerType::ParamType & parameters) :
			LearnerType(filePath, parameters), par(parameters), initialV(this->begin(), this->end()) {}

	protected:

	virtual void finalizeV() {
		LearnerType::finalizeV();
		
		for_each(this->begin(), this->end(), boost::bind(&MaxLearner::maxValue,this,_1));
	}

	private:

		void maxValue(const std::pair<const learnpnp::Marking, double>& destValue) {
			if(initialV.find(destValue.first) == initialV.end())
				initialV[destValue.first] = par.initialValue;
			this->setValueOf(destValue.first, std::max(destValue.second, initialV[destValue.first]));
		}

		std::map<Marking, double> initialV;
	
};

}

#endif // MAXLEARNER_H
