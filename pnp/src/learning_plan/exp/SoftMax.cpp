#include <pnp/learning_plan/exp/SoftMax.h>

#include <boost/bind.hpp>

#include <cmath>
#include <functional>
#include <numeric>
#include <algorithm>

#include <stdlib.h>

using namespace std;
using boost::bind;

namespace learnpnp {

SoftMax::SoftMax(double tau, bool remember) : ExpPolicy(remember), tau(tau) {}

int SoftMax::makeChoice(Learner *learner,const Marking &current, const std::vector<Marking> &states) {

    //this can be impelemented with fewer implicit loops, but I don't want to do it unless some profiling shows
    //it's really necessary to sacrifice readability for efficiency

	vector<double> values;
    transform(states.begin(),states.end(),std::back_inserter(values),
              bind(&Learner::valueOf,learner,_1));


    double min = *min_element(values.begin(),values.end());
    if (min < 0) {
        //make the values positive
        transform(values.begin(),values.end(),values.begin(),bind(plus<double>(),_1,min));
    }

    vector<double> weights;
    transform(values.begin(),values.end(),back_inserter(weights),bind(&SoftMax::computeWeight,this,_1));

    double sum = accumulate(weights.begin(),weights.end(),0.0);

    //normalize the weight with respect to the sum and RAND_MAX
    // for each weight w computes w/sum * RAND_MAX
    transform(weights.begin(),weights.end(),weights.begin(),
              bind(multiplies<double>(),bind(divides<double>(),_1,sum), RAND_MAX));

	//cumulative sums
	/* An example: imagine the normalized weights are 0.2 * RAND_MAX,
	0.3 * RAND_MAX, and 0.5 * RAND_MAX
	we drop RAND_MAX for readability now, the cumulative sums are
	0.2, 0.5, 1
	we extract a value v in [0,1] (in [0,RAND_MAX] in practice)
	and look for the first value among the cumulative sums that is >= v
	*/
	vector<double>cumulative;
	partial_sum(weights.begin(),weights.end(),back_inserter(cumulative));

	int v = rand();

	vector<double>::iterator result = find_if(cumulative.begin(),cumulative.end(),bind(greater_equal<double>(),_1,v));

	return distance(cumulative.begin(),result);

}

double SoftMax::computeWeight(double value) {
    return exp ( value  /tau );
}


}
