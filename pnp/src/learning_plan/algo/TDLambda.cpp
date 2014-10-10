#include <pnp/learning_plan/algo/TDLambda.h>
#include <iostream>
#include <cmath>

namespace learnpnp {

TDLambda::TDLambda ( const std::string &filePath, const TDLParams& parameters ) :
		Learner ( filePath, parameters.initialValue ),
		e(),
		memory(),
		lastV ( 0 ),
		last_e_s ( 0 ),
		lastMarking(),
		par ( parameters ),
		ticks ( 1 ),
		scheduler ( new ConstantAlpha ( parameters.alpha ) ) {}

TDLambda::TDLambda ( const std::string &filePath, const TDLParams& parameters, AlphaScheduler *sched ) :
		Learner ( filePath, parameters.initialValue ),
		e(),
		memory(),
		lastV ( 0 ),
		last_e_s ( 0 ),
		lastMarking(),
		par ( parameters ),
		ticks ( 1 ),
		scheduler ( sched ) {}


void TDLambda::updateV ( const Marking &current, double reward,const Marking &next ) {

// 		std::cout << "current: " << std::make_pair(current,valueOf(current)) << " reward= " << reward << std::endl;

// 	  if(V[current] ==0) {
// 		  std::cout <<std::make_pair(current,0.0);
// 		  std::cout << std::endl;
// 	  }

	scheduler->visited ( current );

	lastV = pow ( par.gamma,ticks ) * valueOf ( next );
	double delta = reward + lastV - valueOf ( current );

	lastMarking = current;

	/*this memory thing is very inefficient, but I'm trying to get the updates
	in finalize() right. I'll fix this with something more reasonable as soon as I can */
	memory.clear();

	if ( par.lambda == 0 ) {
		//special case, don't iterate
		double val = valueOf ( current ) + scheduler->alpha ( current ) * delta;
		setValueOf ( current,val );
		memory[current] =  scheduler->alpha ( current ) * lastV;
	} else {
		e[current]=1;

		std::map<Marking, double>::iterator VIt = begin();
		last_e_s =  pow ( par.gamma,ticks ) * par.lambda;

		for ( ; VIt != end(); ++VIt ) {
			const Marking &s = ( *VIt ).first;
			double &V_s = ( *VIt ).second;
			double &e_s = e[s]; //profiling showed that a lot of time is spent comparing markings in operator[]
			//therefore I'm changing this to do it only once per state.
			//
			//I might consider storing the trace explicitly later

			V_s += scheduler->alpha ( s ) * delta * e_s;
			memory[s] = scheduler->alpha ( s ) * lastV;
			e_s *= last_e_s;
		}
	}

// 	std::cout << "V[current]= " << valueOf ( current )  << std::endl;
	ticks = 1;
}

void TDLambda::finalizeV() {

//		std::cout << "********* finalize lastV: " << lastV << std::endl;

// 		if(par.lambda == 0) {
// 			V[lastMarking] -= scheduler->alpha(lastMarking) * lastV;
// 			return;
// 		}
//
// 		//undo the last par.gamma * V[next] update
// 		std::map<Marking, double>::iterator VIt = V.begin();
//
// 		for ( ; VIt != V.end(); ++VIt ) {
// 			const Marking &s = ( *VIt ).first;
// //			std::cout << std::make_pair(s,V[s]) << std::endl;
// //			std::cout << e[s] << " *  1/" <<  last_e_s  << std::endl;
// 			//e[s] *= 1. / last_e_s;
// //			std::cout << V[s] << " - " << scheduler->alpha(s) << " * " << lastV << " * " << e[s] << std::endl;
// 			V[s] -= scheduler->alpha(s) * lastV * ( e[s] / last_e_s);
// //			std::cout << std::make_pair(s,V[s]) << std::endl;
// 		}

	std::map<Marking, double>::iterator mIt = memory.begin();

	for ( ; mIt != memory.end(); ++mIt ) {
		double val = valueOf ( mIt->first ) - mIt->second;
		setValueOf ( mIt->first,val );
	}

}

void TDLambda::tick() {
	++ticks;
}

TDLambda::~TDLambda() {
	delete scheduler;
}

}


