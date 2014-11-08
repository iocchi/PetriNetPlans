#include <pnp/learning_plan/algo/MonteCarlo.h>

#include <iostream>
#include <cmath>

using namespace std;

namespace learnpnp {

MonteCarlo::MonteCarlo ( const std::string &filePath, const MonteCarlo::ParamType& parameters ) :
		Learner ( filePath, parameters.initialValue ),
		e(),
		accumulatedV(),
		par ( parameters ),
		ticks ( 1 ),
		scheduler ( new ConstantAlpha ( parameters.alpha ) ) {}

MonteCarlo::MonteCarlo ( const std::string &filePath, const ParamType& parameters, AlphaScheduler *sched ) :
		Learner ( filePath, parameters.initialValue ),
		e(),
		accumulatedV(),
		par ( parameters ),
		ticks ( 1 ),
		scheduler ( sched ) {}


void MonteCarlo::updateV ( const Marking &current, double reward,const Marking &next ) {

	if ( accumulatedV.find ( current ) == accumulatedV.end() )
		accumulatedV.insert ( make_pair ( current,.0 ) );

	e[current] = 1;

	std::map<Marking, double>::iterator VIt = accumulatedV.begin();

	for ( ; VIt != accumulatedV.end(); ++VIt ) {
		const Marking &s = ( *VIt ).first;
		double &accumulatedV_s = ( *VIt ).second;
		double &e_s = e[s]; //see comment on TDLambda

		accumulatedV_s += reward * e_s;
		e_s *= pow ( par.gamma,ticks ) ;
	}

	ticks = 1;
}

void MonteCarlo::tick() {
	++ticks;
}

void MonteCarlo::finalizeV() {

	std::map<Marking, double>::iterator VIt = accumulatedV.begin();

	for ( ; VIt != accumulatedV.end(); ++VIt ) {

		const Marking &s = ( *VIt ).first;
		const double &accumulatedV_s = ( *VIt ).second;

		scheduler->visited ( s );

//			std::cout << "V= " << V_s << "accumulatedV= " << accumulatedV_s << std::endl;
		setValueOf ( s, valueOf ( s ) +  scheduler->alpha ( s ) * ( accumulatedV_s - valueOf ( s ) ) );
//			std::cout << "V= " << V_s << std::endl;
	}

}


MonteCarlo::~MonteCarlo() {
	delete scheduler;
}

}
