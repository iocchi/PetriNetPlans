#ifndef learnpnp_TDParams_h__guard
#define learnpnp_TDParams_h__guard

namespace learnpnp {
 
	
  /**
  *\brief Set of parameter for TD(lambda) methods
  *
  *\author Matteo Leonetti
  */
  struct TDLParams  {
	TDLParams() : lambda(0.) {}
	double alpha;
	double gamma;
	double initialValue;
	double lambda;
  };
  
    /**
  *\brief Set of parameters for TD(0) methods
  *\deprecated This used to be a structure subclassed by TDLParams but should not be used any more. TDLParams's lambda defaults to 0.
  *
  *\author Matteo Leonetti
  */
  typedef TDLParams TD0Params; 
}

#endif
