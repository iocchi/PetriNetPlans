#ifndef basic_plan_definition_guard
#define basic_plan_definition_guard

/**\file basic_plan.h
 * \brief This file defines a basic pnp_plan, the one that probably is the most widely used.
 * 
 * \author Matteo Leonetti
 **/

namespace PetriNetPlans {

  template<typename PnpPlaceClass, typename PnpTransitionClass>
  class PnpPlanTemplate;

  class PnpPlace;
  class PnpTransition;
  
  /** 
  *\brief Definition of PnpPlan to simplify the code
  **/
typedef PnpPlanTemplate<PnpPlace, PnpTransition> PnpPlan;

}

#endif
