template<typename PnpPlanClass>
PnpExecuter<PnpPlanClass>::PnpExecuter(ExecutableInstantiator* inst) :
    mainPlan(NULL), istantiator(inst) {}

template<typename PnpPlanClass>
		void PnpExecuter<PnpPlanClass>::setMainPlan(const std::string& planName)
{
	if (mainPlan) delete mainPlan;
	mainPlan = NULL;
	
	try {
		PnpExecutable *planEx = istantiator->createExecutable(planName);
		
		if(planEx == NULL) {
			PNP_ERR("Executable "<< planName << " not instantiated");
			return;
		}

		PnpPlanClass *plan = dynamic_cast< PnpPlanClass* >(planEx);
		
		if (plan == NULL) {
			PNP_ERR(planName << " is not a plan");
			delete planEx;
			return;
		}

		mainPlan = plan;
		mainPlan->resetInitialMarking();

	}catch (std::runtime_error& error) {
		PNP_ERR(error.what());
		return;
	}

}
template<typename PnpPlanClass>
std::string PnpExecuter<PnpPlanClass>::getMainPlanName() const {

	return (mainPlan != NULL)?mainPlan->getPlanName():"";
}

/** 
* \todo implement a planFailed() as well, to test whether the main plan is
* in a fail state.
*/
template<typename PnpPlanClass>
bool PnpExecuter<PnpPlanClass>::goalReached() {

	return mainPlan->finished();
}


template<typename PnpPlanClass>
bool PnpExecuter<PnpPlanClass>::execMainPlanStep()
{
	if(mainPlan == NULL)
		return false;
	
	mainPlan->executeStep();
	
	if (mainPlan->isInGoalState()) {
		PNP_OUT("Main goal reached");
		return false;
	}
	else if (mainPlan->isInFailState()) {
		PNP_OUT("Main failure");
		return false;
	}
	else return true;
}

template<typename PnpPlanClass>
PnpExecuter<PnpPlanClass>::~PnpExecuter(){
	delete mainPlan;
	delete istantiator;
}
