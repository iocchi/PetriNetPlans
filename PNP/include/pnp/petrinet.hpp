/*
 * petrinet.hpp
 * authors: Vittorio Amos Ziparo and Daniele Calisi
 */

#include "utils.h"


template<typename PlaceClass, typename TransitionClass>
PetriNet<PlaceClass, TransitionClass>::PetriNet()
{ }

template<typename T>
struct EqualPredicate  : public std::unary_function<bool, T*> {
	T* first;
	bool operator() (T* second) {
		return *first == *second;
	}
};

//Matteo: added copy constructur to make deep copies
template<typename PlaceClass, typename TransitionClass>
PetriNet<PlaceClass, TransitionClass>::PetriNet(const PetriNet<PlaceClass, TransitionClass> &other) {

	//ok, this is a nightmare but this copy constructor is necessary to preload plans, and clone them during
	//execution. Maybe one day we'll clean this mess and reimplmenet the whole thing, but for now I just try to
	//add this.
	
 	name = other.name;

 	typename std::set<PlaceClass*>::const_iterator pit = other.places.begin();
 	for(; pit != other.places.end(); ++pit) {
 		places.insert(new PlaceClass(**pit));
 	}
// 	std::cout << "other places " << other.places.size() << " my places " << places.size() <<  std::endl;
	typename std::set<TransitionClass*>::const_iterator tit = other.transitions.begin();
	for(; tit != other.transitions.end(); ++tit) {
		transitions.insert(new TransitionClass(**tit));
	}
//	std::cout << "other transitions " << other.transitions.size() << " my transitions " << transitions.size() <<  std::endl;
	typename std::multimap<TransitionClass*, EdgePlace<PlaceClass> >::const_iterator outit = other.outputPlaces.begin();
	for(; outit != other.outputPlaces.end(); ++outit) {
		TransitionClass *t = outit->first;
		const EdgePlace<PlaceClass> & ep =  outit->second;

		EqualPredicate<TransitionClass> predicateTransition;
		predicateTransition.first = t;
		tit = find_if(transitions.begin(), transitions.end(), predicateTransition);

		EqualPredicate<PlaceClass> predicatePlace;
		predicatePlace.first = ep.p;
		pit = find_if(places.begin(), places.end(), predicatePlace);

		this->addEdge(*tit,*pit,ep.weight);
	}
//	std::cout << "other outputPlaces " << other.outputPlaces.size() << " my outputPlaces " << outputPlaces.size() <<  std::endl;
	typename std::multimap<TransitionClass*, EdgePlace<PlaceClass> >::const_iterator intit = other.inputPlaces.begin();
	for(; intit != other.inputPlaces.end(); ++intit) {
		TransitionClass *t = intit->first;
		const EdgePlace<PlaceClass> & ep =  intit->second;
	
		EqualPredicate<TransitionClass> predicateTransition;
		predicateTransition.first = t;
		tit = find_if(transitions.begin(), transitions.end(), predicateTransition);
	
		EqualPredicate<PlaceClass> predicatePlace;
		predicatePlace.first = ep.p;
		pit = find_if(places.begin(), places.end(), predicatePlace);
	
		this->addEdge(*pit,*tit,ep.weight,ep.inhibitor);
	}
//	std::cout << "other inputPlaces " << other.inputPlaces.size() << " my inputPlaces " << inputPlaces.size() <<  std::endl;
	typename std::map<PlaceClass*, std::set<TransitionClass*> >::const_iterator outTit = other.outputTransitions.begin();
	for(; outTit != other.outputTransitions.end(); ++outTit) {

		PlaceClass *place = outTit->first;
		const  std::set<TransitionClass*> & transSet =  outTit->second;

		typename std::set<TransitionClass*>::const_iterator transSetIt = transSet.begin();
		std::set<TransitionClass*> newset;
		for(; transSetIt != transSet.end(); ++transSetIt) {
			EqualPredicate<TransitionClass> predicateTransition;
			predicateTransition.first = *transSetIt;
			tit = find_if(transitions.begin(), transitions.end(), predicateTransition);
			newset.insert(*tit);
		}

		EqualPredicate<PlaceClass> predicatePlace;
		predicatePlace.first = place;
		pit = find_if(places.begin(), places.end(), predicatePlace);

		outputTransitions.insert(make_pair(*pit,newset));

	}

//std::cout << "other outputTransitions " << other.outputTransitions.size() << " my outputTransitions " << outputTransitions.size() <<  std::endl;
	
}

template<typename PlaceClass, typename TransitionClass>
void PetriNet<PlaceClass, TransitionClass>::resetInitialMarking()
{
   typename std::set<PlaceClass*>::iterator p_iter=places.begin();
   for(;p_iter!=places.end();p_iter++) {
      (*p_iter)->currentMarking=(*p_iter)->initialMarking;
   }
}

// LI 2018
// Set an initial marking for this PNP that may be different from the one stored in the pnml file.
// placenames is a vector of string labels corresponding to the places that will have an initial marking set
template<typename PlaceClass, typename TransitionClass>
void PetriNet<PlaceClass, TransitionClass>::setInitialMarking(std::vector<std::string> placenames)
{
   typename std::set<PlaceClass*>::iterator p_iter=places.begin();
   for(;p_iter!=places.end();p_iter++) {
      // TODO
      // ir = (*p_iter)->name in placenames;
      bool ir = false;
      if (ir)
        (*p_iter)->currentMarking = 1;
      else
        (*p_iter)->currentMarking = 0;
   }
}



template<typename PlaceClass, typename TransitionClass>
bool PetriNet<PlaceClass, TransitionClass>::isEnabled(TransitionClass* tr)
{
   typename std::multimap<TransitionClass*, EdgePlace<PlaceClass> >::iterator iter;
   typename std::multimap<TransitionClass*, EdgePlace<PlaceClass> >::iterator lower=inputPlaces.lower_bound(tr);
   typename std::multimap<TransitionClass*, EdgePlace<PlaceClass> >::iterator upper=inputPlaces.upper_bound(tr);
   if(lower==upper)
      PNP_ERR("ERROR: Trying to check if enabled an unexisting Transition!!");
   for (iter = lower; iter != upper; iter++)
   {
      if(iter->second.inhibitor && iter->second.p->currentMarking!=0)
         return false;
      if(!iter->second.inhibitor && iter->second.p->currentMarking< iter->second.weight)
         return false;
   }
   return true;
}

template<typename PlaceClass, typename TransitionClass>
bool PetriNet<PlaceClass, TransitionClass>::fire(TransitionClass* tr)
{
   typename std::multimap<TransitionClass*, EdgePlace<PlaceClass> >::iterator iter;
   typename std::multimap<TransitionClass*, EdgePlace<PlaceClass> >::iterator lower=inputPlaces.lower_bound(tr);
   typename std::multimap<TransitionClass*, EdgePlace<PlaceClass> >::iterator upper=inputPlaces.upper_bound(tr);

   if (lower==upper) {
      PNP_OUT("INFO: Trying to fire an unexisting Transition in inputPlaces!! Probably a source." );
   }
   for (iter = lower; iter != upper; iter++)
   {
      if(iter->second.inhibitor)
         continue;
      if( iter->second.p->currentMarking<iter->second.weight)
      {
         PNP_ERR("ERROR: Tryng to fire disabled Transition!! This shoud not happen!");
         return false;
      }

      iter->second.p->currentMarking-= iter->second.weight;
   }

   lower=outputPlaces.lower_bound(tr);
   upper=outputPlaces.upper_bound(tr);

   if(lower==upper) {
      PNP_OUT("INFO: Trying to fire an unexisting Transition in outputPlaces!. Probably a sink");
   }

   for (iter = lower; iter != upper; iter++)
   {
      if(iter->second.inhibitor)
      {
         PNP_ERR("ERROR: Inhibitor arc cannot go from Transition to Place!!!");
         return false;
      }
      iter->second.p->currentMarking+= iter->second.weight;
   }

   return true;
}

template<typename PlaceClass, typename TransitionClass>
bool PetriNet<PlaceClass, TransitionClass>::addEdge(PlaceClass* p,
	TransitionClass* t, unsigned int weight, bool inhibitor)
{
   EdgePlace<PlaceClass>  ep;
   ep.p=p;
   ep.inhibitor=inhibitor;
   ep.weight=weight;
   std::pair<TransitionClass*,EdgePlace<PlaceClass> > aux(t,ep);
   outputTransitions[p].insert(t);
   inputPlaces.insert(aux);
   return true;
}

template<typename PlaceClass, typename TransitionClass>
bool PetriNet<PlaceClass, TransitionClass>::addEdge(TransitionClass* t, PlaceClass* p, unsigned int weight)
{
   EdgePlace<PlaceClass>  ep;
   ep.p=p;
   ep.weight=weight;
   ep.inhibitor=false;
   std::pair<TransitionClass*,EdgePlace<PlaceClass> > aux(t,ep);
   outputPlaces.insert(aux);
   return true;

}

template<typename PlaceClass, typename TransitionClass>
void PetriNet<PlaceClass, TransitionClass>::printCurrentMarking()
{
   PNP_OUT("INFO: Current Marking");
   typename std::set<PlaceClass*>::iterator p_iter=places.begin();
   for(;p_iter!=places.end();p_iter++)
      PNP_OUT("INFO: "<<(*p_iter)->id<<"->"<<(*p_iter)->currentMarking);
}

template<typename PlaceClass, typename TransitionClass>
std::set<TransitionClass*> PetriNet<PlaceClass, TransitionClass>::getAllEnabledTransitions()
{
   std::set<TransitionClass*> result;
   result.clear();
   std::set<PlaceClass*> nep = getNonEmptyPlaces();
   typename std::set<PlaceClass* >::iterator p_iter=nep.begin();
   for(;p_iter!=nep.end();p_iter++)
   {
      typename std::set<TransitionClass*>::iterator t_iter=outputTransitions[*p_iter].begin();
      for(;t_iter!=outputTransitions[*p_iter].end();t_iter++)
         if(isEnabled(*t_iter))
            result.insert(*t_iter);
   }

   return result;

}

// Fires all enabled Transitions
template<typename PlaceClass, typename TransitionClass>
bool  PetriNet<PlaceClass, TransitionClass>::fireAll()
{
   std::set<TransitionClass*> enabledTr=getAllEnabledTransitions();
   if(enabledTr.empty())
      return false;

   typename std::set<TransitionClass*>::iterator t_iter=enabledTr.begin();
   for(; t_iter!=enabledTr.end();t_iter++)
   {
      if(isEnabled((*t_iter))) // important to check (may have been disabled by firing of other Transition)
      {
         if(!fire((*t_iter)))
         {
            PNP_OUT("DEBUG:Cannot fire the Transition!");
         }
      }

   }

   return true;

}

template<typename PlaceClass, typename TransitionClass>
PetriNet<PlaceClass, TransitionClass>::~PetriNet()
{
   clearAll();
}

template<typename PlaceClass, typename TransitionClass>
void PetriNet<PlaceClass, TransitionClass>::clearAll()
{
   typename std::set<TransitionClass*>::iterator t_iter=transitions.begin();
   for(;t_iter!=transitions.end();t_iter++)
      if(*t_iter!=NULL)
         delete (*t_iter);
   typename std::set<PlaceClass*>::iterator p_iter=places.begin();
   for(;p_iter!=places.end();p_iter++)
      if(*p_iter!=NULL)
         delete (*p_iter);
   name.clear();
   outputPlaces.clear();
   inputPlaces.clear();
   places.clear();
   outputTransitions.clear();
}

template<typename PlaceClass, typename TransitionClass>
std::set<PlaceClass*> PetriNet<PlaceClass, TransitionClass>::getNonEmptyPlaces()
{
	std::set<PlaceClass*> r;
	for (typename std::set<PlaceClass*>::iterator it = places.begin();
	it != places.end(); ++it) {
		if ((*it)->currentMarking>0) r.insert(*it);
	}
	return r;
}
