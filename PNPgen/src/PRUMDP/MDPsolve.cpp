// The part for solving the MDP

#include "PRUMDP/MDP.h"
#include <cmath>

void MDP::initVI() {
  value[0].clear();
  value[1].clear();
  policy.clear();
  int s = 0; // for indexing states
  for(vector<MDPstate*>::iterator itS = states.begin();
      itS != states.end(); ++itS) {
    value[0].push_back(0);
    value[1].push_back(0);
    (*itS)->index = s++;
  } // for *itS in states
}

float MDP::iterate(float gamma) {
  float delta = -1e10;
  int h = policy.size();
  vector<float> &pv = value[h & 1];
  vector<float> &nv = value[(h+1) & 1];
  policy.push_back(vector<const MDPaction*>());
  for(vector<MDPstate*>::const_iterator itS = states.begin();
      itS != states.end(); ++itS) {
    float bv = -1e10;
    const MDPaction *ba = NULL;
    const MDPstate *s = *itS;
    for(set<const MDPaction *>::const_iterator itA = s->availableActions.begin();
	itA != s->availableActions.end(); ++itA) {
      float v = 0;
      for (set<MDPstate*>::const_iterator itO = (*itA)->outcomes.begin();
	   itO != (*itA)->outcomes.end(); ++itO) {
	const MDPstate *s2 = *itO;
	const PRUoutcome *out = s2->prevOutcome;
	// v += T(s,a,s2)*(V(s2)*gamma^duration+quality)
	float quality = out->getQuality(s->stateVariables, s2->stateVariables, (*itA)->parameters );/* out->qualityConstant 
	  + PRUplus::domainFunction(s->stateVariables, s2->stateVariables,
	  out->quality, out->qualityParameter);*/
	float duration = out->getDuration(s->stateVariables, s2->stateVariables, (*itA)->parameters ); /*out->durationConstant 
	  + PRUplus::domainFunction(s->stateVariables, s2->stateVariables,
	  out->duration, out->durationParameter);*/
	float fct = powf(gamma, duration);
	v += out->probability * (pv[s2->index] + quality)*fct;
      } // for *itO in current-action's outcomes
      if (v>bv) {
	bv = v;
	ba = *itA;
      }
    } // for *itA in s.availableActions
    nv[s->index] = bv;
    float d = bv - pv[s->index];
    if (d>delta)
      delta = d;
    policy.back().push_back(ba);
  } // for *itS in states

  return delta;
}

float MDP::getValue(int state) const {
  int h = policy.size();
  const vector<float> &pv = value[h & 1];
  return pv[state];
}
