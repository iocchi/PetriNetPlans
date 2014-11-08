#include "ActionRew.h"

#include "RewEnv.h"

namespace learnpnp {

ActionRew::ActionRew(RewEnv* env, double reward): env(env), reward(reward) {}

void ActionRew::end() {
    env->sumRew(reward);
}

bool ActionRew::finished() {
    return true;
}


}