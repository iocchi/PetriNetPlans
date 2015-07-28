#ifndef _MY_ACTIONS_
#define _MY_ACTIONS_

// robotname as external variable (defined in MyActions.cpp)
extern std::string robotname;

// Action implementation

void init(string params, bool *run);
void gotopose(string params, bool *run);
void home(string params, bool *run);
void wave(string params, bool *run);
void turn360(string params, bool *run);
void sense1(string params, bool *run);

int closeToHomeCond();

#endif

