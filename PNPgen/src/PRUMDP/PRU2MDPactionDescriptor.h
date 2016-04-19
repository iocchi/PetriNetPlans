// Stores an action along with its progress (instanciated Layer)
#ifndef __PRU2MDPactionDescriptor__
#define __PRU2MDPactionDescriptor__

class PRU2MDPprogress;
class MDPaction;

class PRU2MDPactionDescriptor{
 public:
  const PRU2MDPprogress *progress;
  const MDPaction *action;
  PRU2MDPactionDescriptor( const PRU2MDPprogress *prog, const MDPaction *act) {
    progress = prog;
    action = act;
  }
};

#endif
