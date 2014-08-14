#include <pnp/pnp_executable.h>

#include <pnp/utils.h>

namespace PetriNetPlans {
  
bool PnpExecutable::getInternalConditionValue(const std::string& name) {
  if (name == "finished") return finished();
  if (name == "failed") return failed();

  PNP_OUT("Unknown condition '" <<name<< "'"); 
  return false;
}

}
