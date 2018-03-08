// Stores and evaluate constraints on the policy
// WIP ?
//TODO: check status of this part

#include "PRUMDP/PRUplus.h"

#include <stdlib.h>

#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

/** Evaluates a situation on a specific criterion */
class PRUon {
public:
  /** Matches the constraint part with the actual situation, and appends out with necessary data.
      @return false iff match failed and out has not been modified.
   */
  virtual bool match(const string &actionId, const PRUstate &actionParam, const PRUstate &stateVariables,
                     vector<string> out) const = 0;
};

namespace PRUOn {

  class onAction : public PRUon {
  public:
    const string name;
    onAction(const string &actionId) : name(actionId) {}

    virtual bool match(const string &actionId, const PRUstate &actionParam, const PRUstate &stateVariables,
                       vector<string> out) const {
      if (actionId==name) {
        return true;
      } else
        return false;
    }
  };

  class onEachAction : public PRUon {
    virtual bool match(const string &actionId, const PRUstate &actionParam, const PRUstate &stateVariables,
                       vector<string> out) const {
      out.push_back(actionId);
      return true;
    }
  };

  class onParamValue : public PRUon {
  public:
    const string name;
    const string value;
    onParamValue(const string &parameter, const string &pValue) : name(parameter), value(pValue) {}
    virtual bool match(const string &actionId, const PRUstate &actionParam, const PRUstate &stateVariables,
                       vector<string> out) const {
      PRUstate::const_iterator it = actionParam.find(name);
      if (it == actionParam.end())
        return false;
      else if (it->second == value)
        return true;
      else
        return false;
    }
  };

  class onEachParamValue : public PRUon {
  public:
    const string name;
    onEachParamValue(const string &parameter) : name(parameter) {}
    virtual bool match(const string &actionId, const PRUstate &actionParam, const PRUstate &stateVariables,
                       vector<string> out) const {
      PRUstate::const_iterator it = actionParam.find(name);
      if (it == actionParam.end())
        return false;
      else {
        out.push_back(it->second);
        return true;
      }
    }
  };

  class onSVvalue : public PRUon {
  public:
    const string name;
    const string value;
    onSVvalue(const string &variable, const string &vValue) : name(variable), value(vValue) {}
    virtual bool match(const string &actionId, const PRUstate &actionParam, const PRUstate &stateVariables,
                       vector<string> out) const {
      PRUstate::const_iterator it = stateVariables.find(name);
      if (it == stateVariables.end())
        return false;
      else if (it->second == value)
        return true;
      else
        return false;
    }
  };

  class onEachSVvalue : public PRUon {
  public:
    const string name;
    onEachSVvalue(const string &variable) : name(variable) {}
    virtual bool match(const string &actionId, const PRUstate &actionParam, const PRUstate &stateVariables,
                       vector<string> out) const {
      PRUstate::const_iterator it = stateVariables.find(name);
      if (it == stateVariables.end())
        return false;
      else {
        out.push_back(it->second);
        return true;
      }
    }
  };

} // namespace PRUon

PRUconstraint::~PRUconstraint() {
  for (vector<const PRUon*>::const_iterator it=elements.begin();
       it != elements.end(); ++it)
    delete(*it);
  elements.clear();
}

void PRUconstraint::reset() {
  matched.clear();
}

void PRUconstraint::match(string &actionId, PRUstate &actionParam, PRUstate &stateVariables) {
  vector<string> res;
  res.push_back(name); // first element is the constraint name so that comparison of the following variable elements makes sense
  for (vector<const PRUon*>::const_iterator it=elements.begin();
       it != elements.end(); ++it) {
    bool ok = (*it)->match(actionId, actionParam, stateVariables, res);
    if (!ok)
      return; // no match
  };
  matched[res]++;
}

static float readConstValue(xmlpp::TextReader &reader) {
  string v="0";
  if (reader.has_attributes()) {
    reader.move_to_first_attribute();
    do {
      if (reader.get_name() == "than")
        v = reader.get_value();
    } while (reader.move_to_next_attribute());
    reader.move_to_element();
  }
  return atof(v.c_str());
}

PRUconstraint::PRUconstraint(xmlpp::TextReader &reader) {
  validIfNoLessThan = validIfEachNoLessThan = validIfAnyNoLessThan = 0;
  validIfNoMoreThan = validIfEachNoMoreThan = validIfAnyNoMoreThan = 1000000; // Many

  if (reader.has_attributes()) {
    reader.move_to_first_attribute();
    do {
      if (reader.get_name() == "id")
        name = reader.get_value();
    } while (reader.move_to_next_attribute());
    reader.move_to_element();
  }
  while(reader.read()) {
    string name = reader.get_name();
    if ((reader.get_node_type() == xmlpp::TextReader::EndElement) &&
        (name == "Constraint"))
      break;
    if (reader.get_node_type() != xmlpp::TextReader::Element )
      continue;
    if (name == "On") {
    } else if (name == "Action") {
      string id = "";
      if (reader.has_attributes()) {
        reader.move_to_first_attribute();
        do {
          if (reader.get_name() == "id")
            id = string(reader.get_value());
        } while (reader.move_to_next_attribute());
        reader.move_to_element();
      }
      if (id == "")
        elements.push_back(new PRUOn::onEachAction());
      else
        elements.push_back(new PRUOn::onAction(id));
    } else if (name == "ActionParameter") {
      string id = "";
      string val = "";
      if (reader.has_attributes()) {
        reader.move_to_first_attribute();
        do {
          if (reader.get_name() == "id")
            id = reader.get_value();
          else if (reader.get_name() == "value")
            val = reader.get_value();
        } while (reader.move_to_next_attribute());
        reader.move_to_element();
      }
      if (id == "")
        std::cerr << "Unable to test on all action parameters yet!" << std::endl;
      else {
        if (val == "")
          elements.push_back(new PRUOn::onEachParamValue(id));
        else
          elements.push_back(new PRUOn::onParamValue(id,val));
      }
    } else if (name == "StateVariable") {
      string id = "";
      string val = "";
      if (reader.has_attributes()) {
        reader.move_to_first_attribute();
        do {
          if (reader.get_name() == "id")
            id = reader.get_value();
          else if (reader.get_name() == "value")
            val = reader.get_value();
        } while (reader.move_to_next_attribute());
        reader.move_to_element();
      }
      if (id == "")
        std::cerr << "Unable to test on all state variables yet!" << std::endl;
      else {
        if (val == "")
          elements.push_back(new PRUOn::onEachSVvalue(id));
        else
          elements.push_back(new PRUOn::onSVvalue(id,val));
      }
    } else if (name == "NoLess") {
      validIfNoLessThan = readConstValue(reader);
    } else if (name == "EachNoLess") {
      validIfEachNoLessThan = readConstValue(reader);
    } else if (name == "AnyNoLess") {
      validIfAnyNoLessThan = readConstValue(reader);
    } else if (name == "NoMore") {
      validIfNoMoreThan = readConstValue(reader);
    } else if (name == "EachNoMore") {
      validIfEachNoMoreThan = readConstValue(reader);
    } else if (name == "AnyNoMore") {
      validIfAnyNoMoreThan = readConstValue(reader);
    }
  } // while reader.read()
} // PRUconstraint(reader)

float PRUconstraint::getScore() const {
  bool valid = false;
  bool invalid = false;
  int sumAll = 0;
  for (map<vector<string>,int>::const_iterator it=matched.begin();
       it != matched.end(); ++it) {
    int nb = it->second;
    sumAll += nb;
    if (nb < validIfEachNoLessThan) {
      invalid = true;
      break;
    }
    if (nb > validIfEachNoMoreThan) {
      invalid = true;
      break;
    }
    if (nb >= validIfAnyNoLessThan) {
      valid = true;
    }
    if (nb > validIfAnyNoMoreThan) {
      valid = true;
    }
  }
  if (valid)
    if (sumAll < validIfNoLessThan)
      valid = false;
    else if (sumAll > validIfNoMoreThan)
      valid = false;
    else if (! invalid)
      return score;
  // otherwise
  return 0;
}

