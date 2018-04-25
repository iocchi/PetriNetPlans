import os
import glob
import rospy

from AbstractCondition import AbstractCondition
from importlib import import_module
import inspect

class ConditionManager():

    def __init__(self):
        self.condition_instances = {}

        # Initialize all the classes in current folder which implement AbstractCondition
        for file in glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), "*.py")):
            name = os.path.splitext(os.path.basename(file))[0]
            try:
                condition_class = getattr(import_module(name), name)
            except (ImportError, AttributeError):
                continue
            else:
                if issubclass(condition_class, AbstractCondition) and not inspect.isabstract(condition_class):
                    # Instanciate the condition
                    condition_instance = condition_class()

                    self.condition_instances.update({
                        name : condition_instance
                    })

                    rospy.loginfo("Initialized condition " + name)
                else:
                    rospy.logwarn("Class " + name + " does not inherit from AbstractCondition")

    def evaluate(self, condition_name, params):
        try:
            return self.condition_instances[condition_name].evaluate(params)
        except KeyError:
            rospy.logwarn("Condition " + condition_name + " not implemented")
            # return true when the condition is not implemented, to avoid loops..
            return True

    def dump_conditions(self, conditions_list):
        raise NotImplementedError()
