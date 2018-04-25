import rospy
from abc import ABCMeta, abstractmethod, abstractproperty

class AbstractCondition():
    __metaclass__ = ABCMeta

    def __init__(self):
        # subscribe to the topic with a callback
        rospy.Subscriber(self._topic_name, self._topic_type, self._callback)

        # last_data will be None until the subscribed topic will return some data
        self.last_data = None

    def _callback(self, data):
        self.last_data = data

    @abstractproperty
    def _topic_name(self):
        raise NotImplementedError()

    @abstractproperty
    def _topic_type(self):
        raise NotImplementedError()

    @abstractmethod
    def evaluate(self, params):
        raise NotImplementedError()
