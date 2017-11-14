import time
import rospy
from AbstractAction import AbstractAction

class say(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Saying "' + self.params.replace("_", " ") + '"')
        self.starting_time = time.time()

    def _stop_action(self):
        rsopy.loginfo('Finished saying "' + self.params.replace("_", " ") + '"')

    def _is_action_done(self):
        elapsed_time = time.time() - self.starting_time
        if elapsed_time > 4:
            return True
        return False
