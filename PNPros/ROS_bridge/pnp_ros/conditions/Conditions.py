import rospy

from std_msgs.msg import String


def currentNode(params):
    node = str(params[0])

    topic = "/current_node"

    message = rospy.wait_for_message(topic, String, timeout=10)

    return node == message.data

def closestNode(params):
    node = str(params[0])

    topic = "/closest_node"

    message = rospy.wait_for_message(topic, String, timeout=10)

    return node == message.data

def currentGoal(params):
    node = str(params[0])

    topic = "/PNP/goal"

    # this is not a latch message...it should be
    #message = rospy.wait_for_message(topic, String, timeout=10)

    #topo_goal = str(message.goal.params)

    #fake
    topo_goal = node
    return node == topo_goal
