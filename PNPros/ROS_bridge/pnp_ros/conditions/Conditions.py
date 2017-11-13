import rospy

from std_msgs.msg import String


def currentNode(params):
    node = str(params[0])
    print params

    topic = "/current_node"

    message = rospy.wait_for_message(topic, String, timeout=10)

    print message

    return node == message.data
