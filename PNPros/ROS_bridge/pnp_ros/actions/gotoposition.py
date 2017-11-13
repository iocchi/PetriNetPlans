import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from math import radians

def create_nav_goal(x, y, yaw):
    """Create a MoveBaseGoal with x, y position and yaw rotation (in degrees). Returns a MoveBaseGoal"""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    # position
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    # orientation expressed in quaternion
    angle = radians(yaw)
    quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
    goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

    return goal

def actionThread_exec(cancel_event, params):
    # initialize stuff
    #rospy.init_node("goto_action")
    nav_ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    rospy.loginfo("Connecting to /move_base AS...")
    nav_ac.wait_for_server()
    rospy.loginfo("Connected.")

    # create navigation goal
    param_elems = params.split(',')
    if len(param_elems) == 3:
        nav_goal = create_nav_goal(float(param_elems[0]), float(param_elems[1]), float(param_elems[2]))
    else:
        nav_goal = create_nav_goal(.0, .0, 0)

    # send navigation goal
    nav_ac.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_ac.wait_for_result()
    nav_res = nav_ac.get_result()
    rospy.loginfo("Result " + str(nav_res))
    nav_state = nav_ac.get_state()
    rospy.loginfo("State: " + str(nav_state))


    # loop until cancel event
    #while not cancel_event.isSet():
    #    print "diocan"
    #    time.sleep(5)
