#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  if (argc<2) {
  	std::cout << "Use: " << argv[0] << " <robotname>" << std::endl; 
    exit(-1);
  }

  std::string robotname = std::string(argv[1]);

  // Init ROS node
  std::string nodename = "stopmove_"+robotname;
  ros::init(argc, argv, nodename);

  // Set move_base topic
  std::string movebase_topic = "/"+robotname+"/move_base";

  // Define the action client (true: we want to spin a thread)
  MoveBaseClient ac(movebase_topic, true);  

  // Wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Cancel all goals
  ac.cancelAllGoals();

  return 0;
}

