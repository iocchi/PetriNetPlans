#include <sstream>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define radians(a) ((a)/180.0*M_PI)

int main(int argc, char** argv){

  if (argc<5) {
  	std::cout << "Use: " << argv[0] << " <robotname> <X> <Y> <Theta [DEG]>" << std::endl; 
    exit(-1);
  }

  std::string robotname = std::string(argv[1]);
  double GX = atof(argv[2]), GY = atof(argv[3]), GTh = atof(argv[4]);

  // Init ROS node
  std::ostringstream ss;
  ss << "gotopose_" << robotname << "_" << GX << "_" << GY << "_" << GTh;
  std::string nodename = ss.str();
  ros::init(argc, argv, nodename);

  // Set move_base topic
  std::string movebase_topic = "/"+robotname+"/move_base";

  // Define the action client (true: we want to spin a thread)
  MoveBaseClient ac(movebase_topic, true);  

  // Wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for move_base action server to come up");
  }

  // Read time
  double secs =ros::Time::now().toSec();
  while (secs==0) {  // NEEDED OTHERWISE CLOCK WILL BE 0 AND GOAL_ID IS NOT SET CORRECTLY
    ROS_ERROR_STREAM("Time is null: " << ros::Time::now());
    ros::Duration(1.0).sleep();
	  secs =ros::Time::now().toSec();
  }

  // Set the goal (MAP frame)
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = GX;
  goal.target_pose.pose.position.y = GY;
  goal.target_pose.pose.orientation.z = sin(radians(GTh)/2);
  goal.target_pose.pose.orientation.w = cos(radians(GTh)/2);

  // Send the goal
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait for termination
  while (!ac.waitForResult(ros::Duration(1.0))) {
	  ROS_INFO("Running...");
  }

  // Print result
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     ROS_INFO("Hooray, the base reached the goal position.");
  else
     ROS_INFO("The base failed to reach the goal for some reason");

  return 0;
}

