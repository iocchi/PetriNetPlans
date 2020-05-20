#include <sstream>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)

// defined in robotpose.cpp
bool getRobotPose(std::string robotname, double &x, double &y, double &th_rad);

std::string nsrobot = "";  // robot namespace

void setNamespace(string ns) {
    nsrobot = ns;
}

void exec_gotopose(std::string robotname, float GX, float GY, float GTh, bool *run) {

  // Set move_base topic
  std::string movebase_topic = "move_base";
  if (nsrobot!="")
    movebase_topic = nsrobot+"/"+movebase_topic;

  // Set map frame
  string map_frame = "map";
  if (robotname!="" && robotname!="none")  // multi-robot: use robotname prefix
    map_frame = robotname + "/" + map_frame;
  

  // Define the action client
  MoveBaseClient *ac_movebase = NULL;  

  if (ac_movebase==NULL) {
    // Define the action client (true: we want to spin a thread)
    ac_movebase = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(movebase_topic, true);

    // Wait for the action server to come up
    ROS_INFO_STREAM("Waiting for move_base action server to come up (" << movebase_topic << ")");
    while(!ac_movebase->waitForServer(ros::Duration(5.0))){
        ROS_INFO_STREAM("Waiting for move_base action server to come up (" << movebase_topic << ")");
    }
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

  goal.target_pose.header.frame_id = map_frame;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = GX;
  goal.target_pose.pose.position.y = GY;
  goal.target_pose.pose.orientation.z = sin(RAD(GTh)/2);
  goal.target_pose.pose.orientation.w = cos(RAD(GTh)/2);

  // Send the goal
  ROS_INFO("Sending goal");
  ac_movebase->sendGoal(goal);

  // Wait for termination
  double d_threshold=0.5, d=d_threshold+1.0;
  while (!ac_movebase->waitForResult(ros::Duration(0.5)) && (*run) && (d>d_threshold)) {
    // ROS_INFO("Running...");
    double RX,RY,RTH;
    if (getRobotPose(robotname, RX, RY, RTH))
      d = fabs(GX-RX)+fabs(GY-RY);
  }

  // Print result
  if (!(*run))
    ROS_INFO("External interrupt!!!");
  else if (d<=d_threshold) 
    ROS_INFO("Target reached (Internal check)");
  else if (ac_movebase->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
 		ROS_INFO("The base failed to reach the move_base goal for some reason");
  else
    ROS_INFO("!move_base goal reached!");

  // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
  ac_movebase->cancelAllGoals(); ros::Duration(1).sleep(); // wait 1 sec
}



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

  if (robotname!="" && robotname!="none")  // multi-robot: use robotname prefix
    setNamespace(robotname);

  bool run = true;
  exec_gotopose(robotname, GX, GY, GTh, &run);

  return 0;
}

