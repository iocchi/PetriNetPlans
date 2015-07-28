#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <PNPros/PNPAction.h>
//#include <PNPros/PNPCondition.h>
//#include <PNPros/PNPActionServer.h>
#include <sensor_msgs/LaserScan.h>

#include <rp_action_msgs/TurnAction.h>

#include <boost/thread/thread.hpp>

#define radians(a) ((a)/180.0*M_PI)

std::string robotname="";

// defined in robotpose.cpp
bool getRobotPose(std::string robotname, double &x, double &y, double &th_rad);


using namespace std;

std::string turn_topic = "turn";
std::string movebase_topic = "move_base";

actionlib::SimpleActionClient<rp_action_msgs::TurnAction> *ac_turn = NULL;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_movebase = NULL;  


void start_gotopose(float GX, float GY, float GTh, bool *run) {

  if (ac_movebase==NULL) {
    // Define the action client (true: we want to spin a thread)
    ac_movebase = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(movebase_topic, true);  

    // Wait for the action server to come up
    while(!ac_movebase->waitForServer(ros::Duration(5.0))){
		    ROS_INFO("Waiting for move_base action server to come up");
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

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = GX;
  goal.target_pose.pose.position.y = GY;
  goal.target_pose.pose.orientation.z = sin(radians(GTh)/2);
  goal.target_pose.pose.orientation.w = cos(radians(GTh)/2);

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


// Action implementation

void init(string params, bool *run) {
  cout << "### Executing Init ... " << params << endl;
  // Set turn topic

  float GX=2.0;
  float GY=2.0;
  float GTh=0;

  start_gotopose(GX, GY, GTh, run);

  if (*run)
      cout << "### Finished Init " << endl;
  else
      cout << "### Aborted Init  " << endl;
}





void gotopose(string params, bool *run) {
  cout << "### Executing Gotopose ... " << params << endl;

  int i=params.find("_");	
  float GX=atof(params.substr(0,i).c_str());
  int j=params.find("_",i+1);
  float GY=atof(params.substr(i+1,j).c_str());
  float GTh=atof(params.substr(j+1).c_str());

  start_gotopose(GX, GY, GTh, run);

  if (*run)
    cout << "### Finished Gotopose " << endl;
  else
    cout << "### Aborted Gotopose  " << endl;
}

void home(string params, bool *run) 
{
  cout << "### Executing Home ... " << params << endl;

  float GX=2.0;
  float GY=2.0;
  float GTh=0;

  start_gotopose(GX, GY, GTh, run);

  if (*run)
    cout << "### Finished Home " << endl;
  else
    cout << "### Aborted Home  " << endl;
}

void wave(string params, bool *run) {
    cout << "### Executing Wave ... " << params << endl;
    
    cout << "HELLO FROM " << robotname << " !!!"<<endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    if (*run)
        cout << "### Finished Wave " << endl;
    else
        cout << "### Aborted Wave  " << endl;
}

void turn360(string params, bool *run) {
#if 0
    cout << "\033[22;31;1m### Executing turn360 ... " << params << "\033[0m" << endl;
    
    cout << "HELLO FROM " << robotname << " !!!"<<endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    if (*run)
        cout << "### Finished turn360 " << endl;
    else
        cout << "### Aborted turn360  " << endl;
#else
    // Set turn topic
    std::string turn_topic = "/"+robotname+"/turn";

    // Define the action client (true: we want to spin a thread)
    actionlib::SimpleActionClient<rp_action_msgs::TurnAction> ac(turn_topic, true);

    // Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for turn action server to come up");
    }

    // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
    ac.cancelAllGoals(); ros::Duration(1).sleep(); // wait 1 sec

    int counter = 0;
    
    while (counter++ != 3)
    {
      // Set the goal
      rp_action_msgs::TurnGoal goal;
      goal.target_angle = 120;  // deg
      goal.absolute_relative_flag = "REL";
      goal.max_ang_vel = 45.0;  // deg/s
      goal.name = robotname;

      // Send the goal
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      // Wait for termination
      while (!ac.waitForResult(ros::Duration(1.0))) {
	  ROS_INFO_STREAM("Running... [" << ac.getState().toString() << "]");
      }
      ROS_INFO_STREAM("Finished [" << ac.getState().toString() << "]");

      // Print result
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("Turn successful");
      else
	  ROS_INFO("Turn failed");

      // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
      ac.cancelAllGoals(); ros::Duration(1).sleep(); // wait 1 sec
    }
#endif
}

void sense1(string params, bool *run) {
  cout << "### Executing Sense1 ... " << params << endl;
}

int closeToHomeCond()
{
  double x, y, theta;
  
  if (getRobotPose(robotname,x,y,theta))
  {
    if ((fabs(x - 2) <= 4) && (fabs(y - 2) <= 4)) return 1;
    else return 0;
  }
  else return 0;
}
