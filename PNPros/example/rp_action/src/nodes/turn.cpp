#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <rp_action_msgs/TurnAction.h>

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)


// defined in robotpose.cpp
bool getRobotPose(std::string robotname, double &x, double &y, double &th_rad);


inline double norm180(double a) {
  while (a>180) a = a-360;
  while (a<=-180) a = a+360;
  return a;
}

inline double normPI(double a) {
  while (a>M_PI) a = a-2*M_PI;
  while (a<=-M_PI) a = a+2*M_PI;
  return a;
}

inline double norm360(double a) {
  while (a>=360) a = a-360;
  while (a<=-360) a = a+360;
  return a;
}

void doTurn(ros::Publisher &cmd_vel_pub, double target_angle_deg, 
            std::string absolute_relative_flag, double max_ang_vel_deg, std::string robotname)
{
    ROS_INFO_STREAM("Action turn: executing ...");
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0; cmd.linear.y = 0; 

    double x,y,th_rad,th_deg,max_ang_vel,target_angle;

    getRobotPose(robotname,x,y,th_rad); th_deg=DEG(th_rad); max_ang_vel=RAD(max_ang_vel_deg);

    if (absolute_relative_flag=="REL") {
        target_angle_deg = th_deg + target_angle_deg;
    }
    target_angle = RAD(target_angle_deg);
    printf("Robot pose: %.1f %.1f %.3f - Target: %.3f\n",x,y,th_rad,target_angle);

    double Kp=1.0;
    double min_diff_angle=RAD(5); // rad
    double adist = normPI(target_angle-th_rad); // rad
    while (fabs(adist)>min_diff_angle) {
        cmd.angular.z = std::min(Kp*adist, max_ang_vel);
        printf("Robot pose: %.1f %.1f %.3f - Target: %.3f - adist: %.3f - avel: %.3f\n",
                x,y,th_rad,target_angle,adist,cmd.angular.z);
        cmd_vel_pub.publish(cmd);
        ros::Duration(0.1).sleep(); // wait ...    
        getRobotPose(robotname,x,y,th_rad);
        adist = normPI(target_angle-th_rad); // rad
    }

    cmd.angular.z = 0;
    cmd_vel_pub.publish(cmd);
    ros::Duration(0.1).sleep(); // wait ...    

    ROS_INFO_STREAM("Action turn: finished");
}

// ********************    S E R V E R    ***********************

class TurnActionServer {

protected:

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    std::string action_name;
    actionlib::SimpleActionServer<rp_action_msgs::TurnAction> turn_server;


public:

    TurnActionServer(std::string name) : 
        action_name(name),
        turn_server(nh, action_name, boost::bind(&TurnActionServer::executeCB, this, _1), false)
    { 
        //set up the publisher for the cmd_vel topic
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    ~TurnActionServer() { }

    void start() {
        // Start the server
        turn_server.start();
        ROS_INFO_STREAM("Turn Action Server started!!!"); 
    }

    void executeCB(const rp_action_msgs::TurnGoalConstPtr& goal)  {
        // Run action (wait until it finishes)
        ROS_INFO_STREAM("Starting action: turn " << goal->target_angle << " " << 
              goal->absolute_relative_flag << " " << goal->max_ang_vel);
        doTurn(cmd_vel_pub,goal->target_angle, goal->absolute_relative_flag, goal->max_ang_vel, goal->name);

        // Set result
        turn_server.setSucceeded();
    }
};


// ********************    C L I E N T    ***********************

void turn_action_client(std::string robotname, double GTh, std::string absrel) {

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

    // Set the goal
    rp_action_msgs::TurnGoal goal;
	goal.target_angle = GTh;  // deg
    goal.absolute_relative_flag = absrel;
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



// **********************    M A I N    *************************


int main(int argc, char** argv)  {

  if ( (argc<2) || ((std::string(argv[1])=="-client") && (argc<4)) ) {
  	std::cout << "Use: " << argv[0] << " -server" << std::endl; 
  	std::cout << "  or " << argv[0] << " -client <robotname> <Theta [DEG]> <ABS|REL>" << std::endl; 
    exit(-1);
  }

  if (std::string(argv[1])=="-server") {
      // Init ROS node
      ros::init(argc, argv, "turn_action_server");   
      // Start action server  
      TurnActionServer server("turn");
      server.start();
      ros::spin();
  }
  else if (std::string(argv[1])=="-client") {
      // Read args
      std::string robotname = std::string(argv[2]);
      double GTh = atof(argv[3]);
      std::string absrel = std::string(argv[4]);
      // Init ROS node
      std::ostringstream ss;
      ss << "turn_" << robotname << "_" << fabs(GTh);
      std::string nodename = ss.str();
      ros::init(argc, argv, nodename);
      // Start client
      turn_action_client(robotname,GTh,absrel);
  }
  return 0;
}

