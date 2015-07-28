#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>

#include "MyActions.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define radians(a) ((a)/180.0*M_PI)

class MyPNPActionServer : public PNPActionServer
{
private:

    ros::NodeHandle handle;
    ros::Publisher event_pub;
    ros::Subscriber laser_sub;
    
    //int status;
    //std::string movebase_topic;
    // Define the action client (true: we want to spin a thread)
    //MoveBaseClient *ac;  

public:

    MyPNPActionServer() : PNPActionServer() //, status(0), movebase_topic(""), ac(NULL)
    { 
        // boost::thread t(boost::bind(&MyPNPActionServer::changeStatus, this));
	event_pub = handle.advertise<std_msgs::String>("PNPConditionEvent", 10); 
	laser_sub = handle.subscribe("scan", 10, &MyPNPActionServer::laser_callback, this);

	// robotname external defined in MyActions.h/cpp
	handle.param("robot_name",robotname,std::string(""));
	ROS_INFO("ROBOTNAME: %s",robotname.c_str());
	
	register_action("init",&init);
	register_action("gotopose",&gotopose);
	register_action("home",&home);
	register_action("wave",&wave);
	register_action("sense1",&sense1);
	register_action("turn360",&turn360);
	
    }
    
    virtual int evalCondition(string cond)
    {
      if (cond == "closeToHome")
      {
	int res = closeToHomeCond();
	
	cerr << "\033[22;34;1mCloseToHome: " << ((res == 1) ? "true" : "false") << "\033[0m" << endl;
	
	return res;
      }
      
      return PNPActionServer::evalCondition(cond);
    }
    
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      std::vector<float> scans;
      scans=std::vector<float>(msg->ranges);
      if (scans[scans.size()/2]<1.0) {
	std_msgs::String cond;
	cond.data = "obstacle";
	event_pub.publish(cond);
      }
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mypnpas");

  MyPNPActionServer mypnpas;
  mypnpas.start();
  ros::spin();

  return 0;
}

