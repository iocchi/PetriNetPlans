#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define radians(a) ((a)/180.0*M_PI)

class MyPNPActionServer : public PNPActionServer
{
private:

    int status;
    std::string movebase_topic;
    // Define the action client (true: we want to spin a thread)
    MoveBaseClient *ac;  

public:

    MyPNPActionServer() : PNPActionServer(), status(0), movebase_topic(""), ac(NULL)
    { 
        boost::thread t(boost::bind(&MyPNPActionServer::changeStatus, this));
    }

    // change the status used for condition every 10 seconds
    void changeStatus() {
        while (true) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(10000)); 
            status++;
        }
    }
    
    // This function is called in a separate thread for each action to be executed
    // Action implementation must check if run is true during execution, 
    // if not it must abort the action.
    // run must be used read-only!!!
    // The function must return only when the action is finished.
    virtual void actionExecutionThread(string robotname, string action_name, string action_params, bool *run)
    {
	if (ac==NULL) {
	    // Set move_base topic
	    movebase_topic = "/"+robotname+"/move_base";
	    ac = new MoveBaseClient(movebase_topic, true);
	    // Wait for the action server to come up
	    while(!ac->waitForServer(ros::Duration(3.0))){
		ROS_INFO("Waiting for move_base action server to come up");
	    }
	    ROS_INFO("Conneced to move_base %s", movebase_topic.c_str());
	}
	
        if (action_name=="init")
            init(action_params,run);
        else if (action_name=="gotopose")
            gotopose(action_params,run);
        else if (action_name=="home")
            home(action_params,run);
        else if (action_name=="wave")
            wave(action_params,run);
        else if (action_name=="sense1")
            sense1(action_params,run);
        else
            cout << "ERROR: UNKNOWN Action " << action_name << " " << endl;
    }

    // This function must return the truth value of a condition.
    // It is called each time PNP needs the evaluation of a condition
    int evalCondition(string condition) {
        int r = false;
        if (condition=="obstacle") {
            r = status%2==0;
        }
        cout << "+++ Evaluating condition " << condition << " -> " << r << endl;
        return r;
    }

    // Aux functions
    
    // Goto to target point with move_base client (GX,GY [m in /map frame], GTh [degrees])
    void goto_move_base(float GX, float GY, float GTh, bool *run) {
        // cout << "### Executing NEW Gotopose ... " << params << endl;
        
        // Set the goal (map frame)
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = GX;
        goal.target_pose.pose.position.y = GY;
        goal.target_pose.pose.orientation.z = sin(radians(GTh)/2);
        goal.target_pose.pose.orientation.w = cos(radians(GTh)/2);

        // Send the goal
        ROS_INFO("Sending goal %.1f %.1f %.1f",GX,GY,GTh);
        ac->sendGoal(goal);

        // Wait for termination
        while (!ac->waitForResult(ros::Duration(1.0)) && *run) {
            ROS_INFO("Running...");
        }

        // Print result
        if (*run) {
          cout << "### Finished goto " << endl;
          if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base reached the goal position.");
          else
            ROS_INFO("The base failed to reach the goal for some reason");
        }
        else
            cout << "### Aborted goto  " << endl;
    }
    
    
    // Action implementation

    void init(string params, bool *run) {
        ROS_INFO_STREAM("### Executing init ... " << params);
        goto_move_base(2,2,0,run);
    }

    
    
    void gotopose(string params, bool *run) {
        ROS_INFO_STREAM("### Executing gotopose ... " << params);
        
        float GX, GY, GTh;
        // Parse params (replace '_' with ' ')
        int cr=params.find_first_of("_");
        while (cr>0) {
            params.replace(cr,1," ");
            cr=params.find_first_of("_");
        }
        
        stringstream ss(params);
        ss >> GX; ss >> GY; ss >> GTh;
        // cout << "Target " << GX << "," << GY << "," << GTh << endl;
        
        goto_move_base(GX,GY,GTh,run);
    }

    void home(string params, bool *run) {
        ROS_INFO_STREAM("### Executing home ... " << params);
        goto_move_base(2,2,0, run);
    }

    void wave(string params, bool *run) {
        cout << "### Executing Wave ... " << params << endl;
        
        cout << "\033[31;40;1mSAY: Hello obstacle. How are you?\033[0m" << endl;
        
    }

    void sense1(string params, bool *run) {
        cout << "### Executing Sense1 ... " << params << endl;
        
        int cnt=20;
        while (*run && --cnt>0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(250));
        }

        if (*run)
            cout << "### Finished Sense1 " << endl;
        else
            cout << "### Aborted Sense1  " << endl;
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

