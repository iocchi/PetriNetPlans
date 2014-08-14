#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>



#include <boost/thread/thread.hpp>


class MyPNPActionServer : public PNPActionServer
{
private:

    int status;

public:

    MyPNPActionServer() : PNPActionServer(), status(0)
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
    void actionExecutionThread(string action_name, string action_params, bool *run)
    {
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
            cout << "??? UNKNOWN Action " << action_name << " ??? " << endl;
    }

    // This function must return the truth value of a condition.
    // It is called each time PNP needs the evaluation of a condition
    int evalCondition(string condition) {
        int r = false;
        if (condition=="conditionA") {
            r = status%2==0;
        }
        cout << "+++ Evaluating condition " << condition << " -> " << r << endl;
        return r;
    }

    // Action implementation

    void init(string params, bool *run) {
        cout << "### Executing Init ... " << params << endl;
        int cnt=5;
        while (*run && --cnt>0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(250));
        }
        if (*run)
            cout << "### Finished Init " << endl;
        else
            cout << "### Aborted Init  " << endl;
    }

    void gotopose(string params, bool *run) {
        cout << "### Executing Gotopose ... " << params << endl;
        int cnt=20;
        while (*run && --cnt>0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(250));
        }
        if (*run)
            cout << "### Finished Gotopose " << endl;
        else
            cout << "### Aborted Gotopose  " << endl;
    }

    void home(string params, bool *run) {
        cout << "### Executing Home ... " << params << endl;
        int cnt=10;
        while (*run && --cnt>0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(250));
        }
        if (*run)
            cout << "### Finished Home " << endl;
        else
            cout << "### Aborted Home  " << endl;
    }

    void wave(string params, bool *run) {
        cout << "### Executing Wave ... " << params << endl;
        
        int cnt=2;
        while (*run && --cnt>0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(250));
        }
        
        if (*run)
            cout << "### Finished Wave " << endl;
        else
            cout << "### Aborted Wave  " << endl;
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

