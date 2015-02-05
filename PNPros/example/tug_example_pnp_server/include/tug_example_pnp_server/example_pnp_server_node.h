#ifndef EXAMPLE_PNP_SERVER__H_
#define EXAMPLE_PNP_SERVER__H_

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <boost/thread/thread.hpp>
#include <actionlib/server/simple_action_server.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>
#include <actionlib/client/simple_action_client.h>

#include <tug_example_msgs/FetchBoxAction.h>
#include <tug_example_msgs/TransportBoxAction.h>
#include <tug_example_msgs/RecoverNotFetchingBoxAction.h>
#include <tug_example_msgs/RecoverTimedOutAction.h>
#include <tug_example_msgs/PutBoxAction.h>

#include <pnp/pnpfwd.h>

typedef actionlib::SimpleActionClient<tug_example_msgs::FetchBoxAction> FetchBoxActionClient;
typedef actionlib::SimpleActionClient<tug_example_msgs::TransportBoxAction> TransportBoxActionClient;
typedef actionlib::SimpleActionClient<tug_example_msgs::RecoverNotFetchingBoxAction> RecoverNotFetchingBoxActionClient;
typedef actionlib::SimpleActionClient<tug_example_msgs::RecoverTimedOutAction> RecoverTimedOutActionClient;
typedef actionlib::SimpleActionClient<tug_example_msgs::PutBoxAction> PutBoxActionClient;



class ExamplePNPServer : public PNPActionServer
{
private:
    ros::NodeHandle nh_;

    //action clients
    FetchBoxActionClient fetch_box_ac_;
    TransportBoxActionClient transport_box_ac_;
    PutBoxActionClient put_box_ac_;
    RecoverNotFetchingBoxActionClient recovery_not_fetching_box_ac_;
    RecoverTimedOutActionClient recovery_timed_out_ac_;

    //states
    int fetch_box_state_;

    //map for function pointer
    std::map<std::string, boost::function<void(bool*)> > function_map_;

    bool runable_;



public:
    ExamplePNPServer();
    ~ExamplePNPServer();

    //pnp functions
    int evalCondition(std::string condition);
    virtual void actionExecutionThread(string robotname, string action_name, string action_params, bool *run);

    //function to be executed on different places
    void fetchBox(bool *run);
    void transportBox(bool *run);
    void putBox(bool *run);
    void recoveryNotFetchingBox(bool *run);
    void recoveryTimedOut(bool *run);
    void init(bool *run);


    void setRun(bool run)
    {
        runable_ = run;
    }



};




#endif
