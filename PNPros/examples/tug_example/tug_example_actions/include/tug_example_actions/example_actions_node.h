#ifndef EXAMPLE_ACTION_NODE__H_
#define EXAMPLE_ACTION_NODE__H_

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <actionlib/server/simple_action_server.h>

#include <tug_example_msgs/FetchBoxAction.h>
#include <tug_example_msgs/TransportBoxAction.h>
#include <tug_example_msgs/RecoverNotFetchingBoxAction.h>
#include <tug_example_msgs/RecoverTimedOutAction.h>
#include <tug_example_msgs/PutBoxAction.h>




typedef actionlib::SimpleActionServer<tug_example_msgs::FetchBoxAction> FetchBoxActionServer;
typedef actionlib::SimpleActionServer<tug_example_msgs::TransportBoxAction> TransportBoxActionServer;
typedef actionlib::SimpleActionServer<tug_example_msgs::RecoverNotFetchingBoxAction> RecoverNotFetchingBoxActionServer;
typedef actionlib::SimpleActionServer<tug_example_msgs::RecoverTimedOutAction> RecoverTimedOutActionServer;
typedef actionlib::SimpleActionServer<tug_example_msgs::PutBoxAction> PutBoxActionServer;

namespace example_actions_node
{
class ExampleActionsNode
{
public:
    ExampleActionsNode();

    virtual ~ExampleActionsNode();
private:

    ros::NodeHandle nh_;

    //action servers
    FetchBoxActionServer fetch_box_action_server_;
    TransportBoxActionServer transport_box_action_server_;
    RecoverNotFetchingBoxActionServer recover_not_fetching_box_action_server_;
    RecoverTimedOutActionServer recover_timed_out_action_server_;
    PutBoxActionServer put_box_action_server_;

    //variables for actions server
    int counter_for_fetch_box_;

    //action callbacks
    void fetchBoxCB(const tug_example_msgs::FetchBoxGoalConstPtr &goal);
    void transportBoxCB(const tug_example_msgs::TransportBoxGoalConstPtr &goal);
    void recoverNotFetchingBoxCB(const tug_example_msgs::RecoverNotFetchingBoxGoalConstPtr &goal);
    void recoverTimedOutBoxCB(const tug_example_msgs::RecoverTimedOutGoalConstPtr &goal);
    void putBoxCB(const tug_example_msgs::PutBoxGoalConstPtr &goal);

};
}


#endif
