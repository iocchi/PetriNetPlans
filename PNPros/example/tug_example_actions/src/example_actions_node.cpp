#include<tug_example_actions/example_actions_node.h>
namespace example_actions_node
{
ExampleActionsNode::ExampleActionsNode() :
    nh_(),
    fetch_box_action_server_(nh_,"fetch_box", boost::bind( &ExampleActionsNode::fetchBoxCB,this,_1),false),
    transport_box_action_server_(nh_,"transport_box", boost::bind( &ExampleActionsNode::transportBoxCB,this,_1),false),
    recover_not_fetching_box_action_server_(nh_,"recover_not_fetching_box", boost::bind( &ExampleActionsNode::recoverNotFetchingBoxCB,this,_1),false),
    recover_timed_out_action_server_(nh_,"recover_timed_out", boost::bind( &ExampleActionsNode::recoverTimedOutBoxCB,this,_1),false),
    put_box_action_server_(nh_,"put_box", boost::bind( &ExampleActionsNode::putBoxCB,this,_1),false),
    counter_for_fetch_box_(0)
{
    fetch_box_action_server_.start();
    transport_box_action_server_.start();
    recover_not_fetching_box_action_server_.start();
    recover_timed_out_action_server_.start();
    put_box_action_server_.start();

}

ExampleActionsNode::~ExampleActionsNode()
{}

void ExampleActionsNode::fetchBoxCB(const tug_example_msgs::FetchBoxGoalConstPtr &goal)
{
    ROS_INFO("Fetching Box");

    if (fetch_box_action_server_.isPreemptRequested() || !ros::ok())
    {
        ROS_ERROR("Fetching box execution preempted.");
        fetch_box_action_server_.setPreempted();
        return;
    }
    else
    {
        sleep(5.0);
        tug_example_msgs::FetchBoxResult fetching_box_result;

        if(counter_for_fetch_box_ == 15)
        {
            fetching_box_result.result = tug_example_msgs::FetchBoxResult::TIMED_OUT;
            fetch_box_action_server_.setAborted(fetching_box_result);
            ++counter_for_fetch_box_;
            return;
        }

        if(counter_for_fetch_box_%2 == 0)
        {
            fetching_box_result.result = tug_example_msgs::FetchBoxResult::OK;
            fetch_box_action_server_.setSucceeded(fetching_box_result);
            ++counter_for_fetch_box_;
            return;
        }
        else
        {
            fetching_box_result.result = tug_example_msgs::FetchBoxResult::NOT_OK;
            fetch_box_action_server_.setAborted(fetching_box_result);
            ++counter_for_fetch_box_;
            return;
        }
    }

}

void ExampleActionsNode::transportBoxCB(const tug_example_msgs::TransportBoxGoalConstPtr &goal)
{
    ROS_INFO("Transport Box");

    if (transport_box_action_server_.isPreemptRequested() || !ros::ok())
    {
        ROS_ERROR("Transport box execution preempted.");
        transport_box_action_server_.setPreempted();
        return;
    }
    else
    {
        sleep(5.0);
        tug_example_msgs::TransportBoxResult transport_box_result;
        transport_box_result.result = tug_example_msgs::TransportBoxResult::OK;
        transport_box_action_server_.setSucceeded(transport_box_result);
        return;
    }
}

void ExampleActionsNode::recoverNotFetchingBoxCB(const tug_example_msgs::RecoverNotFetchingBoxGoalConstPtr &goal)
{
    ROS_INFO("Recovery behaviour after not fetching box");

    if (recover_not_fetching_box_action_server_.isPreemptRequested() || !ros::ok())
    {
        ROS_ERROR("Recovery behaviour after not fetching box execution preempted.");
        recover_not_fetching_box_action_server_.setPreempted();
        return;
    }
    else
    {
        sleep(5.0);
        recover_not_fetching_box_action_server_.setSucceeded();
        return;
    }
}

void ExampleActionsNode::recoverTimedOutBoxCB(const tug_example_msgs::RecoverTimedOutGoalConstPtr &goal)
{
    ROS_INFO("Recovery behaviour after time out");

    if (recover_timed_out_action_server_.isPreemptRequested() || !ros::ok())
    {
        ROS_ERROR("Recovery behaviour after time out execution preempted.");
        recover_timed_out_action_server_.setPreempted();
        return;
    }
    else
    {
        sleep(5.0);
        recover_timed_out_action_server_.setSucceeded();
        return;
    }

}
void ExampleActionsNode::putBoxCB(const tug_example_msgs::PutBoxGoalConstPtr &goal)
{
    ROS_INFO("Putting Box");

    if (put_box_action_server_.isPreemptRequested() || !ros::ok())
    {
        ROS_ERROR("Putting Box execution preempted.");
        put_box_action_server_.setPreempted();
        return;
    }
    else
    {
        sleep(5.0);
        tug_example_msgs::PutBoxResult put_box_result;
        put_box_result.result = tug_example_msgs::PutBoxResult::OK;
        put_box_action_server_.setSucceeded(put_box_result);
        return;
    }
}

}

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc,argv,"example_actions_node");
        example_actions_node::ExampleActionsNode node;
        ros::spin();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
