#include <tug_example_pnp_server/example_pnp_server_node.h>

ExamplePNPServer::ExamplePNPServer():
    PNPActionServer(),
    nh_(),
    fetch_box_ac_("fetch_box",true),
    transport_box_ac_("transport_box",true),
    put_box_ac_("put_box",true),
    recovery_timed_out_ac_("recover_timed_out",true),
    recovery_not_fetching_box_ac_("recover_not_fetching_box",true),
    runable_(true)
{
    //fuction map to map places names to c++ functions
    function_map_.insert(std::pair<std::string, boost::function<void(bool*)> >("FetchBox", boost::bind(&ExamplePNPServer::fetchBox,this,_1)));
    function_map_.insert(std::pair<std::string, boost::function<void(bool*)> >("TransportBox", boost::bind(&ExamplePNPServer::transportBox,this,_1)));
    function_map_.insert(std::pair<std::string, boost::function<void(bool*)> >("Put", boost::bind(&ExamplePNPServer::putBox,this,_1)));
    function_map_.insert(std::pair<std::string, boost::function<void(bool*)> >("Recovery", boost::bind(&ExamplePNPServer::recoveryNotFetchingBox,this,_1)));
    function_map_.insert(std::pair<std::string, boost::function<void(bool*)> >("RecoveryTimeOut", boost::bind(&ExamplePNPServer::recoveryTimedOut,this,_1)));
    function_map_.insert(std::pair<std::string, boost::function<void(bool*)> >("init", boost::bind(&ExamplePNPServer::init,this,_1)));


    //fetch box state
    fetch_box_state_ = -1;
}

ExamplePNPServer::~ExamplePNPServer()
{
}

int ExamplePNPServer::evalCondition(string condition)
{
    ROS_INFO_STREAM("Evaluating condition "<< condition);
    if((condition == "ok")&&(fetch_box_state_ == static_cast<int>(tug_example_msgs::FetchBoxResult::OK)))
    {
        fetch_box_state_ = -1;
        return 1;
    }

    if((condition == "not_ok")&&(fetch_box_state_ == static_cast<int>(tug_example_msgs::FetchBoxResult::NOT_OK)))
    {
        fetch_box_state_ = -1;
        return 1;
    }

    if((condition == "timed_out")&&(fetch_box_state_ == static_cast<int>(tug_example_msgs::FetchBoxResult::TIMED_OUT)))
    {
        fetch_box_state_ = -1;
        return 1;
    }



    return 0;
}

void ExamplePNPServer::actionExecutionThread(string robotname, string action_name, string action_params, bool *run)
{
    ROS_INFO_STREAM("actionExecutionThread called " << action_name << " run is "<< *run);

    std::map<std::string, boost::function<void(bool*)> >::iterator function_map_it = function_map_.find(action_name);
    if(function_map_it == function_map_.end())
    {
        ROS_ERROR_STREAM("No function " << action_name << " is defined.");
    }
    else
    {
        function_map_it->second(&runable_);
    }
}

void ExamplePNPServer::fetchBox(bool *run)
{
    if(!(*run))
        return;



    // Wait for the action server
    while(!fetch_box_ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for fetch box action server to come up");
    }

    //sending goal
    tug_example_msgs::FetchBoxGoal goal;
    fetch_box_ac_.sendGoal(goal);

    //waiting for result
    while (!fetch_box_ac_.waitForResult(ros::Duration(1.0)))
    {
        ROS_INFO("Fetching Box...");
    }

    // Print result
    if (fetch_box_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Fetch Box Succeded");
        if(fetch_box_ac_.getResult()->result == tug_example_msgs::FetchBoxResult::OK)
        {
            fetch_box_state_ = static_cast<int>(tug_example_msgs::FetchBoxResult::OK);
        }
    }
    else
    {
        ROS_INFO("Fetch Box failed");
        if(fetch_box_ac_.getResult()->result == tug_example_msgs::FetchBoxResult::NOT_OK)
        {
           fetch_box_state_ = static_cast<int>(tug_example_msgs::FetchBoxResult::NOT_OK);
        }
        else if(fetch_box_ac_.getResult()->result == tug_example_msgs::FetchBoxResult::TIMED_OUT)
        {
           fetch_box_state_ = static_cast<int>(tug_example_msgs::FetchBoxResult::TIMED_OUT);
        }

    }

}

void ExamplePNPServer::transportBox(bool *run)
{
    if(!(*run))
        return;

    // Wait for the action server
    while(!transport_box_ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for transport box action server to come up");
    }

    //sending goal
    tug_example_msgs::TransportBoxGoal goal;
    transport_box_ac_.sendGoal(goal);

    //waiting for result
    while (!transport_box_ac_.waitForResult(ros::Duration(1.0)))
    {
        ROS_INFO("Transporting Box...");
    }

    // Print result
    if (transport_box_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Transporting Box Succeded");
    }
    else
    {
        ROS_INFO("Transporting Box failed");
    }


}

void ExamplePNPServer::putBox(bool *run)
{
    if(!(*run))
        return;

    // Wait for the action server
    while(!put_box_ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for put box action server to come up");
    }

    //sending goal
    tug_example_msgs::PutBoxGoal goal;
    put_box_ac_.sendGoal(goal);

    //waiting for result
    while (!put_box_ac_.waitForResult(ros::Duration(1.0)))
    {
        ROS_INFO("Putting Box...");
    }

    // Print result
    if (put_box_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Putting Box Succeded");
    }
    else
    {
        ROS_INFO("Putting Box failed");
    }
}

void ExamplePNPServer::recoveryNotFetchingBox(bool *run)
{
    if(!(*run))
        return;

    // Wait for the action server
    while(!recovery_not_fetching_box_ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for recovery not fetching box action server to come up");
    }

    //sending goal
    tug_example_msgs::RecoverNotFetchingBoxGoal goal;
    recovery_not_fetching_box_ac_.sendGoal(goal);

    //waiting for result
    while (!recovery_not_fetching_box_ac_.waitForResult(ros::Duration(1.0)))
    {
        ROS_INFO("Recovering not fetching box...");
    }

    // Print result
    if (recovery_not_fetching_box_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Recovering not fetching box Succeded");
    }
    else
    {
        ROS_INFO("Recovering not fetching box failed");
    }
}

void ExamplePNPServer::recoveryTimedOut(bool *run)
{
    if(!(*run))
        return;

    // Wait for the action server
    while(!recovery_timed_out_ac_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for recovery timed out action server to come up");
    }

    //sending goal
    tug_example_msgs::RecoverTimedOutGoal goal;
    recovery_timed_out_ac_.sendGoal(goal);

    //waiting for result
    while (!recovery_timed_out_ac_.waitForResult(ros::Duration(1.0)))
    {
        ROS_INFO("Recovering timed out...");
    }

    // Print result
    if (recovery_timed_out_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Recovering timed out Succeded");
    }
    else
    {
        ROS_INFO("Recovering timed out failed");
    }
}

void ExamplePNPServer::init(bool *run)
{
    ROS_INFO("Start Init of PNP Server");

    ROS_INFO("End Init of PNP Server");
}







int main(int argc, char** argv)
{
    ros::init(argc, argv, "mypnpas");


    ExamplePNPServer test_server;
    test_server.start();


    ros::spin();

    return 0;
}
