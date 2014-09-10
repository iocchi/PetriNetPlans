#ifndef __PNP_ACTION_SERVER_H__
#define __PNP_ACTION_SERVER_H__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_msgs/PNPLastEvent.h>
#include <pnp_msgs/PNPClearBuffer.h>
#include <pnp_msgs/PNPGetVariableValue.h>

#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

// Timing for event buffer
#define TIME_THRESHOLD 3
#define REMEMBERING_TIME 60

typedef void (*action_fn_t)(std::string, bool *); 
typedef void (*MRaction_fn_t)(std::string, std::string, bool *); 

#define DECLARE_ACTION_IMPLEMENTATION(f) \
void f(string params, bool *run);

typedef actionlib::ActionServer<pnp_msgs::PNPAction> PNPAS;

struct Event
{
  time_t time;
  std::string eventName;
  std::string parameter;
};


using namespace std;

class PNPActionServer
{

protected:
    ros::NodeHandle nh;
    // action server
    PNPAS as;
    PNPAS::GoalHandle current_gh;
    pnp_msgs::PNPGoal goal;
    // messages to published feedback/result
    pnp_msgs::PNPFeedback feedback;
    pnp_msgs::PNPResult result;

    // condition evaluation
    ros::ServiceServer cond_service;
    ros::ServiceServer getEvent_service;
    ros::ServiceServer clearBuffer_service;
    ros::ServiceServer getVarValue_service;
    ros::Subscriber event_topic_sub;

    boost::mutex state_mutex;
    boost::mutex run_mutex;
    boost::mutex eventBuffer_mutex;

    vector<Event> eventBuffer; 
    
    map<string,bool> run;
    map<string,action_fn_t> global_PNPROS_action_fns;
    map<string,MRaction_fn_t> global_PNPROS_MRaction_fns;
    map<string,string> global_PNPROS_variables;

public:

    PNPActionServer();
    ~PNPActionServer();

    void start();

protected:

    // For registering and retrieving action functions
    void register_action(string actionname, action_fn_t actionfn);
    action_fn_t get_action_fn(string actionname);
    void register_MRaction(string actionname, MRaction_fn_t actionfn); // multi-robot version
    MRaction_fn_t get_MRaction_fn(string actionname);

    // Action execution
    void goalCallback(PNPAS::GoalHandle gh);
    // void cancelCallback(PNPAS::GoalHandle gh)
    void ActionExecutionThread(PNPAS::GoalHandle gh);
    void CancelAction(string robotname, string action_name, string action_params);
    virtual void actionExecutionThread(string robotname, string action_name, string action_params, bool *run);

    // Condition evaluation
    void addEvent_callback(const std_msgs::String::ConstPtr& msg);
    int check_for_event(string cond);
    void remove_old_elements();
    // Can be redefined by actual implementation
    // 1: true, 0: false, -1: unknown
    virtual int evalCondition(string cond);
    bool EvalConditionWrapper(pnp_msgs::PNPCondition::Request  &req,
             pnp_msgs::PNPCondition::Response &res);
    bool GetEventStartingWith(pnp_msgs::PNPLastEvent::Request  &req,
             pnp_msgs::PNPLastEvent::Response &res);
    bool ClearBuffer(pnp_msgs::PNPClearBuffer::Request  &req,
         pnp_msgs::PNPClearBuffer::Response &res);
    bool GetVariableValue(pnp_msgs::PNPGetVariableValue::Request  &req,
         pnp_msgs::PNPGetVariableValue::Response &res);
    
    vector<string> split_condition(string);
    vector<string> get_variables_values(vector<std::string> );
    string get_variable_value(string);
    string replace_vars_with_values(string);
    bool well_formatted_with_variables(string);
    void update_variable_with_value(string, string);
    void internal_clear_buffer();

};

#endif
