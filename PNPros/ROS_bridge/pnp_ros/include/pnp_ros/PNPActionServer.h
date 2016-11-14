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
#include <pnp_msgs/PNPSetVariableValue.h>

#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

// Topics name
#define TOPIC_PLANTOEXEC "planToExec"
#define TOPIC_PNPACTIVEPLACES "pnp/currentActivePlaces"
#define TOPIC_PNPCONDITION "PNPConditionEvent"
#define PARAM_PNPCONDITIONBUFFER "PNPconditionsBuffer/"

// Timing for event buffer
#define TIME_THRESHOLD 2
#define REMEMBERING_TIME 60

typedef void (*action_fn_t)(std::string, bool *); // action name, run flag
typedef void (*MRaction_fn_t)(std::string, std::string, bool *);  // robot name, action name, run flag
typedef boost::function<void(std::string, bool *)> boost_action_fn_t;
typedef boost::function<void(std::string, std::string, bool *)> boost_MRaction_fn_t;

typedef actionlib::ActionServer<pnp_msgs::PNPAction> PNPAS;

using namespace std;



struct Event
{
  time_t time;
  std::string eventName;
  std::string parameter;
};





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
    ros::ServiceServer setVarValue_service;
    ros::Subscriber event_topic_sub, active_places_sub;
    ros::Publisher plantoexec_pub;

    boost::mutex state_mutex;
    boost::mutex run_mutex;
    boost::mutex eventBuffer_mutex;

    vector<Event> eventBuffer; 
    
    map<string,bool> run;
    map<string,double> starttime; // ROS Time in seconds
    map<string,boost_action_fn_t> global_PNPROS_action_fns;
    map<string,boost_MRaction_fn_t> global_PNPROS_MRaction_fns;
    map<string,string> global_PNPROS_variables;

	map<string,int> ConditionCache;

public:

    PNPActionServer();
    ~PNPActionServer();

    void start();

protected:
    //--------------------------------------------------------------------------
    // Callbacks; can be overridden in subclasses:
    //--------------------------------------------------------------------------

    /**
     * Checks a condition.
     * @param cond  name of the condition, as defined in the plan. The default
     * implementation always returns -1.
     * @return 1 if cond is true,
     *         0 if cond is false, and
     *         -1 if cond is unknown.
     */
    virtual int evalCondition(std::string cond);

    /**
     * Called when an action (i.e., a place with ".ex" in its name) is started
     * (i.e., a "start" transition is encountered). The default implementation
     * does nothing.
     * @param robot   name of the robot in multi-robot plans.
     * @param action  name of the action (name of the place without the ".ex").
     * @param params  optional parameters (anything after a "_" in the place's
     *                name).
     */
    virtual void actionStart(const std::string & robot,
                             const std::string & action,
                             const std::string & params);

    /**
     * The action's "main" function. Called in a separate thread, which is
     * started after actionStart() returns. May take an arbitrary amount of
     * time, but should return fast when *run becomes false, which happens
     * when the action is ended or interrupted.
     * @param robot   name of the robot in multi-robot plans.
     * @param action  name of the action (name of the place without the ".ex").
     * @param params  optional parameters (anything after a "_" in the place's
     *                name).
     * @param run     indicates if the action may continue running or should
     *                stop.
     */
    virtual void actionExecutionThread(std::string robot, std::string action,
                                       std::string params, bool * run);

    /**
     * Called when an action is ended (i.e., a "end" transition is encountered).
     * This is called right before *run in actionExecutionThread() is set to
     * false. The default implementation does nothing.
     * @param robot   name of the robot in multi-robot plans.
     * @param action  name of the action (name of the place without the ".ex").
     * @param params  optional parameters (anything after a "_" in the place's
     *                name).
     */
    virtual void actionEnd(const std::string & robot,
                           const std::string & action,
                           const std::string & params);

    /**
     * Called when an action is interrupted (i.e., a "interrupt" transition is
     * encountered). This is called right before *run in actionExecutionThread()
     * is set to false. The default implementation does nothing.
     * @param robot   name of the robot in multi-robot plans.
     * @param action  name of the action (name of the place without the ".ex").
     * @param params  optional parameters (anything after a "_" in the place's
     *                name).
     */
    virtual void actionInterrupt(const std::string & robot,
                                 const std::string & action,
                                 const std::string & params);

    //--------------------------------------------------------------------------



    // For registering and retrieving action functions
    void register_action(string actionname, action_fn_t actionfn);

    void register_MRaction(string actionname, MRaction_fn_t actionfn); // multi-robot version


    template<class T>
    void register_action(string actionname, void(T::*fp)(std::string, bool *), T* obj)  {
        cout << "PNPROS:: REGISTERING ACTION " << actionname << " (class method)" << endl;
        global_PNPROS_action_fns[actionname] = boost::bind(fp, obj, _1, _2);
    }

    template<class T>
    void register_MRaction(string actionname, void(T::*fp)(std::string, std::string, bool *), T* obj)  {
        cout << "PNPROS:: REGISTERING MR ACTION " << actionname << " (class method)" << endl;
        global_PNPROS_MRaction_fns[actionname] = boost::bind(fp, obj, _1, _2, _3);
    }

    boost_action_fn_t get_action_fn(string actionname);
    boost_MRaction_fn_t get_MRaction_fn(string actionname);

    // Action execution
    void goalCallback(PNPAS::GoalHandle gh);
    // void cancelCallback(PNPAS::GoalHandle gh)
    void ActionExecutionThread(PNPAS::GoalHandle gh);
    void CancelAction(string robotname, string action_name, string action_params);

    // Condition evaluation
    void addEvent_callback(const std_msgs::String::ConstPtr& msg);
    void active_places_callback(const std_msgs::String::ConstPtr& msg);
    int check_for_event(string cond);
    void remove_old_elements();
    int doEvalCondition(string cond);
    int evalConditionBuffer(std::string cond);

    int doEvalConditionLiteral(string cond);
    bool EvalConditionWrapper(pnp_msgs::PNPCondition::Request  &req,
             pnp_msgs::PNPCondition::Response &res); 
    bool GetEventStartingWith(pnp_msgs::PNPLastEvent::Request  &req,
             pnp_msgs::PNPLastEvent::Response &res); 
    bool ClearBuffer(pnp_msgs::PNPClearBuffer::Request  &req,
         pnp_msgs::PNPClearBuffer::Response &res);
    bool GetVariableValue(pnp_msgs::PNPGetVariableValue::Request  &req,
         pnp_msgs::PNPGetVariableValue::Response &res);
    bool SetVariableValue(pnp_msgs::PNPSetVariableValue::Request  &req,
         pnp_msgs::PNPSetVariableValue::Response &res);
    
    vector<string> split_condition(string);
    vector<string> get_variables_values(vector<std::string> );
    string get_variable_value(string, string = "");
    string replace_vars_with_values(string);
    bool well_formatted_with_variables(string);
    void update_variable_with_value(string, string);
    void internal_clear_buffer();
    void clear_global_PNPROS_variables() { global_PNPROS_variables.clear(); }

public:
    // Predefined actions
    virtual void none(string params, bool *run);  // no action
    virtual void wait(string params, bool *run);  // wait for <params> seconds
    virtual void waitfor(string params, bool *run); // wait until <params> condition is true, params can be A or not_A
    virtual void restartcurrentplan(string params, bool *run); // restart the current plan
    virtual void stopcurrentplan(string params, bool *run); // stop the current plan
    virtual void initGlobalVariables() { clear_global_PNPROS_variables(); }
};

#endif



