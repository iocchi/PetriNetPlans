#include <std_msgs/String.h>

#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/algorithm/string.hpp>

#include <pnp_ros/PNPActionServer.h>

//DO NOT DELETE IT. NEEDED FOR INITILIZING THE global_PNPROS_variables map that otherwhise gives a seg fault at the first insert.
bool first_insert = true;    

PNPActionServer::PNPActionServer() : as(nh, "PNP", false)
{ 
    cond_service = nh.advertiseService("PNPConditionEval",
                &PNPActionServer::EvalConditionWrapper, this);
    getEvent_service = nh.advertiseService("PNPGetEventStartingWith",
                &PNPActionServer::GetEventStartingWith, this);
    clearBuffer_service = nh.advertiseService("PNPClearBuffer",
                &PNPActionServer::ClearBuffer, this);
    getVarValue_service = nh.advertiseService("PNPGetVariableValue",
                &PNPActionServer::GetVariableValue, this);
    setVarValue_service = nh.advertiseService("PNPSetVariableValue",
                &PNPActionServer::SetVariableValue, this);
    
    as.registerGoalCallback(boost::bind(&PNPActionServer::goalCallback, this, _1) );
    // as.registerCancelCallback(boost::bind(&PNPActionServer::cancelCallback, this, _1) );
    event_topic_sub = nh.subscribe("PNPConditionEvent", 10, 
                &PNPActionServer::addEvent_callback, this);  
    global_PNPROS_variables.clear();
}

PNPActionServer::~PNPActionServer() { }

void PNPActionServer::start() {
    // Start the server
    as.start();
    ROS_INFO_STREAM("PNP Action Server started!!!"); 
}

void PNPActionServer::goalCallback(PNPAS::GoalHandle gh){
    boost::mutex::scoped_lock lock(state_mutex);

    current_gh = gh; 
    goal = *current_gh.getGoal();

    // Run action (wait until it finishes)
    //ROS_INFO_STREAM("### Received Goal: " << goal.id << " " << goal.name << " " << 
    //      goal.params << " " << goal.function);
    

    if (goal.function=="start") {
        ROS_DEBUG_STREAM("Starting action " << goal.robotname << " " << goal.name << " " << 
                        goal.params);
        current_gh.setAccepted();
        actionStart(goal.robotname, goal.name, goal.params);
        boost::thread t(
            boost::bind(&PNPActionServer::ActionExecutionThread, this, _1),
            current_gh);
    }
    else if (goal.function=="end") {
        ROS_DEBUG_STREAM("Terminating action " << goal.robotname << " " << goal.name << " " << 
                        goal.params);
        actionEnd(goal.robotname, goal.name, goal.params);
        CancelAction(goal.robotname,goal.name,goal.params);
        current_gh.setAccepted();
        for (int k=0; k<3; k++) { ros::spinOnce(); }
//        boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        current_gh.setSucceeded();
    }
    else if (goal.function=="interrupt") {
        ROS_DEBUG_STREAM("Interrupting action " << goal.robotname << " " << goal.name << " " << 
                        goal.params);
        actionInterrupt(goal.robotname, goal.name, goal.params);
        CancelAction(goal.robotname,goal.name,goal.params);
        current_gh.setAccepted();
        for (int k=0; k<3; k++) { ros::spinOnce(); }
        //boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        current_gh.setSucceeded();
    }

    // wait for actual delivery...
    for (int k=0; k<5; k++) { ros::spinOnce(); }
    // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
}

void PNPActionServer::ActionExecutionThread(PNPAS::GoalHandle gh) {
    pnp_msgs::PNPGoal goal = *gh.getGoal();

    // cout << "### Executing thread for action " << goal.name << " ..." << endl;

    // Sleep and check for interrupt
    try
    {
        { boost::mutex::scoped_lock lock(run_mutex);
          run[goal.robotname+goal.name+goal.params]=true;
        }

        actionExecutionThread(goal.robotname,goal.name,goal.params,
                      &run[goal.robotname+goal.name+goal.params]);

        { boost::mutex::scoped_lock lock(run_mutex);
          run[goal.robotname+goal.name+goal.params]=false;
        }
        // cout << "### Thread " << goal.name << " completed" << endl;
    }
    catch(boost::thread_interrupted&)
    {
        // cout << "### Thread " << goal.name << " interrupted" << endl;
        run[goal.robotname+goal.name+goal.params]=false;
    }
    
    // notify action termination
    gh.setSucceeded();

    // wait for actual delivery...
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
}

void PNPActionServer::CancelAction(string robotname, string action_name, string action_params) {   
    boost::mutex::scoped_lock lock(run_mutex);
    run[robotname+action_name+action_params]=false;
}

bool PNPActionServer::EvalConditionWrapper(pnp_msgs::PNPCondition::Request  &req,
         pnp_msgs::PNPCondition::Response &res)  {

    //cout << "EvalConditionWrapper started " << endl;

	int r0 = -1;
	// This is necessary because multiple calls to the same condition can happen
	if (ConditionCache.find(req.cond) != ConditionCache.end()) {
		r0 = ConditionCache[req.cond];
	}

    int r1 = evalCondition(req.cond);
    int r2 = check_for_event(req.cond);

	//cout << "EvalConditionWrapper  " << req.cond << " : cache / eval / check " << r0 << " " << r1 << " " << r2 ;

    int result=-1;
    if (r0!=-1) result=r0;
	else if (r1!=-1) result=r1; 
	else result=r2;
 
    //TODO implement unknown value of a condition in PNP
    if (result==-1) result=0;

	//cout << " RESULT = " << result << endl;
	
	ConditionCache[req.cond] = result;
    res.truth_value = result;

    return true;
}

bool PNPActionServer::GetEventStartingWith(pnp_msgs::PNPLastEvent::Request  &req,
         pnp_msgs::PNPLastEvent::Response &res)  {
    std::string substring = req.substring;
    std::string eventNameFound = "";
    
    eventBuffer_mutex.lock();
    for (vector<Event>::reverse_iterator rit = eventBuffer.rbegin(); rit!= eventBuffer.rend(); ++rit)
    {
      if(!rit->eventName.compare(0, substring.size(), substring))
      {
        eventNameFound = rit->eventName;
        rit->eventName = string("***") + rit->eventName;
        break;
      }
    }
    eventBuffer_mutex.unlock();

    if(eventNameFound != "")
      res.eventName = eventNameFound;
    else
      res.eventName = "";
    
    return true;
}

bool PNPActionServer::ClearBuffer(pnp_msgs::PNPClearBuffer::Request  &req,
  pnp_msgs::PNPClearBuffer::Response &res){
  
  internal_clear_buffer();
  return true;
}

bool PNPActionServer::GetVariableValue(pnp_msgs::PNPGetVariableValue::Request  &req,
  pnp_msgs::PNPGetVariableValue::Response &res){
  
  res.answer = get_variable_value(req.variable);
  return true;
}

bool PNPActionServer::SetVariableValue(pnp_msgs::PNPSetVariableValue::Request  &req,
  pnp_msgs::PNPSetVariableValue::Response &res){
  
  update_variable_with_value(req.variable, req.value);
  return true;
}

void PNPActionServer::internal_clear_buffer(){
  eventBuffer_mutex.lock();
  for (vector<Event>::reverse_iterator rit = eventBuffer.rbegin(); rit!= eventBuffer.rend(); ++rit)
  {
    if(rit->eventName.compare(0, 3, "***"))
      rit->eventName = string("***") + rit->eventName;
  }
  eventBuffer_mutex.unlock();
}

void PNPActionServer::register_action(string actionname, action_fn_t actionfn) {
  cout << "PNPROS:: REGISTERING ACTION " << actionname << endl;
  global_PNPROS_action_fns[actionname] = boost::bind(*actionfn,_1,_2);

}

void PNPActionServer::register_MRaction(string actionname, MRaction_fn_t actionfn) {
  cout << "PNPROS:: REGISTERING MR ACTION " << actionname << endl;
  global_PNPROS_MRaction_fns[actionname] = boost::bind(*actionfn,_1,_2,_3);
}

boost_action_fn_t PNPActionServer::get_action_fn(string actionname) {
  return global_PNPROS_action_fns[actionname];
}

boost_MRaction_fn_t PNPActionServer::get_MRaction_fn(string actionname) {
  return global_PNPROS_MRaction_fns[actionname];
}

// This function is called in a separate thread for each action to be executed
// Action implementation must check if run is true during execution, 
// if not it must abort the action.
// run must be used read-only!!!
// The function must return only when the action is finished.
void PNPActionServer::actionExecutionThread(string robotname, string action_name, string action_params, bool *run)  {
  bool found=false;
  if (robotname!="") {
    boost_MRaction_fn_t f = get_MRaction_fn(action_name);
    if (f!=NULL){
      found=true;
      if (action_params.find('@') == std::string::npos)
        f(robotname,action_params,run);
      else
        f(robotname, replace_vars_with_values(action_params),run);
    }
    //else
    //  ROS_ERROR_STREAM("??? UNKNOWN Action " << robotname << "#" << action_name << " ??? ");
  }
  if (!found) {
    boost_action_fn_t f = get_action_fn(action_name);
    if (f!=NULL)
    {
      if (action_params.find('@') == std::string::npos)
        f(action_params,run);
      else
        f(replace_vars_with_values(action_params),run);
    }
    else
      ROS_ERROR_STREAM("??? UNKNOWN Action " << action_name << " ??? ");
  }

}

int PNPActionServer::evalCondition(string cond){
  return -1;
}

void PNPActionServer::actionStart(const std::string & robot, const std::string & action, const std::string & params)
{
    // Default implementation does nothing.
  // Clear condition cache, since new event arrived...
  ConditionCache.clear();
}

void PNPActionServer::actionEnd(const std::string & robot, const std::string & action, const std::string & params)
{
    // Default implementation does nothing.
}

void PNPActionServer::actionInterrupt(const std::string & robot, const std::string & action, const std::string & params)
{
    // Default implementation does nothing.
}

#if 0

// This function must return the truth value of a condition.
// It is called each time PNP needs the evaluation of a condition
bool PNPActionServer::evalCondition(string condition) {
  bool r = false;
  //cout << "+++ Evaluating condition " << condition << endl;
  cond_srv.request.cond = condition;
  if (cond_eval_client.call(cond_srv))
  {
    r = cond_srv.response.truth_value;
    //cout << "Condition value: " << r << endl;
  }
  else
  {
    ROS_ERROR("Failed to call service PNPCondition");
  }
  return r;
}

#endif


/* The addEvent_callback is called when a string is published to the addEvent topic.
* When this happens, a new Event is created with the current timestamp and the string received as eventName.
* Such event is later added to the event buffer.
*/
void PNPActionServer::addEvent_callback(const std_msgs::String::ConstPtr& msg){
  Event newEvent;
  time(&newEvent.time);
  newEvent.eventName = msg->data;
  eventBuffer_mutex.lock();
  eventBuffer.push_back(newEvent);
  eventBuffer_mutex.unlock();

  // Clear condition cache, since new event arrived...
  ConditionCache.clear();

#if 0
//   cerr << endl;
   cout << "+++ Added event " << msg->data.c_str() << " to eventBuffer" << endl;
//   for(vector<Event>::const_iterator i = eventBuffer.begin(); i != eventBuffer.end(); ++i)
//      cout << i->eventName << " " << i->time << endl;
//   cout << "buffer current length:" << eventBuffer.size() << endl;
//   cerr << endl;
#endif
}

/* The check_for_event function searches in the eventBuffer for an event that has cond as a nameEvent and 
* has currentTimestamp - eventTimestamp < TIME_THRESHOLD. If found, true is returned and the vector is cleared.
*/
int PNPActionServer::check_for_event(string cond){

//  cout << "   ++ Check for event:  cond = [" << cond << "] " << endl;


  int result=-1;
  
  //checking if _@X_@Y_..._@Z is contained in the condition. If so, appropriately instantiating the variables in global_PNPROS_variables
  if (well_formatted_with_variables(cond))
  {    
    vector<std::string> splitted_condition = split_condition(cond);
    vector<std::string> variable_values = get_variables_values(splitted_condition);

    if(variable_values.size() + 1 == splitted_condition.size())
    {
      cond = splitted_condition[0].substr(0,splitted_condition[0].length()-1);
      for (unsigned int i = 0; i < variable_values.size(); ++i ){
        update_variable_with_value(splitted_condition[i+1], variable_values[i]);
        cond += "_" + variable_values[i];
      }
    }
  }

  time_t current_time;
  time(&current_time);
  
  eventBuffer_mutex.lock();


//  cout << "   -- Checking cond = [" << cond << "] ... " << endl;

  // ORIG
  //  for(vector<Event>::iterator i = eventBuffer.begin(); i != eventBuffer.end(); ++i) {

    // LI FIX: check !!!!  From most recent!!!
  for(vector<Event>::iterator i = eventBuffer.end(); i != eventBuffer.begin(); ) {

    i--;
	time_t dtime = (current_time - i->time);
	
	// cout << "Searching for condition " << i->eventName << " :: dtime = " << (current_time - i->time) << endl;

    if ((current_time - i->time) <= TIME_THRESHOLD) {

//	  cout << "       -- cmp with buffer [" << i->eventName << "]  - dtime = " << dtime << endl;

      if (i->eventName == cond){
        i->eventName = string("***") + i->eventName;
        
           //cerr << endl;
           //cout << "### Found event " << cond << " in eventBuffer - True" << endl;
           //for(vector<Event>::const_iterator i = eventBuffer.begin(); i != eventBuffer.end(); ++i)
           //	   cout << i->eventName << " " << i->time << endl;
           //cout << "buffer current length:" << eventBuffer.size() << endl;
           //cerr << endl;
        
        result=1;
        break;
      }
      else if (i->eventName == "!"+cond)
      {
        i->eventName = string("***") + i->eventName;

           //cerr << endl;
           //cout << "### Found event " << cond << " in eventBuffer - False" << endl;
           //for(vector<Event>::const_iterator i = eventBuffer.begin(); i != eventBuffer.end(); ++i)
           //   cout << i->eventName << " " << i->time << endl;
           //cout << "buffer current length:" << eventBuffer.size() << endl;
           //cerr << endl;

        result=0;
        break;
      }
    }
  }
  
  eventBuffer_mutex.unlock();
  
  remove_old_elements();
  
  return result;
}

void PNPActionServer::remove_old_elements(){
  time_t current_time;
  time(&current_time);
  size_t upper_bound_elments_to_remove = 0;
  
  eventBuffer_mutex.lock();
  for(size_t i = 0; i < eventBuffer.size(); ++i)
  {
    if((current_time - eventBuffer.at(i).time) < REMEMBERING_TIME)
    {
      upper_bound_elments_to_remove = i;
      break;
    }
  }
  
  if(upper_bound_elments_to_remove != 0 && upper_bound_elments_to_remove < eventBuffer.size())
    eventBuffer.erase(eventBuffer.begin(),eventBuffer.begin() + upper_bound_elments_to_remove);
  eventBuffer_mutex.unlock();
}

bool PNPActionServer::well_formatted_with_variables(std::string cond){
  bool well_formatted = true;
  
  for(unsigned int i = 0; i < cond.length(); ++i)
  {
    if (cond[i] == '@')
    {
      for (unsigned j = i -1; j < cond.length()-1; ++j)
      {
        if (cond[j] == '_' && cond[j+1] != '@')
        {
          cerr << "\033[22;31;1m??? Wrongly formatted transition name " << cond << " ???\033[0m" << endl;
          throw new runtime_error("\033[22;31;1m??? Wrongly formatted transition name ???\033[0m");
        }
      }
      break;
    }
  
    else if (i == cond.length() -1)
      well_formatted = false;
  }
  
  return well_formatted;
}

vector<std::string> PNPActionServer::split_condition(string cond){
  vector<std::string> splitted_condition;
  boost::split(splitted_condition, cond, boost::is_any_of("@"));
  for (unsigned int i =1; i < splitted_condition.size()-1; ++i)
    splitted_condition[i].erase(splitted_condition[i].length()-1,1);
  
  return splitted_condition;
}

vector<std::string> PNPActionServer::get_variables_values(vector<std::string> splitted_condition){  
  vector<std::string> variables_values;
  
  time_t current_time;
  time(&current_time);
  
  eventBuffer_mutex.lock();
  for (vector<Event>::reverse_iterator rit = eventBuffer.rbegin(); rit!= eventBuffer.rend(); ++rit)
  {     
    //checking that the event is not too old
    if ((current_time - rit->time) > TIME_THRESHOLD)
      break;
    else if (rit->eventName.substr(0, splitted_condition[0].length()) == splitted_condition[0])
    {
      std::string truncated_event = rit->eventName.substr(splitted_condition[0].length(), rit->eventName.length()-splitted_condition[0].length());
      
      if (truncated_event.find("_@") == std::string::npos)
        boost::split(variables_values, truncated_event, boost::is_any_of("_"));
      
      if(variables_values.size() + 1 == splitted_condition[0].size())
      {
        rit->eventName = string("***") + rit->eventName;
        break;
      }
    }
  }
  eventBuffer_mutex.unlock();

  return variables_values;
}

string PNPActionServer::get_variable_value(string var_name, string default_value){
  
  if(global_PNPROS_variables.find(var_name) != global_PNPROS_variables.end())
      return global_PNPROS_variables[var_name];
  else if (default_value != "") {
    update_variable_with_value(var_name, default_value);
    ROS_WARN("Variable %s not initialized, instantiating it to the input default value", var_name.c_str());
    return default_value;
  }
  else {
    ROS_ERROR("??? Variable %s not initialized ???", var_name.c_str());
    throw new runtime_error("??? Variable not initialized ???");
  }
}

void PNPActionServer::update_variable_with_value(string var, string value){
  const std::map<std::string,std::string>::iterator& it = global_PNPROS_variables.find(var);

  if (it != global_PNPROS_variables.end())
  {
    it->second = value;
    cerr << "updated " << var << " to value " << value << endl;
  }
  else{
    //serve per inizializzare la mappa che altrimenti va in seg fault al primo insert
    if(first_insert){
      global_PNPROS_variables.clear();
      first_insert = false;
    }
    cerr << "Instantiated variable " << var << " to value " << value <<endl;
    global_PNPROS_variables.insert(make_pair(var, value));
  } 
}

std::string PNPActionServer::replace_vars_with_values(std::string params){
  vector<std::string> splitted_parameters;
  boost::split(splitted_parameters, params, boost::is_any_of("_"));
  std::string new_params;
  
  for (unsigned int i = 0; i < splitted_parameters.size(); ++i)
  {
    if (splitted_parameters[i][0] == '@')
    {
      std::string key = splitted_parameters[i].substr(1,splitted_parameters[i].length()-1);
      if(global_PNPROS_variables.find(key) != global_PNPROS_variables.end())
        splitted_parameters[i] = global_PNPROS_variables[key];
      else
      {
        cerr << "\033[22;31;1m??? Variable " << key << " not initialized ???\033[0m" << endl;
        throw new runtime_error("\033[22;31;1m??? Wrongly formatted transition name ???\033[0m");
      }
        
    }
    
    new_params += "_" + splitted_parameters[i];
  }

  return new_params.substr(1, new_params.length()-1);
}

#if 0
void PNPActionServer::cancelCallback(PNPAS::GoalHandle gh)
{
    // boost::mutex::scoped_lock lock(state_mutex);

    // See if our current goal is the one that needs to be cancelled
    if (current_gh != gh)
    {
        ROS_DEBUG("Got a cancel request for some other goal. Ignoring it");
        return;
    }
    current_gh.setCanceled();
}
#endif


