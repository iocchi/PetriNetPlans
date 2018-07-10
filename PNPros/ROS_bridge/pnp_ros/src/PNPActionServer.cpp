#include <std_msgs/String.h>

#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/algorithm/string.hpp>

#include <pnp_ros/PNPActionServer.h>

//DO NOT DELETE IT. NEEDED FOR INITILIZING THE global_PNPROS_variables map that otherwhise gives a seg fault at the first insert.
bool first_insert = true;    


PNPActionServer::PNPActionServer() : as(nh, "PNP", false)
{

    cond_service = nh.advertiseService(SRV_PNPCONDITIONEVAL,
                &PNPActionServer::EvalConditionWrapper, this);
    getEvent_service = nh.advertiseService(SRV_PNPGETEVENT,
                &PNPActionServer::GetEventStartingWith, this);
    clearBuffer_service = nh.advertiseService(SRV_PNPCLEARBUFFER,
                &PNPActionServer::ClearBuffer, this);
    getVarValue_service = nh.advertiseService(SRV_PNPGETVAR,
                &PNPActionServer::GetVariableValue, this);
    setVarValue_service = nh.advertiseService(SRV_PNPSETVAR,
                &PNPActionServer::SetVariableValue, this);
    
    as.registerGoalCallback(boost::bind(&PNPActionServer::goalCallback, this, _1) );
    // as.registerCancelCallback(boost::bind(&PNPActionServer::cancelCallback, this, _1) );

    event_topic_sub = nh.subscribe(TOPIC_PNPCONDITION, 10,
                &PNPActionServer::addEvent_callback, this);  

    active_places_sub = nh.subscribe(TOPIC_PNPACTIVEPLACES, 10,
                &PNPActionServer::active_places_callback, this);

    plantoexec_pub = nh.advertise<std_msgs::String>(TOPIC_PLANTOEXEC, 10);

    global_PNPROS_variables.clear();

    register_action("wait",&PNPActionServer::wait,this);
    register_action("waitfor",&PNPActionServer::waitfor,this);
    register_action("restartcurrentplan",&PNPActionServer::restartcurrentplan,this);
    register_action("stopcurrentplan",&PNPActionServer::stopcurrentplan,this);
    register_action("unknownvar",&PNPActionServer::unknownvar,this);
    register_action("setvar",&PNPActionServer::setvar,this);
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
          starttime[goal.robotname+goal.name]=ros::Time::now().toSec();
          string atom="timeout_"+goal.name;
          ConditionCache[atom]=0;
          cout << "Action " << goal.name << " " << goal.params << " started at time: " << starttime[goal.robotname+goal.name] << endl;
        }

        actionExecutionThread(goal.robotname,goal.name,goal.params,
                      &run[goal.robotname+goal.name+goal.params]);

        { boost::mutex::scoped_lock lock(run_mutex);
          run[goal.robotname+goal.name+goal.params]=false;
          starttime[goal.robotname+goal.name]=-1;
        }
        // cout << "### Thread " << goal.name << " completed" << endl;
    }
    catch(boost::thread_interrupted&)
    {
        // cout << "### Thread " << goal.name << " interrupted" << endl;
        run[goal.robotname+goal.name+goal.params]=false;
        starttime[goal.robotname+goal.name]=-1;
    }
    
    // notify action termination
    gh.setSucceeded();

    // wait for actual delivery...
    for (int k=0; k<5; k++) { ros::spinOnce(); }
    //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

void PNPActionServer::CancelAction(string robotname, string action_name, string action_params) {   
    boost::mutex::scoped_lock lock(run_mutex);
    run[robotname+action_name+action_params]=false;
    starttime[goal.robotname+goal.name]=-1;
}


string PNPActionServer::set_variables_from_events(string cond) {

    vector<std::string> splitted_condition = split_condition(cond);
    vector<std::string> variable_values = get_variables_values(splitted_condition);

    cond = variable_values[0];
    for (unsigned int i = 1; i < variable_values.size(); i++) {
      cond += "_" + variable_values[i];
    }

    return cond;
}



// Evaluate an atomic condition
int PNPActionServer::doEvalCondition(string cond) {

    int result=-1;

    //cout << "-- EvalCondition (before var. replace) " << cond << " ... " << endl;
    cond = replace_vars_with_values(cond);
    cond = set_variables_from_events(cond);
    //cout << "-- EvalCondition (after var. replace) " << cond << " ... " << endl;

    // check special condition timeout_<actionname>_<value>
    size_t pt = cond.find("timeout");
    if (pt!=string::npos) {
        size_t pt2 = cond.find_last_of("_");
        string act = goal.robotname+cond.substr(pt+8,pt2-(pt+8));
        double val_timeout = atof(cond.substr(pt2+1).c_str());
        // cout << "Evaluating timeout condition for action [" << act << "]" << endl;
        double d=-1;
        if (starttime.find(act)!=starttime.end())
            d = starttime[act];
        if (d<0) {
            ROS_WARN_STREAM("Timeout condition:: action " << act << " not running!!!");
        }
        else {
            // cout << "           start time = ... " << starttime[act] << " now = " << ros::Time::now().toSec() << endl;
            double dt = ros::Time::now().toSec() - starttime[act];
            if (dt>val_timeout) {
                result=1;
                ROS_INFO_STREAM("Timeout condition:: " << cond << " TRUE ");
            }
        }
    }
    else {
        vector<string> toks;
        boost::split(toks,cond,boost::is_any_of("_"));
        if ((toks[0]=="equals" || toks[0]=="equal") && toks.size()==3) { 
            result = toks[1]==toks[2];
            ROS_INFO("evalCondition: equals: %s %s -> %d", toks[1].c_str(),toks[2].c_str(),result);
        }
    }

    
    if (result<0) {

        int r0=-1,r1,r2,r3;

        // This is necessary because multiple calls to the same condition can happen
        if (ConditionCache.find(cond) != ConditionCache.end()) {
            r0 = ConditionCache[cond];
        }

        r1 = evalCondition(cond); // overwritten by subclass
        r2 = check_for_event(cond);
        r3 = evalConditionBuffer(cond); // check condition param buffer

        //cout << "-- EvalCondition " << cond << " : cache / eval / check " << r0 << " " << r1 << " " << r2 ;

        if (r0!=-1) result=r0; // cached value has priority
        else if (r1!=-1) result=r1;
        else if (r2!=-1) result=r2;
        else result=r3;

        //TODO implement unknown value of a condition in PNP
        if (result==-1) result=0;

        ConditionCache[cond] = result;
    }

    //cout << "-- doEvalCondition RESULT = " << result << endl;
    return result;
}


// // Evaluate a literal condition (A or not_A)
int PNPActionServer::doEvalConditionLiteral(string cond) {

    bool neg=false;
    string atom=cond; // assume positive atom
    // check negative conditions
    if (cond.substr(0,4)=="not_") {
        atom = cond.substr(4); neg=true;
    }

    int r = doEvalCondition(atom);

    // apply negation if negative atom
    if (neg and (r==0 || r==1)) r = 1-r;

    return r;
}


// ENTRY POINT FOR CONDITION EVALUATION
bool PNPActionServer::EvalConditionWrapper(pnp_msgs::PNPCondition::Request  &req,
         pnp_msgs::PNPCondition::Response &res)  {

    // ROS_INFO_STREAM("-- EvalConditionWrapper started with cond: " << req.cond);

    ConditionCache.clear();  // reset cache every service call
    res.truth_value = doEvalCondition(req.cond);

    // ROS_INFO_STREAM("-- EvalConditionWrapper ended with result: " << res.truth_value);

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
	//std::cout << "Consumed1 event " << rit->eventName << std::endl;
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
      else if ((action_name == "unknownvar") || (action_name == "setvar"))
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
      else if ((action_name == "unknownvar") || (action_name == "setvar"))
        f(action_params,run);
      else
        f(replace_vars_with_values(action_params),run);
    }
    else
      ROS_ERROR_STREAM("??? UNKNOWN Action " << action_name << " ??? ");
  }

  actionEnd(robotname, action_name, action_params);

}

int PNPActionServer::evalCondition(string cond){
    return -1;
}

int PNPActionServer::evalConditionBuffer(string cond){
    int r=-1,v;

    string rospar = PARAM_PNPCONDITIONBUFFER + cond;

    // cout << "-- EvalConditionBuffer " << cond << " param " << rospar << endl;
    

    if (cond.find('@') != std::string::npos) {
        return -1;
    }

    if (ros::param::get(rospar,v))
        r=v;

    // cout << "-- EvalConditionBuffer " << cond << " param " << rospar << " " << r << endl;

    return r;
}

void PNPActionServer::actionStart(const std::string & robot, const std::string & action, const std::string & params)
{
  // Clear condition cache, since new event arrived...
  ConditionCache.clear();
  // Set status parameter
  stringstream ssbuf;
  ssbuf << PARAM_PNPACTIONSTATUS << action;
  ros::param::set(ssbuf.str(),"run");
}

void PNPActionServer::actionEnd(const std::string & robot, const std::string & action, const std::string & params)
{
  // Set status parameter
  stringstream ssbuf;
  ssbuf << PARAM_PNPACTIONSTATUS << action;
  ros::param::set(ssbuf.str(),"end");
}

void PNPActionServer::actionInterrupt(const std::string & robot, const std::string & action, const std::string & params)
{
  // Set status parameter
  stringstream ssbuf;
  ssbuf << PARAM_PNPACTIONSTATUS << action;
  ros::param::set(ssbuf.str(),"interrupt");
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

/* This function is called when a PNP step is over.
 * Here it is used just to clear the cache of the conditions 
 */
void PNPActionServer::active_places_callback(const std_msgs::String::ConstPtr& msg)
{
    ConditionCache.clear();
    if (msg->data=="init") {  // LI: (msg->data.find("init;")!=string::npos) 
        clear_global_PNPROS_variables();
        ROS_INFO("Init place -> clear PNP global variables.");
    }
}

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

  // cout << "   ++ Check for event:  cond = [" << cond << "] " << endl;


  int result=-1;

#if 0  
   -- MOVED IN PNPActionServer::doEvalCondition(string cond) 
  //checking if _@X_@Y_..._@Z is contained in the condition. If so, appropriately instantiating the variables in global_PNPROS_variables
  if (well_formatted_with_variables(cond))
  {    
    vector<std::string> splitted_condition = split_condition(cond);
    vector<std::string> variable_values = get_variables_values(splitted_condition);

    if(variable_values.size() + 1 == splitted_condition.size())
    {
      cond = splitted_condition[0];//LJ: Why remove _ to add _ later? Was: .substr(0,splitted_condition[0].length()-1);
// corrected so that a variable is not necessarily preceded by '_'
      for (unsigned int i = 0; i < variable_values.size(); ++i ){
        update_variable_with_value(splitted_condition[i+1], variable_values[i]);
        cond += /*"_" +*/ variable_values[i];
      }
    }
  }
#endif

  time_t current_time;
  time(&current_time);
  
  eventBuffer_mutex.lock();


  //cout << "   -- Checking cond = [" << cond << "] ... " << endl;

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
	//std::cout << "Consumed2 event " << i->eventName << std::endl;
           //cerr << endl;
           //cout << "### Found event " << cond << " in eventBuffer - True" << endl;
           //for(vector<Event>::const_iterator i = eventBuffer.begin(); i != eventBuffer.end(); ++i)
           //	   cout << i->eventName << " " << i->time << endl;
           //cout << "buffer current length:" << eventBuffer.size() << endl;
           //cerr << endl;
        
        result=1;
        break;
      }
      else if ( (i->eventName == "!"+cond) || (i->eventName == "not_"+cond) )
      {
        i->eventName = string("***") + i->eventName;
	//std::cout << "Consumed3 event " << i->eventName << std::endl;

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
  
  // cout << "   ** END check_for_event" << endl;

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

 // cerr << "*** condition to split: " << cond << endl;
#if 1
  vector<std::string> splitted_condition;
  boost::split(splitted_condition, cond, boost::is_any_of("_"));
#else
  vector<std::string> splitted_condition;
  boost::split(splitted_condition, cond, boost::is_any_of("@"));
  for (unsigned int i =1; i < splitted_condition.size()-1; ++i)
    splitted_condition[i].erase(splitted_condition[i].length()-1,1);
#endif
  /*  cerr << "*** splitted condition: " << endl;
    for (unsigned int i=0; i < splitted_condition.size(); ++i)
        cerr << "     " << splitted_condition[i] << endl;
  */
  return splitted_condition;
}

vector<std::string> PNPActionServer::get_variables_values(vector<std::string> splitted_condition) {
    vector<std::string> variables_values;
    variables_values.resize(splitted_condition.size());

    bool bounded = true; // term is bounded: i.e., no variables
    for (size_t i=0; i<splitted_condition.size(); i++) {
        // cerr << "   - get_variables_values " << splitted_condition[i];
        if (splitted_condition[i][0]=='@') // variable
            variables_values[i] = get_variable_value(splitted_condition[i].substr(1), splitted_condition[i]);
        else // term (not variable)
            variables_values[i] = splitted_condition[i];
        if (variables_values[i][0]=='@') bounded = false;
        // cerr << " = " << variables_values[i] << endl;
    }


  if (!bounded) { // search for events to set variables

      time_t current_time;
      time(&current_time);
      
      eventBuffer_mutex.lock();
      for (vector<Event>::reverse_iterator rit = eventBuffer.rbegin(); rit!= eventBuffer.rend(); ++rit)
      {     
        //checking that the event is not too old
        if ((current_time - rit->time) > TIME_THRESHOLD)
          break;
        else {

            // cerr << "   === event " << rit->eventName << endl;
            vector<std::string> event_values;
            boost::split(event_values, rit->eventName, boost::is_any_of("_"));

            if (event_values.size() == splitted_condition.size()) {

              for (size_t i=0; i<splitted_condition.size(); i++) {

#if 1
                // cerr << "   +++ compare " << event_values[i] << " " << splitted_condition[i] << endl;

                if (splitted_condition[i][0]=='@') { // variable to set

                    string var = splitted_condition[i].substr(1);
                    // cerr << "   +++ set var " << var << " = " << event_values[i] << endl;
                    update_variable_with_value(var, event_values[i]);

                }
                else if (event_values[i] != splitted_condition[i]) {
                    break;
                }

#else

                if (rit->eventName.substr(0, splitted_condition[i].length()) == splitted_condition[i]) {
                  std::string truncated_event = rit->eventName.substr(splitted_condition[i].length(), rit->eventName.length()-splitted_condition[0].length());

                  cerr << "   - truncated event " << truncated_event << endl;
              
                  if (truncated_event.find("_@") == std::string::npos) {
                    boost::split(variables_values, truncated_event, boost::is_any_of("_"));
                    cerr << "   - truncated event " << variables_values[i] << endl;
                  }
                }

          /* What is the semantic of this ??? if the number of found values is the same as the prefix-length consume and return ?
          if(variables_values.size() + 1 == splitted_condition[0].size())
          {
            rit->eventName = string("***") + rit->eventName;
	    //std::cout << "Consumed4 event " << rit->eventName << std::endl;
            break;
          }
          */
#endif
            } // for condition split
          } // if
        } // else
      } // for event
      eventBuffer_mutex.unlock();
  }

  return variables_values;
}

// reset given variable
void PNPActionServer::reset_variable(string var_name) {
    map<string,string>::iterator i = global_PNPROS_variables.find(var_name);
    if ( i != global_PNPROS_variables.end() )
      global_PNPROS_variables.erase(i);
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
    cerr << "Updated var " << var << " to value " << value << endl;
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

#if 1
// LI: old version
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
        ROS_WARN("Variable %s not initialized, passing it to the action", key.c_str());
        // cerr << "\033[22;31;1m??? Variable " << key << " not initialized ???\033[0m" << endl;
        // throw new runtime_error("\033[22;31;1m??? Wrongly formatted transition name ???\033[0m");
      }
        
    }
    
    new_params += "_" + splitted_parameters[i];
  }

  return new_params.substr(1, new_params.length()-1);
}
#else
// LJ to test : allow for variables inside the parameters
std::string PNPActionServer::replace_vars_with_values(std::string params){
  vector<std::string> splitted_parameters;
  boost::split(splitted_parameters, params, boost::is_any_of("_"));
  std::string new_params;
  
  for (unsigned int i = 0; i < splitted_parameters.size(); ++i)
  {
    vector<std::string> splitted_parameter;
    boost::split(splitted_parameter, splitted_parameters[i], boost::is_any_of("@"));
    splitted_parameters[i] = splitted_parameter[0]; // first element as a basis
    for (unsigned int j = 1; j < splitted_parameter.size(); ++j)
    {
      std::string key = splitted_parameter[j];
      if(global_PNPROS_variables.find(key) != global_PNPROS_variables.end())
        splitted_parameter[j] = global_PNPROS_variables[key];
      else
      {
        ROS_WARN("Variable %s not initialized, passing it to the action", key.c_str());
        // cerr << "\033[22;31;1m??? Variable " << key << " not initialized ???\033[0m" << endl;
        // throw new runtime_error("\033[22;31;1m??? Wrongly formatted transition name ???\033[0m");
      }
      splitted_parameters[i] += splitted_parameter[j];  
    }
    
    new_params += "_" + splitted_parameters[i];
  }

  return new_params.substr(1, new_params.length()-1);
}
#endif


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


// PREDEFINED ACTIONS

void PNPActionServer::none(string params, bool *run)
{
    ROS_INFO_STREAM("### Executing no action ###");
}

void PNPActionServer::wait(string params, bool *run)
{
    ROS_INFO_STREAM("### Executing Wait action for " << params << " seconds ... ");

    double wait_sec = atof(params.c_str()); // if wrong string format atof returns 0.0
    double sleepunit = 0.25;
    int count=(int)(wait_sec/sleepunit)+1;
    while (*run && count-->0)
        ros::Duration(sleepunit).sleep();

    ROS_INFO_STREAM("### Wait " << params << ((*run)?" Completed":" Aborted"));
}

void PNPActionServer::waitfor(string params, bool *run)
{
    ROS_INFO_STREAM("### Executing WaitFor action with parameter " << params << " ... ");

    while (*run && (doEvalConditionLiteral(params)!=1))
        ros::Duration(0.2).sleep();

    ROS_INFO_STREAM("### WaitFor " << params << ((*run)?" Completed":" Aborted"));
}


void PNPActionServer::restartcurrentplan(string params, bool *run)
{
    ROS_INFO_STREAM("### Executing Restart current plan " << params << " ... ");

    // publish planToExec to start the plan
    string planname = "<currentplan>";
    std_msgs::String s;
    s.data = planname;
    plantoexec_pub.publish(s); // restart the plan

    ROS_INFO_STREAM("### Restart " << params << ((*run)?" Completed":" Aborted"));
}

void PNPActionServer::stopcurrentplan(string params, bool *run)
{
    ROS_INFO_STREAM("### Executing Stop current plan " << params << " ... ");

    // publish planToExec to start the plan
    string planname = "stop";
    std_msgs::String s;
    s.data = planname;
    plantoexec_pub.publish(s); // restart the plan

    ROS_INFO_STREAM("### Stop " << params << ((*run)?" Completed":" Aborted"));
}

// set variables to unknown
void PNPActionServer::unknownvar(string params, bool *run)
{
    ROS_INFO_STREAM("### Executing unknownvar " << params << " ... ");

    vector<std::string> vp;
    boost::split(vp, params, boost::is_any_of("_"));
    for (size_t i=0; i<vp.size(); i++) {
        if (vp[i][0]='@') {
            string var = vp[i].substr(1);
            cout << "    Reset variable " << var << endl;
            reset_variable(var);
        }
    }

    ROS_INFO_STREAM("### Unknownvar " << params << ((*run)?" Completed":" Aborted"));
}

// set variable to value
void PNPActionServer::setvar(string params, bool *run)
{
    ROS_INFO_STREAM("### Executing setvar " << params << " ... ");

    vector<std::string> vp;
    boost::split(vp, params, boost::is_any_of("_"));
    for (size_t i=0; i<vp.size()-1; i++) {
        if (vp[i][0]='@') {
            string var = vp[i].substr(1);
            string value = vp[i+1];
            cout << "    Set variable " << var << " = " << value << endl;
            update_variable_with_value(var, value);
        }
    }

    ROS_INFO_STREAM("### setvar " << params << ((*run)?" Completed":" Aborted"));
}

