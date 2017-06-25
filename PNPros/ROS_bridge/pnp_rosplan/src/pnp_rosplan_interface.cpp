#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <pnp_translator/digraph_transl.h>

#define TOPIC_PLANTOEXEC "planToExec"

using namespace std;

class PNP_ROSPlan_Interface{
private:
  ros::NodeHandle nh, lnh;
  ros::Subscriber rosplan_sub;
  ros::Publisher rospnp_pub;
  std_msgs::String plan;
  string file_to_write, robot_name, plan_folder;




  //write the plan message
  bool write_plan(){
    if(this->plan.data.empty()){
      ROS_ERROR("Plan is empty!");
      return false;
    }
    ofstream f;

    this->file_to_write = plan_folder+"/plan.txt";

    f.open(file_to_write.c_str());
    if(!f.good()){
      ROS_ERROR("error while opening the file: %s",this->file_to_write.c_str());
      return false;
    }

    f << this->plan.data.c_str();
    f.close();
    return true;
  }

public:



  PNP_ROSPlan_Interface() : lnh("~") {
    rosplan_sub = nh.subscribe("/kcl_rosplan/plan_graph", 10, &PNP_ROSPlan_Interface::plan_callback, this);
    rospnp_pub = nh.advertise<std_msgs::String>(TOPIC_PLANTOEXEC, 10);

    lnh.param("robot_name",robot_name,string("diago"));
    lnh.param("plan_folder",plan_folder,string("plans"));

    ROS_INFO_STREAM("pnp_rosplan:: robot_name = " << robot_name);
    ROS_INFO_STREAM("pnp_rosplan:: plan_folder = " << plan_folder);

    ROS_INFO("pnp_rosplan:: Waiting for plans...");

  }


  void plan_callback(std_msgs::String plan){
//       ROS_INFO("Plan received: \n %s",plan.data.c_str());
      ROS_INFO("Plan received!");
      this->plan = plan;

      //write the plan message at plans/plan.txt
      if(!write_plan()){
        ROS_ERROR("Problems while writing plan");
        return;
      }
      ROS_INFO("Plan written at: %s",this->file_to_write.c_str());

      string plan_out = plan_folder+"/plan.txt";
      string pnml_out = plan_folder+"/";
      //read plan.txt and generate ConditionalPlan
      DigraphTransl tsr(plan_out,pnml_out);
      tsr.create_PNP();
      string plan_name = "AUTOGEN_"+tsr.getPlanName();

      //generate the pnml
      // vector<ConditionalPlan> v = tsr.get_plan();
      // tsr.write_pnml(v);

      
      ROS_INFO("pnp_rosplan:: Sending plan %s to PNP executor...", plan_name.c_str());

      std_msgs::String msg;
      msg.data = plan_name;
      rospnp_pub.publish(msg);
  }




};

int main(int argc, char** argv){
  ros::init(argc, argv, "pnp_rosplan_interface");

  PNP_ROSPlan_Interface interface;

  ros::spin();

  return 0;
}
